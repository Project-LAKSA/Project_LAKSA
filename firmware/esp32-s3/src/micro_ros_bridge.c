#include "micro_ros_bridge.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_usbcdc_transport.h"
#include "esp32_config.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "status_led.h"

#include <laksa_interfaces/msg/drive_command.h>
#include <laksa_interfaces/msg/vehicle_state.h>
#include <laksa_interfaces/msg/vesc_state.h>
#include <laksa_interfaces/srv/get_vehicle_state.h>
#include <laksa_interfaces/srv/set_drive_command.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/string_functions.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <std_msgs/msg/bool.h>

#define MICRO_ROS_TASK_STACK_SIZE 20000
#define MICRO_ROS_TASK_PRIORITY 5
#define MICRO_ROS_EXECUTOR_HANDLES 4
#define MICRO_ROS_IMU_PERIOD_MS 20
#define MICRO_ROS_MAG_PERIOD_MS 50
#define MICRO_ROS_STATE_PERIOD_MS 100
#define MICRO_ROS_AGENT_PING_PERIOD_MS 500
#define MICRO_ROS_AGENT_PING_TIMEOUT_MS 100
#define MICRO_ROS_AGENT_PING_ATTEMPTS 1
#define MICRO_ROS_RETRY_PERIOD_MS 500
#define MICRO_ROS_PI 3.14159265358979323846f
#define MICRO_ROS_UT_TO_T 0.000001f

typedef enum {
    BRIDGE_WAITING_FOR_AGENT,
    BRIDGE_AGENT_AVAILABLE,
    BRIDGE_AGENT_CONNECTED,
    BRIDGE_AGENT_DISCONNECTED,
} bridge_connection_state_t;

typedef struct {
    micro_ros_bridge_config_t hardware;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rcl_node_t node;
    rclc_executor_t executor;
    rcl_publisher_t imu_publisher;
    rcl_publisher_t mag_publisher;
    rcl_publisher_t vesc_publisher;
    rcl_publisher_t state_publisher;
    rcl_subscription_t command_subscription;
    rcl_subscription_t brake_subscription;
    rcl_service_t set_command_service;
    rcl_service_t get_state_service;
    sensor_msgs__msg__Imu imu_message;
    sensor_msgs__msg__MagneticField mag_message;
    laksa_interfaces__msg__VescState vesc_message;
    laksa_interfaces__msg__VehicleState state_message;
    laksa_interfaces__msg__DriveCommand command_message;
    std_msgs__msg__Bool brake_message;
    laksa_interfaces__srv__SetDriveCommand_Request set_request;
    laksa_interfaces__srv__SetDriveCommand_Response set_response;
    laksa_interfaces__srv__GetVehicleState_Request get_request;
    laksa_interfaces__srv__GetVehicleState_Response get_response;
    TickType_t last_drive_command_tick;
    bool brake_requested;
    bool steering_failsafe_active;
    bool entities_created;
} micro_ros_bridge_t;

static const char *TAG = "micro_ros";
static micro_ros_bridge_t bridge;
static tinyusb_cdcacm_itf_t cdc_port = TINYUSB_CDC_ACM_0;

static void consume_rcl_result(rcl_ret_t result)
{
    (void)result;
}

static float wheel_diameter_m(void)
{
    return (float)CONFIG_LAKSA_WHEEL_DIAMETER_MM / 1000.0f;
}

static float gear_reduction(void)
{
    return (float)CONFIG_LAKSA_GEAR_REDUCTION_X1000 / 1000.0f;
}

static float max_steering_angle_rad(void)
{
    return (float)CONFIG_LAKSA_MAX_STEERING_ANGLE_MRAD / 1000.0f;
}

static bool speed_to_erpm(float speed_mps, int32_t *erpm)
{
    if (erpm == NULL || !isfinite(speed_mps)) {
        return false;
    }

    float circumference_m = MICRO_ROS_PI * wheel_diameter_m();
    float value = speed_mps * 60.0f * gear_reduction() *
                  (float)CONFIG_LAKSA_MOTOR_POLE_PAIRS /
                  circumference_m * (float)CONFIG_LAKSA_DRIVE_FORWARD_SIGN;
    /* Validate the rounded command so a float32 ROS round trip at exactly the
     * configured limit is not rejected by a sub-eRPM conversion error. */
    if (!isfinite(value) ||
        fabsf(value) > (float)VESC_MAX_ABS_RPM + 0.5f) {
        return false;
    }

    int32_t rounded_erpm = (int32_t)lroundf(value);
    if (rounded_erpm < -VESC_MAX_ABS_RPM ||
        rounded_erpm > VESC_MAX_ABS_RPM) {
        return false;
    }

    *erpm = rounded_erpm;
    return true;
}

static float erpm_to_speed(float erpm)
{
    float circumference_m = MICRO_ROS_PI * wheel_diameter_m();
    return erpm * circumference_m /
           (60.0f * gear_reduction() * (float)CONFIG_LAKSA_MOTOR_POLE_PAIRS) *
           (float)CONFIG_LAKSA_DRIVE_FORWARD_SIGN;
}

static bool steering_rad_to_servo(float steering_rad, uint8_t *servo_angle_deg)
{
    if (servo_angle_deg == NULL || !isfinite(steering_rad) ||
        fabsf(steering_rad) > max_steering_angle_rad()) {
        return false;
    }

    float normalized = fabsf(steering_rad) / max_steering_angle_rad();
    float servo_angle;
    if (steering_rad >= 0.0f) {
        servo_angle = (float)STEERING_CENTER_DEG -
                      normalized *
                          (float)(STEERING_CENTER_DEG - STEERING_LEFT_COMMAND_LIMIT_DEG);
    } else {
        servo_angle = (float)STEERING_CENTER_DEG +
                      normalized *
                          (float)(STEERING_RIGHT_COMMAND_LIMIT_DEG - STEERING_CENTER_DEG);
    }
    *servo_angle_deg = (uint8_t)lroundf(servo_angle);
    return true;
}

static float servo_to_steering_rad(uint8_t servo_angle_deg)
{
    if (servo_angle_deg <= STEERING_CENTER_DEG) {
        return ((float)(STEERING_CENTER_DEG - servo_angle_deg) /
                (float)(STEERING_CENTER_DEG - STEERING_LEFT_COMMAND_LIMIT_DEG)) *
               max_steering_angle_rad();
    }
    return -((float)(servo_angle_deg - STEERING_CENTER_DEG) /
             (float)(STEERING_RIGHT_COMMAND_LIMIT_DEG - STEERING_CENTER_DEG)) *
           max_steering_angle_rad();
}

static uint8_t apply_drive_command(float speed_mps, float steering_angle_rad, bool brake)
{
    int32_t erpm;
    uint8_t servo_angle;
    if (!speed_to_erpm(speed_mps, &erpm)) {
        return 1;
    }
    if (!steering_rad_to_servo(brake ? 0.0f : steering_angle_rad, &servo_angle)) {
        return 2;
    }
    if (vesc_uart_set_drive(bridge.hardware.vesc, erpm, brake) != ESP_OK) {
        return 3;
    }
    if (steering_control_set_target(bridge.hardware.steering, servo_angle) != ESP_OK) {
        (void)vesc_uart_set_drive(bridge.hardware.vesc, 0, true);
        return 4;
    }
    bridge.last_drive_command_tick = xTaskGetTickCount();
    bridge.steering_failsafe_active = brake;
    return 0;
}

static void apply_actuator_failsafe(void)
{
    (void)vesc_uart_set_drive(bridge.hardware.vesc, 0, true);
    (void)steering_control_center(bridge.hardware.steering);
    bridge.steering_failsafe_active = true;
}

static builtin_interfaces__msg__Time ros_time_now(void)
{
    int64_t nanoseconds = rmw_uros_epoch_synchronized()
                              ? rmw_uros_epoch_nanos()
                              : esp_timer_get_time() * 1000LL;
    builtin_interfaces__msg__Time stamp = {
        .sec = (int32_t)(nanoseconds / 1000000000LL),
        .nanosec = (uint32_t)(nanoseconds % 1000000000LL),
    };
    return stamp;
}

static void read_imu(bno08x_adapter_vec3_t *accel,
                     bno08x_adapter_vec3_t *gyro,
                     bno08x_adapter_vec3_t *mag_ut,
                     bno08x_adapter_quat_t *rotation,
                     bool *has_accel,
                     bool *has_gyro,
                     bool *has_mag,
                     bool *has_rotation)
{
    *has_accel = false;
    *has_gyro = false;
    *has_mag = false;
    *has_rotation = false;
    if (bridge.hardware.imu == NULL) {
        return;
    }

    if (bridge.hardware.hardware_mutex != NULL) {
        xSemaphoreTake(bridge.hardware.hardware_mutex, portMAX_DELAY);
    }
    *has_accel = bno08x_adapter_get_acceleration(bridge.hardware.imu, accel) == ESP_OK;
    *has_gyro = bno08x_adapter_get_angular_velocity(bridge.hardware.imu, gyro) == ESP_OK;
    *has_mag = bno08x_adapter_get_magnetic_field(bridge.hardware.imu, mag_ut) == ESP_OK;
    *has_rotation = bno08x_adapter_get_rotation_vector(bridge.hardware.imu, rotation) == ESP_OK;
    if (bridge.hardware.hardware_mutex != NULL) {
        xSemaphoreGive(bridge.hardware.hardware_mutex);
    }
}

static void fill_vesc_state(laksa_interfaces__msg__VescState *message,
                            const vesc_uart_snapshot_t *snapshot,
                            builtin_interfaces__msg__Time stamp)
{
    float speed_mps = erpm_to_speed(snapshot->measured_rpm);
    message->stamp = stamp;
    message->command_fresh = snapshot->command_fresh;
    message->direction_change_pending = snapshot->direction_change_pending;
    message->brake_active = snapshot->brake_active;
    message->telemetry_fresh = snapshot->telemetry_fresh;
    message->telemetry_sequence = snapshot->telemetry_sequence;
    message->telemetry_age_ms = snapshot->telemetry_age_ms;
    message->requested_erpm = snapshot->requested_rpm;
    message->active_erpm = snapshot->active_rpm;
    message->measured_erpm = snapshot->measured_rpm;
    message->motor_current_a = snapshot->motor_current;
    message->input_current_a = snapshot->input_current;
    message->duty_cycle = snapshot->duty_cycle;
    message->input_voltage_v = snapshot->input_voltage;
    message->amp_hours = snapshot->amp_hours;
    message->amp_hours_charged = snapshot->amp_hours_charged;
    message->watt_hours = snapshot->watt_hours;
    message->watt_hours_charged = snapshot->watt_hours_charged;
    message->temp_mosfet_c = snapshot->temp_mosfet;
    message->temp_motor_c = snapshot->temp_motor;
    message->pid_position = snapshot->pid_position;
    message->tachometer = snapshot->tachometer;
    message->tachometer_abs = snapshot->tachometer_abs;
    message->controller_id = snapshot->controller_id;
    message->fault_code = snapshot->fault_code;
    message->motor_angular_velocity_rad_s =
        snapshot->measured_rpm / (float)CONFIG_LAKSA_MOTOR_POLE_PAIRS *
        (2.0f * MICRO_ROS_PI / 60.0f);
    message->wheel_angular_velocity_rad_s = speed_mps / (wheel_diameter_m() * 0.5f);
    message->vehicle_linear_velocity_mps = speed_mps;
}

static void fill_vehicle_state(laksa_interfaces__msg__VehicleState *state)
{
    bno08x_adapter_vec3_t accel = {0};
    bno08x_adapter_vec3_t gyro = {0};
    bno08x_adapter_vec3_t mag_ut = {0};
    bno08x_adapter_quat_t rotation = {0};
    bool has_accel, has_gyro, has_mag, has_rotation;
    steering_snapshot_t steering = {0};
    vesc_uart_snapshot_t vesc = {0};
    builtin_interfaces__msg__Time stamp = ros_time_now();

    read_imu(&accel, &gyro, &mag_ut, &rotation,
             &has_accel, &has_gyro, &has_mag, &has_rotation);
    (void)steering_control_get_snapshot(bridge.hardware.steering, &steering);
    (void)vesc_uart_get_snapshot(bridge.hardware.vesc, &vesc);

    state->stamp = stamp;
    state->imu_available = has_accel || has_gyro || has_mag || has_rotation;
    state->orientation.x = rotation.i;
    state->orientation.y = rotation.j;
    state->orientation.z = rotation.k;
    state->orientation.w = has_rotation ? rotation.real : 1.0;
    state->orientation_accuracy_rad = rotation.accuracy;
    state->angular_velocity_rad_s.x = gyro.x;
    state->angular_velocity_rad_s.y = gyro.y;
    state->angular_velocity_rad_s.z = gyro.z;
    state->linear_acceleration_m_s2.x = accel.x;
    state->linear_acceleration_m_s2.y = accel.y;
    state->linear_acceleration_m_s2.z = accel.z;
    state->magnetic_field_t.x = mag_ut.x * MICRO_ROS_UT_TO_T;
    state->magnetic_field_t.y = mag_ut.y * MICRO_ROS_UT_TO_T;
    state->magnetic_field_t.z = mag_ut.z * MICRO_ROS_UT_TO_T;
    fill_vesc_state(&state->vesc, &vesc, stamp);
    state->steering_target_rad = servo_to_steering_rad(steering.target_angle_deg);
    state->steering_current_rad = servo_to_steering_rad(steering.current_angle_deg);
    state->steering_endpoint_relief_active = steering.endpoint_relief_active;
}

static void fill_imu_messages(void)
{
    bno08x_adapter_vec3_t accel = {0};
    bno08x_adapter_vec3_t gyro = {0};
    bno08x_adapter_vec3_t mag_ut = {0};
    bno08x_adapter_quat_t rotation = {0};
    bool has_accel, has_gyro, has_mag, has_rotation;
    builtin_interfaces__msg__Time stamp = ros_time_now();

    read_imu(&accel, &gyro, &mag_ut, &rotation,
             &has_accel, &has_gyro, &has_mag, &has_rotation);

    bridge.imu_message.header.stamp = stamp;
    bridge.imu_message.orientation.x = rotation.i;
    bridge.imu_message.orientation.y = rotation.j;
    bridge.imu_message.orientation.z = rotation.k;
    bridge.imu_message.orientation.w = has_rotation ? rotation.real : 1.0;
    bridge.imu_message.orientation_covariance[0] = has_rotation ? 0.0 : -1.0;
    bridge.imu_message.angular_velocity.x = gyro.x;
    bridge.imu_message.angular_velocity.y = gyro.y;
    bridge.imu_message.angular_velocity.z = gyro.z;
    bridge.imu_message.angular_velocity_covariance[0] = has_gyro ? 0.0 : -1.0;
    bridge.imu_message.linear_acceleration.x = accel.x;
    bridge.imu_message.linear_acceleration.y = accel.y;
    bridge.imu_message.linear_acceleration.z = accel.z;
    bridge.imu_message.linear_acceleration_covariance[0] = has_accel ? 0.0 : -1.0;

    bridge.mag_message.header.stamp = stamp;
    bridge.mag_message.magnetic_field.x = mag_ut.x * MICRO_ROS_UT_TO_T;
    bridge.mag_message.magnetic_field.y = mag_ut.y * MICRO_ROS_UT_TO_T;
    bridge.mag_message.magnetic_field.z = mag_ut.z * MICRO_ROS_UT_TO_T;
    bridge.mag_message.magnetic_field_covariance[0] = has_mag ? 0.0 : -1.0;
}

static void command_callback(const void *message)
{
    const laksa_interfaces__msg__DriveCommand *command =
        (const laksa_interfaces__msg__DriveCommand *)message;
    uint8_t error = apply_drive_command(command->speed_mps,
                                        command->steering_angle_rad,
                                        bridge.brake_requested || command->brake);
    if (error != 0) {
        ESP_LOGW(TAG, "Rejected drive command (error %u)", (unsigned)error);
    }
}

static void brake_callback(const void *message)
{
    const std_msgs__msg__Bool *brake = (const std_msgs__msg__Bool *)message;
    bridge.brake_requested = brake->data;
    if (brake->data) {
        apply_actuator_failsafe();
    }
}

static void set_command_callback(const void *request, void *response)
{
    const laksa_interfaces__srv__SetDriveCommand_Request *set_request =
        (const laksa_interfaces__srv__SetDriveCommand_Request *)request;
    laksa_interfaces__srv__SetDriveCommand_Response *set_response =
        (laksa_interfaces__srv__SetDriveCommand_Response *)response;
    set_response->error_code = apply_drive_command(set_request->command.speed_mps,
                                                   set_request->command.steering_angle_rad,
                                                   bridge.brake_requested ||
                                                       set_request->command.brake);
    set_response->accepted = set_response->error_code == 0;
}

static void get_state_callback(const void *request, void *response)
{
    (void)request;
    laksa_interfaces__srv__GetVehicleState_Response *get_response =
        (laksa_interfaces__srv__GetVehicleState_Response *)response;
    fill_vehicle_state(&get_response->state);
}

static bool create_entities(void)
{
    bridge.allocator = rcl_get_default_allocator();
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    if (rcl_init_options_init(&init_options, bridge.allocator) != RCL_RET_OK ||
        rcl_init_options_set_domain_id(&init_options, CONFIG_LAKSA_ROS_DOMAIN_ID) != RCL_RET_OK ||
        rclc_support_init_with_options(&bridge.support, 0, NULL, &init_options,
                                       &bridge.allocator) != RCL_RET_OK) {
        consume_rcl_result(rcl_init_options_fini(&init_options));
        return false;
    }
    consume_rcl_result(rcl_init_options_fini(&init_options));
    /* Zero-initialized handles can now be finalized if the Agent disappears
       while the remaining XRCE entities are only partially created. */
    bridge.entities_created = true;

    bridge.node = rcl_get_zero_initialized_node();
    if (rclc_node_init_default(&bridge.node, "laksa_esp32", "", &bridge.support) != RCL_RET_OK) {
        return false;
    }

    bridge.imu_publisher = rcl_get_zero_initialized_publisher();
    bridge.mag_publisher = rcl_get_zero_initialized_publisher();
    bridge.vesc_publisher = rcl_get_zero_initialized_publisher();
    bridge.state_publisher = rcl_get_zero_initialized_publisher();
    if (rclc_publisher_init_best_effort(&bridge.imu_publisher, &bridge.node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/laksa/imu/data") != RCL_RET_OK ||
        rclc_publisher_init_best_effort(&bridge.mag_publisher, &bridge.node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField), "/laksa/imu/mag") != RCL_RET_OK ||
        rclc_publisher_init_best_effort(&bridge.vesc_publisher, &bridge.node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(laksa_interfaces, msg, VescState), "/laksa/vesc/state") != RCL_RET_OK ||
        rclc_publisher_init_best_effort(&bridge.state_publisher, &bridge.node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(laksa_interfaces, msg, VehicleState), "/laksa/state") != RCL_RET_OK) {
        return false;
    }

    bridge.command_subscription = rcl_get_zero_initialized_subscription();
    bridge.brake_subscription = rcl_get_zero_initialized_subscription();
    if (rclc_subscription_init_default(&bridge.command_subscription, &bridge.node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(laksa_interfaces, msg, DriveCommand), "/laksa/command") != RCL_RET_OK ||
        rclc_subscription_init_default(&bridge.brake_subscription, &bridge.node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/laksa/brake") != RCL_RET_OK) {
        return false;
    }

    bridge.set_command_service = rcl_get_zero_initialized_service();
    bridge.get_state_service = rcl_get_zero_initialized_service();
    if (rclc_service_init_default(&bridge.set_command_service, &bridge.node,
            ROSIDL_GET_SRV_TYPE_SUPPORT(laksa_interfaces, srv, SetDriveCommand),
            "/laksa/set_drive_command") != RCL_RET_OK ||
        rclc_service_init_default(&bridge.get_state_service, &bridge.node,
            ROSIDL_GET_SRV_TYPE_SUPPORT(laksa_interfaces, srv, GetVehicleState),
            "/laksa/get_state") != RCL_RET_OK) {
        return false;
    }

    if (!sensor_msgs__msg__Imu__init(&bridge.imu_message) ||
        !sensor_msgs__msg__MagneticField__init(&bridge.mag_message) ||
        !laksa_interfaces__msg__VescState__init(&bridge.vesc_message) ||
        !laksa_interfaces__msg__VehicleState__init(&bridge.state_message) ||
        !laksa_interfaces__msg__DriveCommand__init(&bridge.command_message) ||
        !std_msgs__msg__Bool__init(&bridge.brake_message) ||
        !laksa_interfaces__srv__SetDriveCommand_Request__init(&bridge.set_request) ||
        !laksa_interfaces__srv__SetDriveCommand_Response__init(&bridge.set_response) ||
        !laksa_interfaces__srv__GetVehicleState_Request__init(&bridge.get_request) ||
        !laksa_interfaces__srv__GetVehicleState_Response__init(&bridge.get_response)) {
        return false;
    }
    if (!rosidl_runtime_c__String__assign(&bridge.imu_message.header.frame_id, "imu_link") ||
        !rosidl_runtime_c__String__assign(&bridge.mag_message.header.frame_id, "imu_link")) {
        return false;
    }

    bridge.executor = rclc_executor_get_zero_initialized_executor();
    if (rclc_executor_init(&bridge.executor, &bridge.support.context,
                           MICRO_ROS_EXECUTOR_HANDLES, &bridge.allocator) != RCL_RET_OK ||
        rclc_executor_add_subscription(&bridge.executor, &bridge.command_subscription,
                                       &bridge.command_message, command_callback, ON_NEW_DATA) != RCL_RET_OK ||
        rclc_executor_add_subscription(&bridge.executor, &bridge.brake_subscription,
                                       &bridge.brake_message, brake_callback, ON_NEW_DATA) != RCL_RET_OK ||
        rclc_executor_add_service(&bridge.executor, &bridge.set_command_service,
                                  &bridge.set_request, &bridge.set_response,
                                  set_command_callback) != RCL_RET_OK ||
        rclc_executor_add_service(&bridge.executor, &bridge.get_state_service,
                                  &bridge.get_request, &bridge.get_response,
                                  get_state_callback) != RCL_RET_OK) {
        return false;
    }

    (void)rmw_uros_sync_session(1000);
    ESP_LOGI(TAG, "ROS 2 entities ready in domain %d", CONFIG_LAKSA_ROS_DOMAIN_ID);
    return true;
}

static void destroy_entities(void)
{
    bridge.brake_requested = true;
    apply_actuator_failsafe();
    if (!bridge.entities_created) {
        memset(&bridge.support, 0, sizeof(bridge.support));
        return;
    }

    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&bridge.support.context);
    if (rmw_context != NULL) {
        (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    }
    (void)rclc_executor_fini(&bridge.executor);
    consume_rcl_result(rcl_service_fini(&bridge.get_state_service, &bridge.node));
    consume_rcl_result(rcl_service_fini(&bridge.set_command_service, &bridge.node));
    consume_rcl_result(rcl_subscription_fini(&bridge.brake_subscription, &bridge.node));
    consume_rcl_result(rcl_subscription_fini(&bridge.command_subscription, &bridge.node));
    consume_rcl_result(rcl_publisher_fini(&bridge.state_publisher, &bridge.node));
    consume_rcl_result(rcl_publisher_fini(&bridge.vesc_publisher, &bridge.node));
    consume_rcl_result(rcl_publisher_fini(&bridge.mag_publisher, &bridge.node));
    consume_rcl_result(rcl_publisher_fini(&bridge.imu_publisher, &bridge.node));
    consume_rcl_result(rcl_node_fini(&bridge.node));
    (void)rclc_support_fini(&bridge.support);

    sensor_msgs__msg__Imu__fini(&bridge.imu_message);
    sensor_msgs__msg__MagneticField__fini(&bridge.mag_message);
    laksa_interfaces__msg__VescState__fini(&bridge.vesc_message);
    laksa_interfaces__msg__VehicleState__fini(&bridge.state_message);
    laksa_interfaces__msg__DriveCommand__fini(&bridge.command_message);
    std_msgs__msg__Bool__fini(&bridge.brake_message);
    laksa_interfaces__srv__SetDriveCommand_Request__fini(&bridge.set_request);
    laksa_interfaces__srv__SetDriveCommand_Response__fini(&bridge.set_response);
    laksa_interfaces__srv__GetVehicleState_Request__fini(&bridge.get_request);
    laksa_interfaces__srv__GetVehicleState_Response__fini(&bridge.get_response);
    bridge.entities_created = false;
}

static void publish_periodic(TickType_t now,
                             TickType_t *last_imu,
                             TickType_t *last_mag,
                             TickType_t *last_state)
{
    if (now - *last_imu >= pdMS_TO_TICKS(MICRO_ROS_IMU_PERIOD_MS)) {
        *last_imu = now;
        fill_imu_messages();
        consume_rcl_result(rcl_publish(&bridge.imu_publisher, &bridge.imu_message, NULL));
    }
    if (now - *last_mag >= pdMS_TO_TICKS(MICRO_ROS_MAG_PERIOD_MS)) {
        *last_mag = now;
        consume_rcl_result(rcl_publish(&bridge.mag_publisher, &bridge.mag_message, NULL));
    }
    if (now - *last_state >= pdMS_TO_TICKS(MICRO_ROS_STATE_PERIOD_MS)) {
        *last_state = now;
        fill_vehicle_state(&bridge.state_message);
        bridge.vesc_message = bridge.state_message.vesc;
        consume_rcl_result(rcl_publish(&bridge.vesc_publisher, &bridge.vesc_message, NULL));
        consume_rcl_result(rcl_publish(&bridge.state_publisher, &bridge.state_message, NULL));
    }
}

static void micro_ros_task(void *argument)
{
    (void)argument;
    bridge_connection_state_t state = BRIDGE_WAITING_FOR_AGENT;
    TickType_t last_ping = 0;
    TickType_t last_imu = 0;
    TickType_t last_mag = 0;
    TickType_t last_state = 0;

    status_led_set_mode(STATUS_LED_WAITING_FOR_MICRO_ROS);

    while (true) {
        TickType_t now = xTaskGetTickCount();
        switch (state) {
        case BRIDGE_WAITING_FOR_AGENT:
            if (rmw_uros_ping_agent(MICRO_ROS_AGENT_PING_TIMEOUT_MS,
                                    MICRO_ROS_AGENT_PING_ATTEMPTS) == RMW_RET_OK) {
                state = BRIDGE_AGENT_AVAILABLE;
            } else {
                vTaskDelay(pdMS_TO_TICKS(MICRO_ROS_RETRY_PERIOD_MS));
            }
            break;
        case BRIDGE_AGENT_AVAILABLE:
            if (create_entities()) {
                last_ping = now;
                last_imu = now;
                last_mag = now;
                last_state = now;
                status_led_set_mode(STATUS_LED_MICRO_ROS_CONNECTED);
                state = BRIDGE_AGENT_CONNECTED;
            } else {
                ESP_LOGE(TAG, "Failed to create ROS 2 entities; retrying");
                destroy_entities();
                state = BRIDGE_WAITING_FOR_AGENT;
            }
            break;
        case BRIDGE_AGENT_CONNECTED:
            if (now - last_ping >= pdMS_TO_TICKS(MICRO_ROS_AGENT_PING_PERIOD_MS)) {
                last_ping = now;
                if (rmw_uros_ping_agent(MICRO_ROS_AGENT_PING_TIMEOUT_MS,
                                        MICRO_ROS_AGENT_PING_ATTEMPTS) != RMW_RET_OK) {
                    state = BRIDGE_AGENT_DISCONNECTED;
                    break;
                }
            }
            (void)rclc_executor_spin_some(&bridge.executor, RCL_MS_TO_NS(5));
            /* A subscription callback can update last_drive_command_tick while
             * spin_some() runs. Refresh now afterwards; comparing the stale
             * pre-spin tick against a newer callback tick would underflow the
             * unsigned FreeRTOS tick counter and cause a false timeout. */
            now = xTaskGetTickCount();
            if (!bridge.steering_failsafe_active &&
                now - bridge.last_drive_command_tick >
                    pdMS_TO_TICKS(VESC_COMMAND_TIMEOUT_MS)) {
                ESP_LOGW(TAG, "Drive command timeout; braking and centering steering");
                apply_actuator_failsafe();
            }
            publish_periodic(now, &last_imu, &last_mag, &last_state);
            vTaskDelay(pdMS_TO_TICKS(2));
            break;
        case BRIDGE_AGENT_DISCONNECTED:
            ESP_LOGW(TAG, "Agent disconnected; stopping actuators");
            status_led_set_mode(STATUS_LED_WAITING_FOR_MICRO_ROS);
            destroy_entities();
            state = BRIDGE_WAITING_FOR_AGENT;
            break;
        }
    }
}

esp_err_t micro_ros_bridge_start(const micro_ros_bridge_config_t *config)
{
    if (config == NULL || config->steering == NULL || config->vesc == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(&bridge, 0, sizeof(bridge));
    bridge.hardware = *config;
    bridge.brake_requested = true;
    bridge.steering_failsafe_active = true;

#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
    if (rmw_uros_set_custom_transport(true, &cdc_port,
                                      esp_usbcdc_open, esp_usbcdc_close,
                                      esp_usbcdc_write, esp_usbcdc_read) != RMW_RET_OK) {
        return ESP_FAIL;
    }
#else
#error "micro-ROS must be built with RMW_UXRCE_TRANSPORT=custom"
#endif

    if (xTaskCreate(micro_ros_task, "micro_ros", MICRO_ROS_TASK_STACK_SIZE,
                    NULL, MICRO_ROS_TASK_PRIORITY, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "USB-CDC bridge started; waiting for micro-ROS Agent");
    return ESP_OK;
}
