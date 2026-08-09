/* Embedded WiFi dashboard implementation.
 *
 * The ESP32 joins the configured WiFi network as a station and starts a small
 * HTTP server. The page polls JSON state and sends steering/VESC commands.
 */

#include "web_dashboard.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp32_config.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"

#define BNO08X_TRANSPORT_NAME "SPI"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
#define HTTP_QUERY_BUF_LEN 96
#define HTTP_RESPONSE_BUF_LEN 2048

static const char *TAG = "web_dashboard";
static EventGroupHandle_t wifi_event_group;
static web_dashboard_t *active_dashboard;

static const char dashboard_html[] =
    "<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>LAKSA Dashboard</title><style>"
    "body{margin:0;font-family:system-ui,-apple-system,Segoe UI,sans-serif;background:#10151f;color:#edf2f7}"
    "main{max-width:980px;margin:0 auto;padding:18px}h1{font-size:24px;margin:0 0 14px}"
    ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:12px}"
    "section{background:#17202d;border:1px solid #283548;border-radius:8px;padding:14px}"
    "h2{font-size:16px;margin:0 0 12px;color:#9cc5ff}.row{display:flex;justify-content:space-between;gap:10px;margin:8px 0}"
    ".value{font-variant-numeric:tabular-nums;color:#d6e4ff}.controls{display:grid;grid-template-columns:48px 1fr 48px;gap:8px;align-items:center}"
    ".drive{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px;margin-top:12px}"
    "button{height:44px;border:0;border-radius:6px;background:#2f80ed;color:white;font-size:16px;touch-action:none;user-select:none}"
    "button.stop{background:#c0392b}button:active{filter:brightness(.75)}input[type=range]{width:100%}"
    ".muted{color:#91a0b5;font-size:13px}.bad{color:#ff8a8a}.ok{color:#86efac}"
    "</style></head><body><main><h1>LAKSA ESP32 Dashboard</h1><div class='grid'>"
    "<section><h2>Steering</h2><div class='row'><span>PCA9685 CH 7</span><span id='steerNow' class='value'>100&deg;</span></div>"
    "<div class='controls'><button onclick='nudge(-2)'>&larr;</button><input id='steer' type='range' min='65' max='137' value='100' onchange='setSteering(this.value)'><button onclick='nudge(2)'>&rarr;</button></div>"
    "<button style='width:100%;margin-top:10px' onclick='setSteering(100)'>Center: 100&deg;</button><div id='steerLimits' class='muted'></div></section>"
    "<section><h2>VESC Motor</h2><div class='row'><span>Commanded ERPM</span><span id='motorRpm' class='value'>0</span></div>"
    "<div class='row'><span>Command limit</span><span id='speedValue' class='value'>900 ERPM</span></div><input id='speed' type='range' min='100' max='900' step='50' value='900' oninput='speedValue.textContent=this.value+\" ERPM\"'>"
    "<div class='drive'><button id='reverse'>Reverse</button><button class='stop' onclick='stopMotor()'>STOP</button><button id='forward'>Forward</button></div>"
    "<div id='motorStatus' class='muted'>Hold a direction button to move</div>"
    "<hr style='border:0;border-top:1px solid #283548;margin:14px 0'><div class='row'><span>UART Telemetry</span><span id='vescLink' class='bad'>no data</span></div>"
    "<div class='row'><span>Measured ERPM</span><span id='vescRpm' class='value'>--</span></div>"
    "<div class='row'><span>Input voltage</span><span id='vescVoltage' class='value'>--</span></div>"
    "<div class='row'><span>Motor / input current</span><span id='vescCurrent' class='value'>--</span></div>"
    "<div class='row'><span>Duty</span><span id='vescDuty' class='value'>--</span></div>"
    "<div class='row'><span>MOSFET / motor temperature</span><span id='vescTemp' class='value'>--</span></div>"
    "<div class='row'><span>Energy used / regenerated</span><span id='vescEnergy' class='value'>--</span></div>"
    "<div class='row'><span>Tachometer / fault</span><span id='vescTacho' class='value'>--</span></div></section>"
    "<section><h2>IMU</h2><div class='row'><span>Transport</span><span id='transport' class='value'></span></div>"
    "<div id='imu'></div></section></div></main><script>"
    "let driveTimer=null,driveSign=0;"
    "function fmt(v,n=3){return Number(v||0).toFixed(n)}"
    "async function setSteering(angle){angle=Math.max(65,Math.min(137,Math.round(angle)));steer.value=angle;await fetch(`/api/steering?angle=${angle}`)}"
    "function nudge(delta){setSteering(Number(steer.value)+delta)}"
    "function sendMotor(){let rpm=driveSign*Number(speed.value);fetch(`/api/motor?rpm=${rpm}`).catch(()=>stopMotor())}"
    "function startMotor(sign){stopMotor(false);driveSign=sign;sendMotor();driveTimer=setInterval(sendMotor,100)}"
    "function stopMotor(send=true){if(driveTimer)clearInterval(driveTimer);driveTimer=null;driveSign=0;if(send)fetch('/api/motor?rpm=0').catch(()=>{})}"
    "function bindHold(el,sign){el.addEventListener('pointerdown',e=>{e.preventDefault();el.setPointerCapture(e.pointerId);startMotor(sign)});el.addEventListener('pointerup',()=>stopMotor());el.addEventListener('pointercancel',()=>stopMotor());el.addEventListener('lostpointercapture',()=>stopMotor())}"
    "function quat(name,q,showAcc=true){let acc=showAcc?` acc ${fmt(q.accuracy,4)}`:'';return `<div class='row'><span>${name}</span><span class='${q.has?'ok':'bad'}'>${q.has?'Y':'N'}</span></div><div class='value'>i ${fmt(q.i,4)} j ${fmt(q.j,4)} k ${fmt(q.k,4)} r ${fmt(q.real,4)}${acc}</div>`}"
    "async function refresh(){try{let r=await fetch('/api/state');let d=await r.json();steerNow.textContent=d.steering.current+'\u00b0 (target '+d.steering.target+'\u00b0)';if(document.activeElement!==steer)steer.value=d.steering.target;steerLimits.textContent=`Safe range ${d.steering.left}\u00b0 - ${d.steering.right}\u00b0${d.steering.endpoint_relief?' - endpoint relieved':''}`;motorRpm.textContent=d.motor.active_rpm;motorStatus.textContent=d.motor.direction_change_pending?'Neutral pause before direction change':(!d.motor.command_fresh&&d.motor.requested_rpm!==0?'Stopped by watchdog':(d.motor.active_rpm===0?'Stopped':'Web command active'));motorStatus.className=d.motor.active_rpm!==0?'muted ok':'muted';let t=d.motor.telemetry;vescLink.textContent=t.fresh?'connected':'no recent data';vescLink.className=t.fresh?'ok':'bad';vescRpm.textContent=t.fresh?Math.round(t.rpm):'--';vescVoltage.textContent=t.fresh?fmt(t.input_voltage,1)+' V':'--';vescCurrent.textContent=t.fresh?fmt(t.motor_current,1)+' / '+fmt(t.input_current,1)+' A':'--';vescDuty.textContent=t.fresh?fmt(t.duty*100,1)+' %':'--';vescTemp.textContent=t.fresh?fmt(t.temp_mosfet,1)+' / '+fmt(t.temp_motor,1)+' \u00b0C':'--';vescEnergy.textContent=t.fresh?fmt(t.watt_hours,2)+' / '+fmt(t.watt_hours_charged,2)+' Wh':'--';vescTacho.textContent=t.fresh?t.tachometer+' / '+t.fault_code:'--';transport.textContent=d.imu.transport;imu.innerHTML=`<div class='row'><span>Acceleration</span><span class='${d.imu.accel.has?'ok':'bad'}'>${d.imu.accel.has?'Y':'N'}</span></div><div class='value'>x ${fmt(d.imu.accel.x)} y ${fmt(d.imu.accel.y)} z ${fmt(d.imu.accel.z)}</div>${quat('Rotation',d.imu.rotation)}${quat('Game rotation',d.imu.game_rotation,false)}`}"
    "catch(e){imu.innerHTML='<span class=bad>offline</span>'}}"
    "bindHold(reverse,-1);bindHold(forward,1);document.addEventListener('visibilitychange',()=>{if(document.hidden)stopMotor()});window.addEventListener('pagehide',()=>stopMotor());setInterval(refresh,250);refresh();</script></body></html>";

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Dashboard URL: http://" IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static esp_err_t wifi_start(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(err, TAG, "NVS init failed");

    wifi_event_group = xEventGroupCreate();
    if (wifi_event_group == NULL) {
        return ESP_ERR_NO_MEM;
    }

    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "netif init failed");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "event loop init failed");
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init_config), TAG, "WiFi init failed");
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(WIFI_EVENT,
                                                            ESP_EVENT_ANY_ID,
                                                            wifi_event_handler,
                                                            NULL,
                                                            NULL),
                        TAG,
                        "WiFi event handler failed");
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(IP_EVENT,
                                                            IP_EVENT_STA_GOT_IP,
                                                            wifi_event_handler,
                                                            NULL,
                                                            NULL),
                        TAG,
                        "IP event handler failed");

    wifi_config_t wifi_config = {0};
    strlcpy((char *)wifi_config.sta.ssid, WIFI_STA_SSID, sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password, WIFI_STA_PASSWORD, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "WiFi mode failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &wifi_config), TAG, "WiFi config failed");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "WiFi start failed");

    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));

    if ((bits & WIFI_CONNECTED_BIT) == 0) {
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, dashboard_html, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t state_handler(httpd_req_t *req)
{
    web_dashboard_t *dashboard = (web_dashboard_t *)req->user_ctx;
    bno08x_adapter_vec3_t accel = {0};
    bno08x_adapter_quat_t rotation = {0};
    bno08x_adapter_quat_t game_rotation = {0};

    bool has_accel = false;
    bool has_rotation = false;
    bool has_game_rotation = false;
    steering_snapshot_t steering = {0};
    vesc_uart_snapshot_t motor = {0};

    if (dashboard->imu != NULL && dashboard->hardware_mutex != NULL) {
        xSemaphoreTake(dashboard->hardware_mutex, portMAX_DELAY);
    }

    if (dashboard->imu != NULL) {
        has_accel = bno08x_adapter_get_acceleration(dashboard->imu, &accel) == ESP_OK;
        has_rotation = bno08x_adapter_get_rotation_vector(dashboard->imu, &rotation) == ESP_OK;
        has_game_rotation = bno08x_adapter_get_game_rotation_vector(dashboard->imu, &game_rotation) == ESP_OK;
    }

    if (dashboard->imu != NULL && dashboard->hardware_mutex != NULL) {
        xSemaphoreGive(dashboard->hardware_mutex);
    }

    ESP_RETURN_ON_ERROR(steering_control_get_snapshot(dashboard->steering, &steering),
                        TAG,
                        "steering snapshot failed");
    ESP_RETURN_ON_ERROR(vesc_uart_get_snapshot(dashboard->vesc, &motor),
                        TAG,
                        "VESC snapshot failed");

    char *response = malloc(HTTP_RESPONSE_BUF_LEN);
    if (response == NULL) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "out of memory");
    }

    int len = snprintf(response,
                       HTTP_RESPONSE_BUF_LEN,
                       "{\"steering\":{\"channel\":%d,\"left\":%d,\"center\":%d,\"right\":%d,"
                       "\"target\":%u,\"current\":%u,\"endpoint_relief\":%s},"
                       "\"motor\":{\"requested_rpm\":%ld,\"active_rpm\":%ld,\"max_abs_rpm\":%d,"
                       "\"command_fresh\":%s,\"direction_change_pending\":%s,"
                       "\"telemetry\":{\"fresh\":%s,\"rpm\":%.1f,\"motor_current\":%.2f,"
                       "\"input_current\":%.2f,\"duty\":%.4f,\"input_voltage\":%.1f,"
                       "\"amp_hours\":%.4f,\"amp_hours_charged\":%.4f,"
                       "\"watt_hours\":%.4f,\"watt_hours_charged\":%.4f,"
                       "\"temp_mosfet\":%.1f,\"temp_motor\":%.1f,\"pid_position\":%.4f,"
                       "\"tachometer\":%ld,\"tachometer_abs\":%ld,\"controller_id\":%u,\"fault_code\":%u}},"
                       "\"imu\":{\"transport\":\"%s\","
                       "\"accel\":{\"has\":%s,\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
                       "\"rotation\":{\"has\":%s,\"i\":%.4f,\"j\":%.4f,\"k\":%.4f,\"real\":%.4f,\"accuracy\":%.4f},"
                       "\"game_rotation\":{\"has\":%s,\"i\":%.4f,\"j\":%.4f,\"k\":%.4f,\"real\":%.4f,\"accuracy\":%.4f}}}",
                       STEERING_PCA_CHANNEL,
                       STEERING_LEFT_SAFE_DEG,
                       STEERING_CENTER_DEG,
                       STEERING_RIGHT_SAFE_DEG,
                       steering.target_angle_deg,
                       steering.current_angle_deg,
                       steering.endpoint_relief_active ? "true" : "false",
                       (long)motor.requested_rpm,
                       (long)motor.active_rpm,
                       VESC_MAX_ABS_RPM,
                       motor.command_fresh ? "true" : "false",
                       motor.direction_change_pending ? "true" : "false",
                       motor.telemetry_fresh ? "true" : "false",
                       motor.measured_rpm,
                       motor.motor_current,
                       motor.input_current,
                       motor.duty_cycle,
                       motor.input_voltage,
                       motor.amp_hours,
                       motor.amp_hours_charged,
                       motor.watt_hours,
                       motor.watt_hours_charged,
                       motor.temp_mosfet,
                       motor.temp_motor,
                       motor.pid_position,
                       (long)motor.tachometer,
                       (long)motor.tachometer_abs,
                       (unsigned)motor.controller_id,
                       (unsigned)motor.fault_code,
                       BNO08X_TRANSPORT_NAME,
                       has_accel ? "true" : "false",
                       accel.x,
                       accel.y,
                       accel.z,
                       has_rotation ? "true" : "false",
                       rotation.i,
                       rotation.j,
                       rotation.k,
                       rotation.real,
                       rotation.accuracy,
                       has_game_rotation ? "true" : "false",
                       game_rotation.i,
                       game_rotation.j,
                       game_rotation.k,
                       game_rotation.real,
                       game_rotation.accuracy);

    if (len < 0 || len >= HTTP_RESPONSE_BUF_LEN) {
        free(response);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "state too large");
    }

    httpd_resp_set_type(req, "application/json; charset=utf-8");
    esp_err_t err = httpd_resp_send(req, response, len);
    free(response);
    return err;
}

static esp_err_t steering_handler(httpd_req_t *req)
{
    web_dashboard_t *dashboard = (web_dashboard_t *)req->user_ctx;
    char query[HTTP_QUERY_BUF_LEN] = {0};
    char angle_text[8] = {0};

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "angle", angle_text, sizeof(angle_text)) != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "expected angle");
    }

    char *end = NULL;
    long angle = strtol(angle_text, &end, 10);
    if (end == angle_text || *end != '\0' ||
        angle < STEERING_LEFT_SAFE_DEG || angle > STEERING_RIGHT_SAFE_DEG) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "angle outside safe steering range");
    }

    esp_err_t err = steering_control_set_target(dashboard->steering, (uint8_t)angle);
    if (err != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, esp_err_to_name(err));
    }

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

static esp_err_t motor_handler(httpd_req_t *req)
{
    web_dashboard_t *dashboard = (web_dashboard_t *)req->user_ctx;
    char query[HTTP_QUERY_BUF_LEN] = {0};
    char rpm_text[16] = {0};

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "rpm", rpm_text, sizeof(rpm_text)) != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "expected rpm");
    }

    char *end = NULL;
    long rpm = strtol(rpm_text, &end, 10);
    if (end == rpm_text || *end != '\0' || rpm < -VESC_MAX_ABS_RPM || rpm > VESC_MAX_ABS_RPM) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "rpm outside safe range");
    }

    esp_err_t err = vesc_uart_set_target_rpm(dashboard->vesc, (int32_t)rpm);
    if (err != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, esp_err_to_name(err));
    }

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

esp_err_t web_dashboard_start(web_dashboard_t *dashboard)
{
    if (dashboard == NULL || dashboard->steering == NULL || dashboard->vesc == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    active_dashboard = dashboard;
    ESP_RETURN_ON_ERROR(wifi_start(), TAG, "WiFi failed");

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    config.uri_match_fn = httpd_uri_match_wildcard;

    httpd_handle_t server = NULL;
    ESP_RETURN_ON_ERROR(httpd_start(&server, &config), TAG, "HTTP server failed");

    httpd_uri_t index_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = index_handler,
        .user_ctx = active_dashboard,
    };
    httpd_uri_t state_uri = {
        .uri = "/api/state",
        .method = HTTP_GET,
        .handler = state_handler,
        .user_ctx = active_dashboard,
    };
    httpd_uri_t steering_uri = {
        .uri = "/api/steering",
        .method = HTTP_GET,
        .handler = steering_handler,
        .user_ctx = active_dashboard,
    };
    httpd_uri_t motor_uri = {
        .uri = "/api/motor",
        .method = HTTP_GET,
        .handler = motor_handler,
        .user_ctx = active_dashboard,
    };

    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &index_uri), TAG, "index route failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &state_uri), TAG, "state route failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &steering_uri), TAG, "steering route failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &motor_uri), TAG, "motor route failed");

    return ESP_OK;
}
