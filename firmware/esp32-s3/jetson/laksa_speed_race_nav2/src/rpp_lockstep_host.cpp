// Copyright 2026 Leobardo Gomez
// Licensed under the Apache License, Version 2.0

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_map_server/map_io.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"

namespace laksa_speed_race_nav2
{

class FixedGoalChecker final : public nav2_core::GoalChecker
{
public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    const std::string &,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}

  void reset() override {}

  bool isGoalReached(
    const geometry_msgs::msg::Pose &,
    const geometry_msgs::msg::Pose &,
    const geometry_msgs::msg::Twist &) override
  {
    return false;
  }

  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance) override
  {
    const double invalid = std::numeric_limits<double>::lowest();
    pose_tolerance.position.x = 0.25;
    pose_tolerance.position.y = 0.25;
    pose_tolerance.position.z = invalid;
    vel_tolerance.linear.x = invalid;
    vel_tolerance.linear.y = invalid;
    vel_tolerance.linear.z = invalid;
    vel_tolerance.angular.x = invalid;
    vel_tolerance.angular.y = invalid;
    vel_tolerance.angular.z = invalid;
    return true;
  }
};

class RppLockstepHost final : public rclcpp_lifecycle::LifecycleNode
{
public:
  RppLockstepHost()
  : rclcpp_lifecycle::LifecycleNode("rpp_lockstep_host"),
    controller_loader_("nav2_core", "nav2_core::Controller")
  {
    declare_parameter("controller_frequency", 100.0);
    declare_parameter("map_yaml_path", std::string{});
    declare_parameter(
      "controller_plugin",
      std::string{"nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"});
  }

  void initialize()
  {
    const auto map_yaml = get_parameter("map_yaml_path").as_string();
    if (map_yaml.empty()) {
      throw std::runtime_error("map_yaml_path is required");
    }

    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      "c1_rpp_costmap", get_namespace(), "c1_rpp_costmap");
    costmap_ros_->set_parameters({
      rclcpp::Parameter("global_frame", "map"),
      rclcpp::Parameter("robot_base_frame", "c1/base_link"),
      rclcpp::Parameter("plugins", std::vector<std::string>{}),
      rclcpp::Parameter("filters", std::vector<std::string>{}),
      rclcpp::Parameter("rolling_window", false),
      rclcpp::Parameter("track_unknown_space", false),
      rclcpp::Parameter("resolution", 0.05),
      rclcpp::Parameter("width", 50),
      rclcpp::Parameter("height", 20),
      rclcpp::Parameter("origin_x", -4.0),
      rclcpp::Parameter("origin_y", -4.0),
      rclcpp::Parameter("footprint_padding", 0.0),
      rclcpp::Parameter(
        "footprint", "[[-0.149,-0.148],[0.419,-0.148],[0.419,0.148],[-0.149,0.148]]")
    });
    costmap_ros_->on_configure(rclcpp_lifecycle::State());
    load_costmap(map_yaml);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    const auto plugin_type = get_parameter("controller_plugin").as_string();
    controller_ = controller_loader_.createSharedInstance(plugin_type);
    controller_->configure(shared_from_this(), "RPP", tf_buffer_, costmap_ros_);
    controller_->activate();

    command_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/c1/nav2_cmd_vel", 10);
    fault_pub_ = create_publisher<std_msgs::msg::String>("/c1/controller_fault", 10);
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/c1/nav2_path", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&RppLockstepHost::on_path, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/c1/odom", 10, std::bind(&RppLockstepHost::on_odom, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(), "C1.2 lockstep host loaded %s; no control timer exists", plugin_type.c_str());
  }

  ~RppLockstepHost() override
  {
    if (controller_) {
      controller_->deactivate();
      controller_->cleanup();
    }
  }

private:
  static int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
  }

  void load_costmap(const std::string & yaml_path)
  {
    nav_msgs::msg::OccupancyGrid map;
    if (nav2_map_server::loadMapFromYaml(yaml_path, map) != nav2_map_server::LOAD_MAP_SUCCESS) {
      throw std::runtime_error("failed to load canonical C1 map: " + yaml_path);
    }
    auto * costmap = costmap_ros_->getCostmap();
    costmap->resizeMap(
      map.info.width, map.info.height, map.info.resolution,
      map.info.origin.position.x, map.info.origin.position.y);
    for (unsigned int y = 0; y < map.info.height; ++y) {
      for (unsigned int x = 0; x < map.info.width; ++x) {
        const int8_t occupancy = map.data[y * map.info.width + x];
        unsigned char cost = nav2_costmap_2d::NO_INFORMATION;
        if (occupancy == 0) {
          cost = nav2_costmap_2d::FREE_SPACE;
        } else if (occupancy > 0) {
          cost = nav2_costmap_2d::LETHAL_OBSTACLE;
        }
        costmap->setCost(x, y, cost);
      }
    }
  }

  void on_path(const nav_msgs::msg::Path::SharedPtr message)
  {
    if (message->header.frame_id != "map" || message->poses.size() != 2185U) {
      publish_fault("invalid_unrolled_path");
      return;
    }
    controller_->setPlan(*message);
    path_ready_ = true;
    if (pending_odom_) {
      auto pending = std::move(*pending_odom_);
      pending_odom_.reset();
      on_odom(std::make_shared<nav_msgs::msg::Odometry>(pending));
    }
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    if (!path_ready_) {
      pending_odom_ = *message;
      return;
    }
    const int64_t stamp = stamp_ns(message->header.stamp);
    if (stamp <= 0) {
      publish_fault("invalid_zero_state_stamp");
      return;
    }
    if (last_state_stamp_ && stamp == *last_state_stamp_) {
      ++duplicate_stamp_rejections_;
      RCLCPP_WARN(get_logger(), "Rejected duplicate odometry stamp: %ld", stamp);
      return;
    }
    if (last_state_stamp_ && stamp < *last_state_stamp_) {
      publish_fault("non_monotonic_state_stamp");
      return;
    }
    last_state_stamp_ = stamp;

    geometry_msgs::msg::TransformStamped transform;
    transform.header = message->header;
    transform.header.frame_id = "map";
    transform.child_frame_id = "c1/base_link";
    transform.transform.translation.x = message->pose.pose.position.x;
    transform.transform.translation.y = message->pose.pose.position.y;
    transform.transform.rotation = message->pose.pose.orientation;
    tf_buffer_->setTransform(transform, "c1_rpp_lockstep_host", false);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = message->header;
    pose.header.frame_id = "map";
    pose.pose = message->pose.pose;
    try {
      auto command = controller_->computeVelocityCommands(pose, message->twist.twist, &goal_checker_);
      command.header = message->header;
      command.header.frame_id = "c1/base_link";
      command_pub_->publish(command);
      ++command_count_;
    } catch (const std::exception & error) {
      publish_fault(std::string{"nav2_rpp_exception:"} + error.what());
    }
  }

  void publish_fault(const std::string & fault)
  {
    if (fault_published_) {
      return;
    }
    fault_published_ = true;
    std_msgs::msg::String message;
    message.data = fault;
    fault_pub_->publish(message);
    RCLCPP_ERROR(get_logger(), "C1.2 controller fault: %s", fault.c_str());
  }

  pluginlib::ClassLoader<nav2_core::Controller> controller_loader_;
  nav2_core::Controller::Ptr controller_;
  FixedGoalChecker goal_checker_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fault_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::optional<nav_msgs::msg::Odometry> pending_odom_;
  std::optional<int64_t> last_state_stamp_;
  uint64_t duplicate_stamp_rejections_{0};
  uint64_t command_count_{0};
  bool path_ready_{false};
  bool fault_published_{false};
};

}  // namespace laksa_speed_race_nav2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<laksa_speed_race_nav2::RppLockstepHost>();
  try {
    node->initialize();
    rclcpp::spin(node->get_node_base_interface());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(node->get_logger(), "C1.2 lockstep host startup failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
