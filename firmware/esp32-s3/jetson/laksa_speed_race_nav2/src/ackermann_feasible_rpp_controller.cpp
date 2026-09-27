// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
// Copyright 2026 Leobardo Gomez
//
// Licensed under the Apache License, Version 2.0
//
// computeVelocityCommands() follows Navigation2 1.1.20 at
// a097086719c88f781aa59788eca29ac6ca5e56db. The sole tracking-behavior delta
// is constrain_ackermann_curvature() between upstream velocity regulation and
// upstream TTC evaluation; TTC and the returned Twist use the same command.

#include "laksa_speed_race_nav2/ackermann_feasible_rpp_controller.hpp"

#include <cmath>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include "laksa_speed_race_nav2/ackermann_feasibility.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

namespace laksa_speed_race_nav2
{

void AckermannFeasibleRppController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  RegulatedPurePursuitController::configure(parent, std::move(name), tf, costmap_ros);
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock lifecycle node for feasibility telemetry");
  }
  feasibility_pub_ = node->create_publisher<std_msgs::msg::String>(
    "/c1/rpp_feasibility", rclcpp::QoS(100).reliable());
}

void AckermannFeasibleRppController::activate()
{
  RegulatedPurePursuitController::activate();
  feasibility_pub_->on_activate();
}

void AckermannFeasibleRppController::deactivate()
{
  feasibility_pub_->on_deactivate();
  RegulatedPurePursuitController::deactivate();
}

void AckermannFeasibleRppController::publish_feasibility(
  const geometry_msgs::msg::PoseStamped & pose,
  const double requested_curvature, const double linear_velocity,
  const double angular_before_feasibility, const double commanded_curvature,
  const double commanded_angular_velocity, const double equivalent_steering,
  const bool saturated, const bool ttc_passed)
{
  std::ostringstream json;
  json << std::setprecision(17)
       << "{\"stamp_ns\":"
       << (static_cast<int64_t>(pose.header.stamp.sec) * 1000000000LL + pose.header.stamp.nanosec)
       << ",\"kappa_req\":" << requested_curvature
       << ",\"kappa_max\":" << maximum_ackermann_curvature_1pm()
       << ",\"kappa_cmd\":" << commanded_curvature
       << ",\"v_cmd\":" << linear_velocity
       << ",\"omega_pre_feasibility\":" << angular_before_feasibility
       << ",\"omega_cmd\":" << commanded_angular_velocity
       << ",\"delta_equivalent\":" << equivalent_steering
       << ",\"curvature_saturated\":" << (saturated ? "true" : "false")
       << ",\"ttc_pass\":" << (ttc_passed ? "true" : "false")
       << ",\"ttc_horizon_s\":" << max_allowed_time_to_collision_up_to_carrot_
       << "}";
  std_msgs::msg::String message;
  message.data = json.str();
  feasibility_pub_->publish(message);
}

geometry_msgs::msg::TwistStamped AckermannFeasibleRppController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed,
  nav2_core::GoalChecker * goal_checker)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist vel_tolerance;
  if (!goal_checker->getTolerances(pose_tolerance, vel_tolerance)) {
    RCLCPP_WARN(logger_, "Unable to retrieve goal checker's tolerances!");
  } else {
    goal_dist_tol_ = pose_tolerance.position.x;
  }

  auto transformed_plan = transformGlobalPlan(pose);
  double lookahead_dist = getLookAheadDistance(speed);

  if (allow_reversing_) {
    const double dist_to_cusp = findVelocitySignChange(transformed_plan);
    if (dist_to_cusp < lookahead_dist) {
      lookahead_dist = dist_to_cusp;
    }
  }

  auto carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);
  carrot_pub_->publish(createCarrotMsg(carrot_pose));

  double linear_vel = desired_linear_vel_;
  double angular_vel = 0.0;
  const double carrot_dist2 =
    (carrot_pose.pose.position.x * carrot_pose.pose.position.x) +
    (carrot_pose.pose.position.y * carrot_pose.pose.position.y);
  double requested_curvature = 0.0;
  if (carrot_dist2 > 0.001) {
    requested_curvature = 2.0 * carrot_pose.pose.position.y / carrot_dist2;
  }

  double sign = 1.0;
  if (allow_reversing_) {
    sign = carrot_pose.pose.position.x >= 0.0 ? 1.0 : -1.0;
  }

  double angle_to_heading = 0.0;
  if (shouldRotateToGoalHeading(carrot_pose)) {
    const double angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);
    rotateToHeading(linear_vel, angular_vel, angle_to_goal, speed);
  } else if (shouldRotateToPath(carrot_pose, angle_to_heading)) {
    rotateToHeading(linear_vel, angular_vel, angle_to_heading, speed);
  } else {
    applyConstraints(
      requested_curvature, speed,
      costAtPose(pose.pose.position.x, pose.pose.position.y), transformed_plan,
      linear_vel, sign);
    angular_vel = linear_vel * requested_curvature;
  }

  const double angular_before_feasibility = angular_vel;
  const double curvature_before_feasibility =
    std::abs(linear_vel) <= kZeroVelocityEpsilonMps ? 0.0 : angular_vel / linear_vel;
  const auto feasible = constrain_ackermann_curvature(
    linear_vel, curvature_before_feasibility);
  linear_vel = feasible.linear_mps;
  const double commanded_curvature = feasible.commanded_curvature_1pm;
  angular_vel = feasible.angular_rps;

  const double carrot_dist = std::hypot(
    carrot_pose.pose.position.x, carrot_pose.pose.position.y);
  const bool collision = use_collision_detection_ &&
    isCollisionImminent(pose, linear_vel, angular_vel, carrot_dist);
  publish_feasibility(
    pose, requested_curvature, linear_vel, angular_before_feasibility,
    commanded_curvature, angular_vel,
    feasible.equivalent_steering_rad, feasible.curvature_saturated, !collision);
  if (collision) {
    throw nav2_core::PlannerException(
            "AckermannFeasibleRppController detected collision ahead!");
  }

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = angular_vel;
  return cmd_vel;
}

}  // namespace laksa_speed_race_nav2

PLUGINLIB_EXPORT_CLASS(
  laksa_speed_race_nav2::AckermannFeasibleRppController,
  nav2_core::Controller)
