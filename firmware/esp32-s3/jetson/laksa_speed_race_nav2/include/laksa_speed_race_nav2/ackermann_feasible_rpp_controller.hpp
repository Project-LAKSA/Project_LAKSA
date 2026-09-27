// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
// Copyright 2026 Leobardo Gomez
//
// Licensed under the Apache License, Version 2.0

#ifndef LAKSA_SPEED_RACE_NAV2__ACKERMANN_FEASIBLE_RPP_CONTROLLER_HPP_
#define LAKSA_SPEED_RACE_NAV2__ACKERMANN_FEASIBLE_RPP_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/string.hpp"

namespace laksa_speed_race_nav2
{

class AckermannFeasibleRppController :
  public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & speed,
    nav2_core::GoalChecker * goal_checker) override;

private:
  void publish_feasibility(
    const geometry_msgs::msg::PoseStamped & pose,
    double requested_curvature, double linear_velocity,
    double angular_before_feasibility, double commanded_curvature,
    double commanded_angular_velocity, double equivalent_steering,
    bool saturated, bool ttc_passed);

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>>
  feasibility_pub_;
};

}  // namespace laksa_speed_race_nav2

#endif  // LAKSA_SPEED_RACE_NAV2__ACKERMANN_FEASIBLE_RPP_CONTROLLER_HPP_
