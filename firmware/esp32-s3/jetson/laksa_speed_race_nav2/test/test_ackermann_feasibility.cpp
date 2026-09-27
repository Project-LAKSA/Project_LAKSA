// Copyright 2026 Leobardo Gomez
// Licensed under the Apache License, Version 2.0

#include <cmath>

#include "gtest/gtest.h"
#include "laksa_speed_race_nav2/ackermann_feasibility.hpp"

namespace
{

using laksa_speed_race_nav2::constrain_ackermann_curvature;
using laksa_speed_race_nav2::kAckermannWheelbaseM;
using laksa_speed_race_nav2::kSteeringLimitRad;
using laksa_speed_race_nav2::maximum_ackermann_curvature_1pm;

TEST(AckermannFeasibility, CurvatureCases)
{
  const double limit = maximum_ackermann_curvature_1pm();
  EXPECT_DOUBLE_EQ(constrain_ackermann_curvature(1.0, 0.5).commanded_curvature_1pm, 0.5);
  EXPECT_DOUBLE_EQ(constrain_ackermann_curvature(1.0, -0.5).commanded_curvature_1pm, -0.5);
  EXPECT_DOUBLE_EQ(constrain_ackermann_curvature(1.0, limit * 2.0).commanded_curvature_1pm, limit);
  EXPECT_DOUBLE_EQ(constrain_ackermann_curvature(1.0, -limit * 2.0).commanded_curvature_1pm, -limit);
  EXPECT_DOUBLE_EQ(constrain_ackermann_curvature(1.0, limit).commanded_curvature_1pm, limit);
  EXPECT_DOUBLE_EQ(constrain_ackermann_curvature(1.0, -limit).commanded_curvature_1pm, -limit);
  EXPECT_DOUBLE_EQ(constrain_ackermann_curvature(1.0, 0.0).commanded_curvature_1pm, 0.0);
}

TEST(AckermannFeasibility, ZeroVelocityIsFiniteAndStopped)
{
  const auto command = constrain_ackermann_curvature(0.0, 9.0);
  EXPECT_TRUE(std::isfinite(command.angular_rps));
  EXPECT_TRUE(std::isfinite(command.equivalent_steering_rad));
  EXPECT_DOUBLE_EQ(command.angular_rps, 0.0);
  EXPECT_DOUBLE_EQ(command.equivalent_steering_rad, 0.0);
}

TEST(AckermannFeasibility, OmegaAndSteeringRespectPhysicalEnvelope)
{
  for (const double requested : {-4.0, -0.5, 0.0, 0.5, 4.0}) {
    const auto command = constrain_ackermann_curvature(0.4285508430, requested);
    EXPECT_DOUBLE_EQ(
      command.angular_rps,
      command.linear_mps * command.commanded_curvature_1pm);
    EXPECT_LE(std::abs(command.commanded_curvature_1pm), maximum_ackermann_curvature_1pm());
    EXPECT_LE(std::abs(command.equivalent_steering_rad), kSteeringLimitRad + 1.0e-15);
    EXPECT_NEAR(
      command.equivalent_steering_rad,
      std::atan(kAckermannWheelbaseM * command.angular_rps / command.linear_mps), 1.0e-15);
  }
}

TEST(AckermannFeasibility, ReproducesC12aInitialConstraint)
{
  const auto command = constrain_ackermann_curvature(0.4285508430, -2.1334891826);
  EXPECT_NEAR(command.commanded_curvature_1pm, -0.9143085878, 1.0e-10);
  EXPECT_NEAR(command.angular_rps, -0.3918277161, 1.0e-10);
  EXPECT_NEAR(command.equivalent_steering_rad, -0.288, 1.0e-15);
  EXPECT_TRUE(command.curvature_saturated);
}

}  // namespace
