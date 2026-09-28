// Copyright 2026 Leobardo Gomez
// Licensed under the Apache License, Version 2.0

#ifndef LAKSA_SPEED_RACE_NAV2__ACKERMANN_FEASIBILITY_HPP_
#define LAKSA_SPEED_RACE_NAV2__ACKERMANN_FEASIBILITY_HPP_

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace laksa_speed_race_nav2
{

constexpr double kAckermannWheelbaseM = 0.324;
constexpr double kSteeringLimitRad = 0.288;
constexpr double kZeroVelocityEpsilonMps = 1.0e-9;

struct FeasibleCommand
{
  double linear_mps;
  double requested_curvature_1pm;
  double commanded_curvature_1pm;
  double angular_rps;
  double equivalent_steering_rad;
  bool curvature_saturated;
};

struct AckermannTwistValidation
{
  double curvature_1pm;
  double equivalent_steering_rad;
  bool feasible;
};

inline double maximum_ackermann_curvature_1pm()
{
  return std::tan(kSteeringLimitRad) / kAckermannWheelbaseM;
}

inline FeasibleCommand constrain_ackermann_curvature(
  const double linear_mps, const double requested_curvature_1pm)
{
  if (!std::isfinite(linear_mps) || !std::isfinite(requested_curvature_1pm)) {
    throw std::invalid_argument("Ackermann feasibility input must be finite");
  }
  const double curvature_limit = maximum_ackermann_curvature_1pm();
  const double commanded_curvature = std::clamp(
    requested_curvature_1pm, -curvature_limit, curvature_limit);
  const bool stopped = std::abs(linear_mps) <= kZeroVelocityEpsilonMps;
  const double angular_rps = stopped ? 0.0 : linear_mps * commanded_curvature;
  const double steering_rad = stopped ? 0.0 :
    std::atan(kAckermannWheelbaseM * angular_rps / linear_mps);
  return FeasibleCommand{
    linear_mps,
    requested_curvature_1pm,
    commanded_curvature,
    angular_rps,
    steering_rad,
    requested_curvature_1pm != commanded_curvature};
}

inline AckermannTwistValidation validate_ackermann_twist(
  const double linear_mps, const double angular_rps)
{
  if (!std::isfinite(linear_mps) || !std::isfinite(angular_rps)) {
    return {0.0, 0.0, false};
  }
  if (linear_mps < 0.0) {
    return {0.0, 0.0, false};
  }
  if (std::abs(linear_mps) <= kZeroVelocityEpsilonMps) {
    return {0.0, 0.0, std::abs(angular_rps) <= kZeroVelocityEpsilonMps};
  }
  const double curvature = angular_rps / linear_mps;
  const double steering = std::atan(kAckermannWheelbaseM * curvature);
  return {
    curvature,
    steering,
    std::abs(curvature) <= maximum_ackermann_curvature_1pm() + 1.0e-9 &&
    std::abs(steering) <= kSteeringLimitRad + 1.0e-9};
}

}  // namespace laksa_speed_race_nav2

#endif  // LAKSA_SPEED_RACE_NAV2__ACKERMANN_FEASIBILITY_HPP_
