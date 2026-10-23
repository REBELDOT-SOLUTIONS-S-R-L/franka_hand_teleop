#pragma once
#include "franka_hand_controllers/rolling_curve.hpp"
#include <yaml-cpp/yaml.h>
#include <stdexcept>

namespace franka_hand_controllers {
struct MotionConfig {
  RollingLimits limits;
  double proposal_horizon;
};
inline MotionConfig load_motion_profile(const std::string& path) {
  if(path.empty()) throw std::invalid_argument("motion_profile is required; no implicit motion limits");
  const auto profile=YAML::LoadFile(path);
  const auto m=profile["motion"];
  MotionConfig c;
  c.limits.joint_step=m["max_joint_step"].as<double>();
  c.limits.tool_travel=m["max_tool_step"].as<double>();
  c.limits.tool_speed=m["max_speed"].as<double>();
  c.limits.tool_acceleration=m["max_acceleration"].as<double>();
  c.limits.joint_speed=m["joint_speed"].as<double>();
  c.limits.joint_acceleration=m["joint_acceleration"].as<double>();
  c.limits.joint_jerk=m["joint_jerk"].as<double>();
  c.limits.maximum_horizon=m["max_curve_duration_s"].as<double>();
  c.proposal_horizon=profile["bridge"]["proposal_horizon_s"].as<double>();
  if(!c.limits.valid() || !std::isfinite(c.proposal_horizon) || c.proposal_horizon<.02 || c.proposal_horizon>.5)
    throw std::invalid_argument("Motion profile exceeds FR3 envelope or stream protocol bounds");
  return c;
}
}  // namespace franka_hand_controllers
