#pragma once

#include <joint_trajectory_controller/joint_trajectory_controller.hpp>

namespace franka_hand_controllers {
class RobotClockTrajectoryController
    : public joint_trajectory_controller::JointTrajectoryController {
 public:
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;

 private:
  void replace_stop_request();
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> continuous_stop_;
  hardware_interface::LoanedStateInterface* robot_clock_{nullptr};
  bool clock_started_{false};
  int64_t previous_robot_ns_{0};
  rclcpp::Time interpolation_time_{0, 0, RCL_ROS_TIME};
};
}  // namespace franka_hand_controllers
