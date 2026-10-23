#pragma once
#include "franka_hand_controllers/robot_clock_controller.hpp"
#include "franka_hand_controllers/rolling_config.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <atomic>

namespace franka_hand_controllers {
class GuardedRollingController : public RobotClockTrajectoryController {
 public:
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
  controller_interface::return_type update(const rclcpp::Time&,const rclcpp::Duration&) override;
 protected:
  void receive(const trajectory_msgs::msg::JointTrajectory&);
  struct Control { uint64_t epoch{0}; bool enabled{false}; };
  realtime_tools::RealtimeBuffer<Control> controls_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<RollingPacket>> packets_;
 protected:
  void report(double time);
  hardware_interface::LoanedStateInterface* clock_{nullptr};
  double previous_time_{-1.}, reach_{1.722834753163608};
  uint64_t epoch_{0}, sequence_{0}, rejected_{0}, ticks_{0};
  uint64_t reported_rejection_{0};
  std::array<double, 16> rejection_detail_{};
  bool enabled_{false}, has_active_{false}, has_pending_{false};
  RollingLimits limits_;
  RollingCurve active_, pending_, accepted_;
  RollingState command_;
  std::shared_ptr<RollingPacket> seen_;
  std::atomic<uint64_t> epoch_counter_{0};
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr rolling_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> status_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> rejection_status_;
};
}
