#include "franka_hand_controllers/robot_clock_controller.hpp"
#include "franka_hand_controllers/bounded_stop.hpp"

#include <cmath>
#include <optional>
#include <pluginlib/class_list_macros.hpp>

namespace franka_hand_controllers {
controller_interface::InterfaceConfiguration
RobotClockTrajectoryController::state_interface_configuration() const {
  auto config = JointTrajectoryController::state_interface_configuration();
  config.names.push_back("fr3/robot_time");
  return config;
}

controller_interface::CallbackReturn RobotClockTrajectoryController::on_configure(
    const rclcpp_lifecycle::State& state) {
  const auto result = JointTrajectoryController::on_configure(state);
  if (result != controller_interface::CallbackReturn::SUCCESS)
    return result;
  if (get_update_rate() != 1000 ||
      params_.command_interfaces != std::vector<std::string>{"position"} ||
      !params_.constraints.decelerate_on_cancel) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "FR3 robot-clock controller requires position commands at 1000 Hz and cancellation enabled");
    return controller_interface::CallbackReturn::ERROR;
  }
  continuous_stop_ = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  continuous_stop_->joint_names = params_.joints;
  continuous_stop_->points.resize(2);
  for (auto& point : continuous_stop_->points) {
    point.positions.resize(dof_);
    point.velocities.resize(dof_);
    point.accelerations.resize(dof_);
  }
  return result;
}

void RobotClockTrajectoryController::replace_stop_request() {
  const auto incoming = *new_trajectory_msg_.readFromRT();
  if (incoming != stop_trajectory_ && incoming != hold_position_msg_ptr_)
    return;
  const auto original = current_trajectory_->get_trajectory_msg();
  // Never use measured positions to replace a continuous position command.
  // The installed JTC's constant-deceleration cancel jumps both position and
  // acceleration; its action callback publishes stop_trajectory_ as a marker.
  const auto& last = last_commanded_state_;
  if (last.positions.size() != dof_ || last.velocities.size() != dof_ ||
      last.accelerations.size() != dof_)
    return;
  auto& first = continuous_stop_->points[0];
  auto& end = continuous_stop_->points[1];
  first = last;
  first.time_from_start = rclcpp::Duration::from_seconds(0.);
  end = first;
  end.velocities.assign(dof_, 0.);
  end.accelerations.assign(dof_, 0.);
  end.time_from_start = rclcpp::Duration::from_seconds(.001);
  bool at_rest = true;
  for (size_t i = 0; i < dof_; ++i)
    at_rest = at_rest && std::abs(last.velocities[i]) < 1e-12 && std::abs(last.accelerations[i]) < 1e-10;
  if (!at_rest && original && original->points.size() == 2) {
    const auto& a = original->points.front().positions;
    const auto& b = original->points.back().positions;
    size_t axis = 0;
    for (size_t i = 1; i < dof_; ++i)
      if (std::abs(b[i]-a[i]) > std::abs(b[axis]-a[axis]))
        axis = i;
    const double distance = b[axis]-a[axis];
    const double total = rclcpp::Duration(original->points.back().time_from_start).seconds();
    const double remaining = total - (traj_time_-current_trajectory_->time_from_start()).seconds() - .001;
    ProgressStop stop;
    if (std::abs(distance) > 1e-12 &&
        choose_progress_stop((last.positions[axis]-a[axis])/distance,
          last.velocities[axis]/distance, last.accelerations[axis]/distance, total, remaining, stop)) {
      for (size_t i = 0; i < dof_; ++i)
        end.positions[i] = a[i] + (b[i]-a[i])*stop.endpoint;
      end.time_from_start = rclcpp::Duration::from_seconds(stop.duration);
    } else {
      // No earlier stop satisfying the existing derivative/path limits: keep
      // the remaining, already guarded quintic, never restart or jump to q.
      new_trajectory_msg_.initRT(original);
      return;
    }
  } else if (!at_rest) {
    new_trajectory_msg_.initRT(original);
    return;
  }
  new_trajectory_msg_.initRT(continuous_stop_);
}

controller_interface::CallbackReturn RobotClockTrajectoryController::on_activate(
    const rclcpp_lifecycle::State& state) {
  clock_started_ = false;
  robot_clock_ = nullptr;
  for (auto& item : state_interfaces_) {
    if (item.get_name() == "fr3/robot_time")
      robot_clock_ = &item;
  }
  if (!robot_clock_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Required fr3/robot_time state interface is missing");
    return controller_interface::CallbackReturn::ERROR;
  }
  const auto result = JointTrajectoryController::on_activate(state);
  if (result == controller_interface::CallbackReturn::SUCCESS) {
    // Position-only activation holds the seeded FCI command. Measured encoder
    // velocity is not the derivative of that constant command.
    last_commanded_state_.velocities.assign(dof_, 0.);
    last_commanded_state_.accelerations.assign(dof_, 0.);
  }
  return result;
}

controller_interface::return_type RobotClockTrajectoryController::update(
    const rclcpp::Time& time,
    const rclcpp::Duration& /*host_period*/) {
  const auto value = robot_clock_ ? robot_clock_->get_optional() : std::nullopt;
  if (!value || !std::isfinite(*value) || *value < 0. || *value > 1e9) {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid FR3 robot clock; refusing trajectory update");
    return controller_interface::return_type::ERROR;
  }
  const auto robot_ns = static_cast<int64_t>(std::llround(*value * 1e9));
  int64_t elapsed_ns = 1000000;
  if (clock_started_) {
    elapsed_ns = robot_ns - previous_robot_ns_;
    if (elapsed_ns <= 0 || elapsed_ns >= 20000000) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "FR3 robot clock stale/reversed or gap >=20 ms; refusing update");
      return controller_interface::return_type::ERROR;
    }
    interpolation_time_ = interpolation_time_ + rclcpp::Duration(0, elapsed_ns);
  } else {
    interpolation_time_ = time;
    clock_started_ = true;
  }
  previous_robot_ns_ = robot_ns;
  const auto trajectory = *new_trajectory_msg_.readFromRT();
  if (trajectory && (trajectory->header.stamp.sec || trajectory->header.stamp.nanosec)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "FR3 robot-clock trajectories require zero header stamp (start now)");
    return controller_interface::return_type::ERROR;
  }
  if (trajectory && trajectory != current_trajectory_->get_trajectory_msg() &&
      trajectory != stop_trajectory_ && trajectory != hold_position_msg_ptr_ && trajectory != continuous_stop_) {
    bool valid = trajectory->points.size() == 2 && trajectory->joint_names == params_.joints;
    if (valid) {
      const double total = rclcpp::Duration(trajectory->points.back().time_from_start).seconds();
      valid = total > 0. && total <= 1.5 &&
        rclcpp::Duration(trajectory->points.front().time_from_start).nanoseconds() == 0;
      for (const auto& point : trajectory->points) {
        valid = valid && point.positions.size() == dof_ && point.velocities.size() == dof_ &&
          point.accelerations.size() == dof_;
        for (const auto q : point.positions) valid = valid && std::isfinite(q);
        for (const auto v : point.velocities) valid = valid && v == 0.;
        for (const auto a : point.accelerations) valid = valid && a == 0.;
      }
    }
    if (!valid) {
      RCLCPP_ERROR(get_node()->get_logger(), "FR3 controller requires a bounded, two-point rest-to-rest trajectory");
      return controller_interface::return_type::ERROR;
    }
  }
  if (trajectory && trajectory != current_trajectory_->get_trajectory_msg())
    replace_stop_request();
  // Use elapsed robot time, including skipped robot cycles. Host scheduling
  // jitter must not fast-forward positions on an otherwise regular FCI stream.
  return JointTrajectoryController::update(interpolation_time_, rclcpp::Duration(0, elapsed_ns));
}
}  // namespace franka_hand_controllers

PLUGINLIB_EXPORT_CLASS(franka_hand_controllers::RobotClockTrajectoryController,
                       controller_interface::ControllerInterface)
