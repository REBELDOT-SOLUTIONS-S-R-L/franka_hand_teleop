// Copyright 2026 Robotics Lab. SPDX-License-Identifier: Apache-2.0
// Discrete IK proposals for the bounded-action bridge. Never integrates from a
// proposal that the bridge may have discarded, shortened or still be executing.
#include <moveit_servo/servo.hpp>
#include <moveit_servo/utils/common.hpp>
#include <moveit_msgs/msg/servo_status.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "franka_hand_controllers/rolling_config.hpp"
#include "franka_hand_controllers/joint_feedback.hpp"
#include "franka_hand_controllers/path_clearance.hpp"
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>

class FeedbackServo
{
public:
  explicit FeedbackServo(const rclcpp::Node::SharedPtr& node) : node_(node)
  {
    listener_ = std::make_shared<servo::ParamListener>(node_, "moveit_servo");
    scene_ = moveit_servo::createPlanningSceneMonitor(node_, listener_->get_params());
    servo_ = std::make_unique<moveit_servo::Servo>(node_, listener_, scene_);
    params_ = servo_->getParams();
    if(!node_->has_parameter("motion_profile")) node_->declare_parameter<std::string>("motion_profile", "");
    const auto path=node_->get_parameter("motion_profile").as_string();
    const auto config=franka_hand_controllers::load_motion_profile(path);
    limits_=config.limits; horizon_=config.proposal_horizon;
    RCLCPP_INFO(node_->get_logger(), "Motion profile: %s; tool speed=%.3f m/s acceleration=%.3f m/s2",path.c_str(),limits_.tool_speed,limits_.tool_acceleration);
    arm_joints_ = scene_->getRobotModel()->getJointModelGroup(params_.move_group_name)->getActiveJointModelNames();
    feedback_ = std::make_unique<franka_hand_controllers::JointFeedback>(arm_joints_);
    feedback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions feedback_options;
    feedback_options.callback_group = feedback_group_;
    arm_feedback_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "joint_state_broadcaster/joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) { feedback_->arm(*msg, node_->now().seconds()); },
      feedback_options);
    finger_feedback_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "gripper_joint_states", rclcpp::QoS(5).reliable(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) { feedback_->fingers(*msg, node_->now().seconds()); },
      feedback_options);
    merged_feedback_ = node_->create_publisher<sensor_msgs::msg::JointState>(params_.joint_topic, 1);
    // Collision validation can be expensive. Input reception and complete-state
    // publication must keep running independently of those callbacks.
    feedback_timer_ = node_->create_wall_timer(std::chrono::duration<double>(params_.publish_period),
      [this] { if (feedback_->reason(node_->now().seconds()).empty())
                 merged_feedback_->publish(feedback_->snapshot()); }, feedback_group_);
    if (!params_.check_collisions || params_.use_smoothing ||
        params_.command_out_type != "trajectory_msgs/JointTrajectory" ||
        !std::isfinite(horizon_) || horizon_ < .02 || horizon_ > .5)
      throw std::runtime_error("Feedback Servo requires collision checks, no position filter, and a 20-500 ms proposal horizon");
    servo_->setCommandType(moveit_servo::CommandType::TWIST);
    command_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
      params_.cartesian_command_in_topic, rclcpp::QoS(1),
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) { latest_ = msg; });
    output_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(params_.command_out_topic, 1);
    status_ = node_->create_publisher<moveit_msgs::msg::ServoStatus>(params_.status_topic, 1);
    switch_ = node_->create_service<moveit_msgs::srv::ServoCommandType>("~/switch_command_type",
      [](const std::shared_ptr<moveit_msgs::srv::ServoCommandType::Request> request,
         std::shared_ptr<moveit_msgs::srv::ServoCommandType::Response> response) {
        response->success = request->command_type == static_cast<int>(moveit_servo::CommandType::TWIST);
      });
    rolling_output_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("fr3_arm_controller/rolling_trajectory", 1);
    rolling_candidate_ = node_->create_subscription<trajectory_msgs::msg::JointTrajectory>("~/rolling_candidate", 1,
      [this](trajectory_msgs::msg::JointTrajectory::ConstSharedPtr message) { check_curve(*message); });
    timer_ = node_->create_wall_timer(std::chrono::duration<double>(params_.publish_period), [this] { tick(); });
    RCLCPP_INFO(node_->get_logger(), "Feedback-anchored Servo: every proposal starts at measured joints; collision checking enabled");
  }

private:
  std::string feedback_reason()
  {
    auto now = node_->now();
    auto reason = feedback_->reason(now.seconds());
    if (!reason.empty()) return reason;
    const auto monitor = scene_->getStateMonitor();
    std::vector<std::string> missing;
    auto model_current = [&] {
      missing.clear();
      const double age = (now-monitor->getCurrentStateTime()).seconds();
      return monitor->haveCompleteState(rclcpp::Duration::from_seconds(.15), missing) &&
             age >= -.02 && age < .15;
    };
    if (!model_current()) {
      // A newly arrived Hand sample may be in the input cache just before PSM
      // consumes the complete snapshot. Give that transport one bounded chance
      // to catch up, then recheck the original source deadlines at the new time.
      monitor->waitForCurrentState(rclcpp::Time(feedback_->snapshot().header.stamp), .02);
      now = node_->now();
      reason = feedback_->reason(now.seconds());
      if (!reason.empty()) return reason;
      if (!model_current()) {
        reason = "Complete joint snapshot missing or delayed in MoveIt";
        for (const auto& joint : missing) reason += " "+joint;
        return reason;
      }
    }
    return {};
  }

  void publish_status(moveit_servo::StatusCode code, const std::string& message)
  {
    if (code != last_code_ || message != last_message_)
    {
      if (code == moveit_servo::StatusCode::INVALID || code == moveit_servo::StatusCode::HALT_FOR_COLLISION ||
          code == moveit_servo::StatusCode::HALT_FOR_SINGULARITY || code == moveit_servo::StatusCode::JOINT_BOUND)
        RCLCPP_WARN(node_->get_logger(), "Servo status=%d: %s", static_cast<int>(code), message.c_str());
      last_code_ = code;
      last_message_ = message;
    }
    moveit_msgs::msg::ServoStatus status;
    status.code = static_cast<int8_t>(code);
    status.message = message;
    status_->publish(status);
  }

  void check_curve(const trajectory_msgs::msg::JointTrajectory& msg)
  {
    using namespace franka_hand_controllers;
    const auto began = std::chrono::steady_clock::now();
    RollingPacket packet;
    if (!rolling_id(msg.header.frame_id, packet) || msg.joint_names != arm_joints_ || msg.points.size()!=2)
      return;
    const auto& first=msg.points.front(); const auto& last=msg.points.back();
    if (first.positions.size()!=7 || first.velocities.size()!=7 || first.accelerations.size()!=7 ||
        last.positions.size()!=7 || last.velocities.size()!=7 || last.accelerations.size()!=7 ||
        rclcpp::Duration(first.time_from_start).nanoseconds()!=0) return;
    auto& curve=packet.curve;
    curve.start=rclcpp::Time(msg.header.stamp).seconds();
    curve.duration=rclcpp::Duration(last.time_from_start).seconds();
    for(size_t i=0;i<7;++i) {
      if(last.velocities[i]!=0. || last.accelerations[i]!=0.) return;
      curve.initial.q[i]=first.positions[i]; curve.initial.v[i]=first.velocities[i];
      curve.initial.a[i]=first.accelerations[i]; curve.target[i]=last.positions[i];
    }
    curve.build();
    // Same fixed reach as controller; bridge verifies this against its URDF guard.
    if(!curve.valid(1.722834753163608,limits_)) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                          "Rolling candidate rejected: derivative/travel limits");
      return;
    }
    const auto input_reason=feedback_reason();
    if(!input_reason.empty()) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                          "Rolling candidate rejected: %s", input_reason.c_str());
      return;
    }
    if(last_code_!=moveit_servo::StatusCode::NO_WARNING &&
       last_code_!=moveit_servo::StatusCode::DECELERATE_FOR_COLLISION &&
       last_code_!=moveit_servo::StatusCode::DECELERATE_FOR_APPROACHING_SINGULARITY &&
       last_code_!=moveit_servo::StatusCode::DECELERATE_FOR_LEAVING_SINGULARITY) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                          "Rolling candidate rejected: Servo status=%d", static_cast<int>(last_code_));
      return;
    }
    planning_scene_monitor::LockedPlanningSceneRO locked(scene_);
    auto state=locked->getCurrentState();
    // At the conservative whole-arm speed cap, <=1 mm motion per sample plus
    // >=1 mm clearance enclose motion between samples. Retain all existing ACM pairs.
    const int count=std::max(1, static_cast<int>(std::ceil(
      curve.duration*curve.speed_bound(1.722834753163608)/.001)));
    for(int tick=0; tick<=count; ++tick) {
      const auto point=curve.at(curve.start+curve.duration*tick/count);
      state.setJointGroupPositions(params_.move_group_name, point.q.data()); state.update();
      if(!state.satisfiesBounds() || !path_clearance(*locked.operator->(), state)) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                            "Rolling candidate rejected: joint bounds or collision clearance at sample %d/%d",
                            tick, count);
        return;
      }
    }
    const double validation_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now()-began).count();
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "ROLLING VALIDATION: epoch=%lu sequence=%lu samples=%d elapsed_ms=%.3f",
      packet.epoch, packet.sequence, count+1, validation_ms);
    const auto final_reason=feedback_->reason(node_->now().seconds());
    if (final_reason.empty()) rolling_output_->publish(msg);
    else RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Rolling candidate rejected after path validation: %s", final_reason.c_str());
  }

  void tick()
  {
    const auto input_reason = feedback_reason();
    if (!input_reason.empty()) {
      publish_status(moveit_servo::StatusCode::INVALID, input_reason);
      return;
    }
    const auto now = node_->now();
    const auto monitor = scene_->getStateMonitor();
    if (!latest_ || (now-latest_->header.stamp).seconds() < -.02 ||
        (now-latest_->header.stamp).seconds() > params_.incoming_command_timeout)
    {
      publish_status(moveit_servo::StatusCode::INVALID, "Twist command missing or stale");
      return;
    }
    auto state = monitor->getCurrentState();
    // IK and collision/singularity scaling use fresh measured state. The bridge
    // consumes an increment with its exact seed, never an absolute command.
    // Rest-to-rest interpolation belongs to the bridge/JTC, not a reset filter.
    const auto seed = moveit_servo::extractRobotState(state, params_.move_group_name);
    const auto& t = latest_->twist;
    const moveit_servo::TwistCommand twist{latest_->header.frame_id,
      Eigen::Vector<double, 6>{t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z}};
    const auto next = servo_->getNextJointState(state, twist);
    const auto code = servo_->getStatus();
    publish_status(code, servo_->getStatusMessage());
    if (code != moveit_servo::StatusCode::NO_WARNING &&
        code != moveit_servo::StatusCode::DECELERATE_FOR_COLLISION &&
        code != moveit_servo::StatusCode::DECELERATE_FOR_APPROACHING_SINGULARITY &&
        code != moveit_servo::StatusCode::DECELERATE_FOR_LEAVING_SINGULARITY)
      return;
    if (!next.positions.allFinite() || seed.joint_names != next.joint_names)
      return;
    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.header.stamp = now;
    trajectory.header.frame_id = "fr3_link0";
    trajectory.joint_names = next.joint_names;
    trajectory_msgs::msg::JointTrajectoryPoint first, last;
    first.positions.assign(seed.positions.data(), seed.positions.data()+seed.positions.size());
    const Eigen::VectorXd endpoint = seed.positions +
      (next.positions-seed.positions)*(horizon_/params_.publish_period);
    last.positions.assign(endpoint.data(), endpoint.data()+endpoint.size());
    last.time_from_start = rclcpp::Duration::from_seconds(horizon_);
    trajectory.points = {first, last};
    output_->publish(trajectory);
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<servo::ParamListener> listener_;
  planning_scene_monitor::PlanningSceneMonitorPtr scene_;
  std::unique_ptr<moveit_servo::Servo> servo_;
  servo::Params params_;
  double horizon_;
  franka_hand_controllers::RollingLimits limits_;
  std::vector<std::string> arm_joints_;
  std::unique_ptr<franka_hand_controllers::JointFeedback> feedback_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arm_feedback_, finger_feedback_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr merged_feedback_;
  rclcpp::CallbackGroup::SharedPtr feedback_group_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;
  geometry_msgs::msg::TwistStamped::ConstSharedPtr latest_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr command_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr output_;
  rclcpp::Publisher<moveit_msgs::msg::ServoStatus>::SharedPtr status_;
  rclcpp::Service<moveit_msgs::srv::ServoCommandType>::SharedPtr switch_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr rolling_output_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr rolling_candidate_;
  rclcpp::TimerBase::SharedPtr timer_;
  moveit_servo::StatusCode last_code_ = moveit_servo::StatusCode::INVALID;
  std::string last_message_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("servo_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  try
  {
    FeedbackServo servo(node);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
  }
  catch (const std::exception& error)
  {
    RCLCPP_FATAL(node->get_logger(), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
