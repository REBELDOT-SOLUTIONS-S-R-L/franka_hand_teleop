#pragma once
#include <sensor_msgs/msg/joint_state.hpp>
#include <algorithm>
#include <cmath>
#include <string>
#include <mutex>
#include <vector>

namespace franka_hand_controllers {
// Source timestamps remain authoritative. A combined snapshot is stamped at
// its oldest component; publication never turns a cached sample into new data.
class JointFeedback {
 public:
  explicit JointFeedback(std::vector<std::string> arm_names) : arm_names_(std::move(arm_names)) {}
  void arm(const sensor_msgs::msg::JointState& msg, double now) {
    std::lock_guard<std::mutex> lock(mutex_);
    receive(arm_, msg, arm_names_, now, .10, false);
  }
  void fingers(const sensor_msgs::msg::JointState& msg, double now) {
    std::lock_guard<std::mutex> lock(mutex_);
    receive(fingers_, msg, finger_names_, now, .15, true);
  }
  std::string reason(double now) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!fresh(arm_, now, .10)) return "Arm joint feedback missing, invalid or stale (100 ms)";
    if (!fresh(fingers_, now, .15)) return "Gripper joint feedback missing, invalid or stale (150 ms): fr3_finger_joint1";
    return {};
  }
  sensor_msgs::msg::JointState snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = arm_.time <= fingers_.time ? arm_.stamp : fingers_.stamp;
    msg.name = arm_names_;
    msg.name.insert(msg.name.end(), finger_names_.begin(), finger_names_.end());
    msg.position = arm_.positions;
    msg.position.insert(msg.position.end(), fingers_.positions.begin(), fingers_.positions.end());
    msg.velocity = arm_.velocities;
    msg.velocity.insert(msg.velocity.end(), fingers_.velocities.begin(), fingers_.velocities.end());
    return msg;
  }
 private:
  struct Sample {
    bool valid{false};
    double time{0.};
    builtin_interfaces::msg::Time stamp;
    std::vector<double> positions, velocities;
  };
  static bool fresh(const Sample& sample, double now, double deadline) {
    return sample.valid && std::isfinite(now) && now-sample.time >= -.02 && now-sample.time < deadline;
  }
  static void receive(Sample& output, const sensor_msgs::msg::JointState& msg,
                      const std::vector<std::string>& names, double now, double deadline, bool finger) {
    const double stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9;
    if (!std::isfinite(now) || !std::isfinite(stamp) || stamp <= 0. ||
        now-stamp < -.02 || now-stamp >= deadline || msg.name.size()!=msg.position.size()) {
      output.valid = false;
      return;
    }
    Sample next;
    next.time=stamp; next.stamp=msg.header.stamp;
    for (const auto& name : names) {
      const auto found=std::find(msg.name.begin(),msg.name.end(),name);
      if (found==msg.name.end() || std::count(msg.name.begin(),msg.name.end(),name)!=1) {
        output.valid=false; return;
      }
      const auto i=std::distance(msg.name.begin(),found);
      const double q=msg.position[i];
      const double v=msg.velocity.empty()?0.:msg.velocity.size()==msg.name.size()?msg.velocity[i]:NAN;
      if (!std::isfinite(q) || !std::isfinite(v) || (finger && (q<0. || q>.04))) {
        output.valid=false; return;
      }
      next.positions.push_back(q); next.velocities.push_back(v);
    }
    // Late/repeated messages cannot refresh a source timestamp or replace newer data.
    if (output.valid && stamp <= output.time) return;
    next.valid=true;
    output=std::move(next);
  }
  std::vector<std::string> arm_names_;
  const std::vector<std::string> finger_names_{"fr3_finger_joint1","fr3_finger_joint2"};
  Sample arm_, fingers_;
  mutable std::mutex mutex_;
};
}  // namespace franka_hand_controllers
