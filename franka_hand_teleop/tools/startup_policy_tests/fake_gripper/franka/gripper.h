#pragma once
// Compiled only into gripper_feedback_tests. The Gripper token is renamed to
// OfflineGripper for this target, so no physical constructor can be invoked.
#include <franka/exception.h>
#include <franka/gripper_state.h>
#include <atomic>
#include <string>
namespace franka {
namespace offline {
inline std::atomic<bool> fail_reads{false};
inline std::atomic<unsigned> motion_calls{0};
}
class Gripper {
 public:
  explicit Gripper(const std::string& address) {
    if(address!="offline-fixture") throw Exception("Offline fixture refuses real endpoints");
  }
  GripperState readOnce() const {
    if(offline::fail_reads) throw Exception("Synthetic gripper read failure");
    GripperState result;
    result.width=.04; result.max_width=.08;
    return result;
  }
  bool homing() { ++offline::motion_calls; return false; }
  bool move(double,double) { ++offline::motion_calls; return false; }
  bool grasp(double,double,double,double,double) { ++offline::motion_calls; return false; }
  bool stop() { ++offline::motion_calls; return false; }
};
}
