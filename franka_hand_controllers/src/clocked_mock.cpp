#include <hardware_interface/mock_components/generic_system.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace franka_hand_controllers {
// Test fixture only. The real deployment refuses this plugin's identity.
class ClockedMockSystem : public mock_components::GenericSystem {
 public:
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override {
    const auto result = GenericSystem::read(time, period);
    ++ticks_;
    set_state("fr3/robot_time", static_cast<double>(ticks_) * .001);
    return result;
  }

 private:
  uint64_t ticks_{0};
};
}  // namespace franka_hand_controllers

PLUGINLIB_EXPORT_CLASS(franka_hand_controllers::ClockedMockSystem,
                       hardware_interface::SystemInterface)
