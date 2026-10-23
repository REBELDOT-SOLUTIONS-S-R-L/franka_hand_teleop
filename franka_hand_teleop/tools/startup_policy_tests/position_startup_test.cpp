#include <fstream>
#include <limits>
#include <sstream>

#include <franka/exception.h>
#include <gmock/gmock.h>
#include <hardware_interface/component_parser.hpp>
#include <joint_trajectory_controller/joint_trajectory_controller.hpp>

#include "franka_hardware/franka_hardware_interface.hpp"
#include "franka_hardware_mocks/franka_hardware_robot_mock.hpp"

class PositionStartupTests : public testing::Test {
 protected:
  std::shared_ptr<testing::NiceMock<MockRobot>> robot =
      std::make_shared<testing::NiceMock<MockRobot>>();
  franka_hardware::FrankaHardwareInterface hardware{robot, "fr3"};
  franka::RobotState state{};
  std::vector<std::string> interfaces;

  void SetUp() override {
    std::ifstream stream(HARDWARE_TEST_URDF);
    std::stringstream xml;
    xml << stream.rdbuf();
    auto info = hardware_interface::parse_control_resources_from_urdf(xml.str());
    ASSERT_EQ(info.size(), 1u);
    ASSERT_EQ(hardware.on_init(info[0]), hardware_interface::CallbackReturn::SUCCESS);
    state.q = {.1, -.2, .3, -1.7, .5, 1.6, .7};
    state.q_d = {.1004, -.2003, .3002, -1.7001, .5003, 1.6002, .7001};
    ON_CALL(*robot, readOnce()).WillByDefault(testing::Return(state));
    for (int i = 1; i <= 7; ++i) {
      interfaces.push_back("fr3_joint"+std::to_string(i)+"/position");
    }
    ASSERT_EQ(hardware.prepare_command_mode_switch(interfaces, {}), hardware_interface::return_type::OK);
  }
};

TEST_F(PositionStartupTests, SeedsDesiredPositionsBeforeControllerCanActivate) {
  testing::InSequence order;
  EXPECT_CALL(*robot, stopRobot());
  EXPECT_CALL(*robot, initializeJointPositionInterface());
  EXPECT_CALL(*robot, readOnce()).WillOnce(testing::Return(state));
  ASSERT_EQ(hardware.perform_command_mode_switch(interfaces, {}), hardware_interface::return_type::OK);
  auto commands = hardware.export_command_interfaces();
  for (size_t i = 0; i < 7; ++i) {
    auto command = std::find_if(commands.begin(), commands.end(), [&](const auto& item) {
      return item.get_name() == interfaces[i];
    });
    ASSERT_NE(command, commands.end());
    EXPECT_DOUBLE_EQ(command->get_optional().value(), state.q_d[i]);
  }
  EXPECT_CALL(*robot, writeOnce(std::vector<double>(state.q_d.begin(), state.q_d.end())));
  EXPECT_EQ(hardware.write(rclcpp::Time(0), rclcpp::Duration(0, 0)), hardware_interface::return_type::OK);
}

TEST_F(PositionStartupTests, FailedInitialReadCannotWriteOrActivate) {
  EXPECT_CALL(*robot, readOnce()).WillOnce(testing::Throw(franka::NetworkException("synthetic startup read failure")));
  EXPECT_CALL(*robot, writeOnce(testing::A<const std::vector<double>&>())).Times(0);
  EXPECT_EQ(hardware.perform_command_mode_switch(interfaces, {}), hardware_interface::return_type::ERROR);
  hardware.write(rclcpp::Time(0), rclcpp::Duration(0, 0));
}

TEST_F(PositionStartupTests, NonFiniteDesiredPositionsCannotActivate) {
  state.q_d[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_CALL(*robot, readOnce()).WillOnce(testing::Return(state));
  EXPECT_CALL(*robot, writeOnce(testing::A<const std::vector<double>&>())).Times(0);
  EXPECT_EQ(hardware.perform_command_mode_switch(interfaces, {}), hardware_interface::return_type::ERROR);
  hardware.write(rclcpp::Time(0), rclcpp::Duration(0, 0));
}

TEST_F(PositionStartupTests, RestartSeedsFreshDesiredPositionsInsteadOfPreviousCommands) {
  ASSERT_EQ(hardware.perform_command_mode_switch(interfaces, {}), hardware_interface::return_type::OK);
  ASSERT_EQ(hardware.prepare_command_mode_switch({}, interfaces), hardware_interface::return_type::OK);
  ASSERT_EQ(hardware.perform_command_mode_switch({}, interfaces), hardware_interface::return_type::OK);
  for (auto& value : state.q_d) {
    value += .002;
  }
  EXPECT_CALL(*robot, readOnce()).WillOnce(testing::Return(state));
  ASSERT_EQ(hardware.prepare_command_mode_switch(interfaces, {}), hardware_interface::return_type::OK);
  ASSERT_EQ(hardware.perform_command_mode_switch(interfaces, {}), hardware_interface::return_type::OK);
  EXPECT_CALL(*robot, writeOnce(std::vector<double>(state.q_d.begin(), state.q_d.end())));
  EXPECT_EQ(hardware.write(rclcpp::Time(0), rclcpp::Duration(0, 0)), hardware_interface::return_type::OK);
}

TEST_F(PositionStartupTests, ActualTrajectoryControllerHoldsDesiredPositionsWhenMeasuredPositionsDiffer) {
  ASSERT_EQ(hardware.perform_command_mode_switch(interfaces, {}), hardware_interface::return_type::OK);
  auto commands = hardware.export_command_interfaces();
  auto states = hardware.export_state_interfaces();
  std::ifstream stream(HARDWARE_TEST_URDF);
  std::stringstream xml;
  xml << stream.rdbuf();
  joint_trajectory_controller::JointTrajectoryController controller;
  auto options = rclcpp::NodeOptions().arguments({"--ros-args", "--params-file", CONTROLLER_PROFILE});
  ASSERT_EQ(controller.init("fr3_arm_controller", xml.str(), 1000, "/offline_startup", options),
            controller_interface::return_type::OK);
  ASSERT_EQ(controller.configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  std::vector<hardware_interface::LoanedCommandInterface> loaned_commands;
  std::vector<hardware_interface::LoanedStateInterface> loaned_states;
  for (auto& command : commands) {
    if (command.get_interface_name() == "position") {
      loaned_commands.emplace_back(command);
    }
  }
  for (auto& state_interface : states) {
    if (state_interface.get_interface_name() == "position" || state_interface.get_interface_name() == "velocity") {
      loaned_states.emplace_back(state_interface);
    }
  }
  controller.assign_interfaces(std::move(loaned_commands), std::move(loaned_states));
  ASSERT_EQ(controller.get_node()->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  EXPECT_CALL(*robot, writeOnce(std::vector<double>(state.q_d.begin(), state.q_d.end()))).Times(20);
  for (int i = 0; i < 20; ++i) {
    auto now = controller.get_node()->now();
    const auto period = rclcpp::Duration::from_seconds(.001);
    hardware.read(now, period);
    ASSERT_EQ(controller.update(now, period), controller_interface::return_type::OK);
    ASSERT_EQ(hardware.write(now, period), hardware_interface::return_type::OK);
  }
  controller.get_node()->deactivate();
  controller.release_interfaces();
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(0, nullptr);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
