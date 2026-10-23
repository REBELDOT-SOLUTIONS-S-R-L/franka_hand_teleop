#include <franka/exception.h>
#include <gmock/gmock.h>
#include <pthread.h>

#include "realtime_tools/realtime_helpers.hpp"

#include "franka_hardware/robot.hpp"
#include "franka_hardware_mocks/franka_model_mock.hpp"
#include "franka_hardware_mocks/franka_robot_mock.hpp"

class StartupPolicyTests : public testing::TestWithParam<int> {};

TEST_P(StartupPolicyTests, DisabledRecoveryPropagatesFaultWithoutRetry) {
  auto device = std::make_unique<MockFrankaRobot>();
  auto model = std::make_unique<MockModel>();
  using Method = void (franka_hardware::Robot::*)();
  Method method = nullptr;
  const auto failure = testing::Throw(franka::ControlException("offline synthetic start fault"));
  switch (GetParam()) {
    case 0:
      EXPECT_CALL(*device, startTorqueControl()).Times(1).WillOnce(failure);
      method = &franka_hardware::Robot::initializeTorqueInterface;
      break;
    case 1:
      EXPECT_CALL(*device, startJointPositionControl(testing::_)).Times(1).WillOnce(failure);
      method = &franka_hardware::Robot::initializeJointPositionInterface;
      break;
    case 2:
      EXPECT_CALL(*device, startJointVelocityControl(testing::_)).Times(1).WillOnce(failure);
      method = &franka_hardware::Robot::initializeJointVelocityInterface;
      break;
    case 3:
      EXPECT_CALL(*device, startCartesianPoseControl(testing::_)).Times(1).WillOnce(failure);
      method = &franka_hardware::Robot::initializeCartesianPoseInterface;
      break;
    case 4:
      EXPECT_CALL(*device, startCartesianVelocityControl(testing::_)).Times(1).WillOnce(failure);
      method = &franka_hardware::Robot::initializeCartesianVelocityInterface;
      break;
  }
  ASSERT_NE(method, nullptr);
  franka_hardware::Robot robot(std::move(device), std::move(model));
  robot.setAutomaticErrorRecoveryOnStart(false);
  EXPECT_THROW((robot.*method)(), franka::ControlException);
}

INSTANTIATE_TEST_SUITE_P(AllControlInterfaces, StartupPolicyTests, testing::Range(0, 5));

TEST(RealtimeCompatibility, NamesCurrentThreadWithLinuxLengthLimit) {
  char original[16] = {};
  ASSERT_EQ(pthread_getname_np(pthread_self(), original, sizeof(original)), 0);
  const auto result = realtime_tools::set_current_thread_name("hand_teleop_offline_long");
  char actual[16] = {};
  const int read_result = pthread_getname_np(pthread_self(), actual, sizeof(actual));
  const int restore_result = pthread_setname_np(pthread_self(), original);
  EXPECT_TRUE(result.first) << result.second;
  EXPECT_EQ(read_result, 0);
  EXPECT_EQ(std::string(actual), std::string("hand_teleop_offline_long").substr(0, 15));
  EXPECT_EQ(restore_result, 0);
}
