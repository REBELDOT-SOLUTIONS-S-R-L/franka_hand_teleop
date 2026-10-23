#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <type_traits>

#include <gtest/gtest.h>
#include <franka_hand_controllers/robot_clock_controller.hpp>
#include <joint_trajectory_controller/joint_trajectory_controller.hpp>

template <typename Base>
class InspectableTrajectoryController : public Base {
 public:
  using Base::add_new_trajectory_msg;
  void requestStop() { this->add_new_trajectory_msg(this->decelerate_to_hold_position()); }
};

struct Peaks {
  double acceleration{0.};
  double jerk{0.};
};

template <typename Base>
Peaks sampleCommands(int segments = 1, bool jitter = true, int fault = 0,
                     int cancel_tick = -1, double measured_bias = 0.,
                     double displacement = .0012, double segment_duration = .6,
                     bool single_joint = false) {
  InspectableTrajectoryController<Base> controller;
  std::ifstream stream(HARDWARE_TEST_URDF);
  std::stringstream xml;
  xml << stream.rdbuf();
  auto options =
      rclcpp::NodeOptions().arguments({"--ros-args", "--params-file", CONTROLLER_PROFILE});
  EXPECT_EQ(controller.init("fr3_arm_controller", xml.str(), 1000, "/continuity_check", options),
            controller_interface::return_type::OK);
  EXPECT_EQ(controller.configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  std::array<double, 7> command{.1, -.2, .3, -1.7, .5, 1.6, .7};
  auto actual = command;
  for (auto& q : actual) q -= measured_bias;
  std::array<double, 7> velocity{};
  if (measured_bias != 0.) velocity.fill(.001);  // encoder noise must not become hold-command velocity
  double robot_time = 0.;
  std::vector<hardware_interface::CommandInterface> commands;
  std::vector<hardware_interface::StateInterface> states;
  for (size_t i = 0; i < 7; ++i) {
    auto name = "fr3_joint" + std::to_string(i + 1);
    commands.emplace_back(name, "position", &command[i]);
    states.emplace_back(name, "position", &actual[i]);
    states.emplace_back(name, "velocity", &velocity[i]);
  }
  if (fault != 6)
    states.emplace_back("fr3", "robot_time", &robot_time);
  std::vector<hardware_interface::LoanedCommandInterface> loaned_commands;
  std::vector<hardware_interface::LoanedStateInterface> loaned_states;
  for (auto& item : commands)
    loaned_commands.emplace_back(item);
  for (auto& item : states)
    loaned_states.emplace_back(item);
  controller.assign_interfaces(std::move(loaned_commands), std::move(loaned_states));
  if (fault == 6) {
    EXPECT_EQ(controller.on_activate(rclcpp_lifecycle::State()),
              controller_interface::CallbackReturn::ERROR);
    controller.release_interfaces();
    return {};
  }
  EXPECT_EQ(controller.get_node()->activate().id(),
            lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  auto stamp = rclcpp::Time(100, 0, RCL_ROS_TIME);
  const auto nominal = rclcpp::Duration::from_seconds(.001);
  controller.update(stamp, nominal);
  auto previous_position = command;
  std::array<double, 7> previous_velocity{}, previous_acceleration{};
  Peaks peaks;
  for (int segment = 0; segment < segments; ++segment) {
    auto trajectory = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
    for (size_t i = 0; i < 7; ++i)
      trajectory->joint_names.push_back("fr3_joint" + std::to_string(i + 1));
    trajectory_msgs::msg::JointTrajectoryPoint start, end;
    start.positions.assign(command.begin(), command.end());
    start.velocities = start.accelerations = std::vector<double>(7, 0.);
    end = start;
    end.positions[0] += segment % 2 ? -displacement : displacement;
    if (cancel_tick >= 0 && !single_joint)
      for (size_t i = 1; i < 7; ++i)
        end.positions[i] += displacement*(i % 2 ? -.5 : .25);
    end.time_from_start = rclcpp::Duration::from_seconds(segment_duration);
    trajectory->points = {start, end};
    if (fault == 5)
      trajectory->header.stamp.sec = 101;
    controller.add_new_trajectory_msg(trajectory);
    for (int tick = 0; tick < static_cast<int>(std::ceil(segment_duration*1000))+200; ++tick) {
      // Every command is delivered. A clock-period jump alone can fast-forward
      // JTC's interpolator even when the robot reports 100% packet delivery.
      const auto measured = rclcpp::Duration::from_seconds(jitter && tick == 300 ? .005 : .001);
      stamp = stamp + measured;
      const auto previous_command = command;
      if (tick == cancel_tick)
        controller.requestStop();
      robot_time += .001;
      if (tick == 300) {
        if (fault == 1)
          robot_time -= .001;
        if (fault == 2)
          robot_time -= .002;
        if (fault == 3)
          robot_time += .020;
        if (fault == 4)
          robot_time = std::numeric_limits<double>::quiet_NaN();
      }
      if ((fault >= 1 && fault <= 4 && tick == 300) || (fault == 5 && tick == 0)) {
        EXPECT_EQ(controller.update(stamp, measured), controller_interface::return_type::ERROR);
        EXPECT_EQ(command, previous_command);
        break;
      }
      EXPECT_EQ(controller.update(stamp, measured), controller_interface::return_type::OK);
      actual = command;
      for (auto& q : actual) q -= measured_bias;
      for (size_t i = 0; i < 7; ++i) {
        const double speed = (command[i]-previous_position[i])/.001;
        const double acceleration = (speed-previous_velocity[i])/.001;
        const double jerk = (acceleration-previous_acceleration[i])/.001;
        peaks.acceleration = std::max(peaks.acceleration, std::abs(acceleration));
        peaks.jerk = std::max(peaks.jerk, std::abs(jerk));
        velocity[i] = speed;
        previous_position[i] = command[i];
        previous_velocity[i] = speed;
        previous_acceleration[i] = acceleration;
        if constexpr (std::is_same_v<Base, franka_hand_controllers::RobotClockTrajectoryController>) {
          if (cancel_tick >= 0) {
            const double fraction = (command[0]-start.positions[0])/(end.positions[0]-start.positions[0]);
            EXPECT_NEAR(command[i], start.positions[i]+fraction*(end.positions[i]-start.positions[i]), 1e-10);
          }
        }
      }
    }
    if (!fault && cancel_tick < 0)
      EXPECT_NEAR(command[0], end.positions[0], 1e-12);
    if (!fault && cancel_tick >= 0) {
      EXPECT_LE(command[0], end.positions[0]+1e-12);
      EXPECT_NEAR(previous_velocity[0], 0., 1e-9);
      if constexpr (std::is_same_v<Base, franka_hand_controllers::RobotClockTrajectoryController>) {
        EXPECT_GE(command[0], start.positions[0]-1e-12);
        if (cancel_tick == 100)
          EXPECT_LT(command[0], end.positions[0]-.0001);
      }
    }
  }
  controller.get_node()->deactivate();
  controller.release_interfaces();
  return peaks;
}

TEST(CommandContinuity, PeriodJumpCanViolateFrankaLimitsDespiteSmoothWaypoints) {
  const auto peaks = sampleCommands<joint_trajectory_controller::JointTrajectoryController>();
  std::cout << "BASELINE: peak sampled acceleration=" << peaks.acceleration
            << " rad/s^2, jerk=" << peaks.jerk << " rad/s^3\n";
  EXPECT_GT(peaks.acceleration, 10.);
  EXPECT_GT(peaks.jerk, 5000.);
}

TEST(CommandContinuity, RobotClockPreservesDerivativesAcrossTwentySegmentsAndHostPeriodJumps) {
  const auto peaks = sampleCommands<franka_hand_controllers::RobotClockTrajectoryController>(20);
  std::cout << "ROBOT CLOCK: peak sampled acceleration=" << peaks.acceleration
            << " rad/s^2, jerk=" << peaks.jerk << " rad/s^3\n";
  EXPECT_LT(peaks.acceleration, .3);
  EXPECT_LT(peaks.jerk, 1.);
}

TEST(CommandContinuity, BaselineIdealClockAlsoObeysCommissioningLimits) {
  const auto peaks =
      sampleCommands<joint_trajectory_controller::JointTrajectoryController>(1, false);
  EXPECT_LT(peaks.acceleration, .3);
  EXPECT_LT(peaks.jerk, 1.);
}

TEST(CommandContinuity, InvalidRobotClockOrAbsoluteTimestampDoesNotProduceNewCommands) {
  for (int fault = 1; fault <= 6; ++fault) {
    SCOPED_TRACE(fault);
    sampleCommands<franka_hand_controllers::RobotClockTrajectoryController>(1, true, fault);
  }
}

TEST(CommandContinuity, CancelPreservesCommandDerivativesDespiteMeasuredDesiredBias) {
  for (const int tick : {5, 30, 100, 200, 300, 420, 500, 580, 599}) {
    for (const double bias : {-.0002, 0., .0002}) {
      SCOPED_TRACE(tick);
      SCOPED_TRACE(bias);
      const auto peaks = sampleCommands<franka_hand_controllers::RobotClockTrajectoryController>(
        1, true, 0, tick, bias);
      EXPECT_LT(peaks.acceleration, .3);
      EXPECT_LT(peaks.jerk, 1.);
    }
  }
}

TEST(CommandContinuity, StockCancelReproducesDiscontinuityWithMeasuredDesiredBias) {
  const auto peaks = sampleCommands<joint_trajectory_controller::JointTrajectoryController>(
    1, false, 0, 200, .0002);
  EXPECT_GT(peaks.acceleration, 10.);
  EXPECT_GT(peaks.jerk, 1000.);
}

TEST(CommandContinuity, FiftyMillimetrePathPreservesCancelDerivatives) {
  // Seven-joint pattern sums to 3.25*displacement. At a 2 m reach bound,
  // 0.05/6.5 rad produces exactly 50 mm conservative travel. Match production
  // tool speed/acceleration retiming while retaining joint derivative limits.
  const double displacement = .05/6.5;
  const double sum = 3.25*displacement;
  const double length = 2.*sum;
  const double total = std::max(1.875*length/.08,
    std::sqrt(((10./std::sqrt(3.))*length+1.875*1.875*length*sum)/.20));
  ASSERT_LE(total, 1.5);
  for (const double phase : {.02, .10, .25, .50, .75, .98}) {
    for (const double bias : {-.0002, .0002}) {
      SCOPED_TRACE(phase);
      SCOPED_TRACE(bias);
      const auto peaks = sampleCommands<franka_hand_controllers::RobotClockTrajectoryController>(
        1, true, 0, static_cast<int>(phase*total*1000), bias, displacement, total);
      EXPECT_LT(peaks.acceleration, .3);
      EXPECT_LT(peaks.jerk, 1.);
    }
  }
}

TEST(CommandContinuity, LargerSingleJointStepPreservesCancelDerivatives) {
  const double displacement = .03;
  const double reach = 1.;  // Supported minimum reach; exercises full joint-step ceiling.
  const double length = reach*displacement;
  const double total = std::max({.20, 1.875*displacement/.1,
    std::sqrt((10./std::sqrt(3.))*displacement/.3),
    std::cbrt(60.*displacement), 1.875*length/.08,
    std::sqrt(((10./std::sqrt(3.))*length+1.875*1.875*length*displacement)/.20)});
  ASSERT_LE(total, 1.5);
  ASSERT_LE(length, .05);
  for (const double phase : {.02, .10, .25, .50, .75, .98}) {
    for (const double bias : {-.0002, .0002}) {
      SCOPED_TRACE(phase);
      SCOPED_TRACE(bias);
      const auto peaks = sampleCommands<franka_hand_controllers::RobotClockTrajectoryController>(
        1, true, 0, static_cast<int>(phase*total*1000), bias, displacement, total, true);
      EXPECT_LT(peaks.acceleration, .3);
      EXPECT_LT(peaks.jerk, 1.);
    }
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(0, nullptr);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
