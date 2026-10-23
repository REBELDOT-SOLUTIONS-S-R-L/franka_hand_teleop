# Focused mock-only test target; does not require upstream's optional lint suite.
cmake_minimum_required(VERSION 3.16)
project(hand_teleop_hardware_policy_tests)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(Franka REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(franka_hardware REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(franka_hand_controllers REQUIRED)
set(FRANKA_HARDWARE_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/../../../../franka_ros2/franka_hardware"
  CACHE PATH "Source checkout of franka_hardware for mock hardware-policy tests")
get_filename_component(HARDWARE_SOURCE "${FRANKA_HARDWARE_SOURCE}" ABSOLUTE)
if(NOT EXISTS "${HARDWARE_SOURCE}/test/fr3.urdf")
  message(FATAL_ERROR "Set FRANKA_HARDWARE_SOURCE to the patched franka_hardware checkout")
endif()
ament_add_gmock(startup_policy_tests startup_policy_test.cpp
  "${HARDWARE_SOURCE}/src/robot.cpp")
ament_target_dependencies(startup_policy_tests franka_msgs realtime_tools rclcpp rclcpp_action)
target_link_libraries(startup_policy_tests Franka::Franka)
target_include_directories(startup_policy_tests PRIVATE
  "${HARDWARE_SOURCE}/include" "${HARDWARE_SOURCE}/test" "${HARDWARE_SOURCE}/test/include")

ament_add_gmock(position_startup_tests position_startup_test.cpp)
ament_target_dependencies(position_startup_tests franka_hardware franka_msgs rclcpp rclcpp_action joint_trajectory_controller)
target_link_libraries(position_startup_tests Franka::Franka)
target_include_directories(position_startup_tests PRIVATE
  "${HARDWARE_SOURCE}/test" "${HARDWARE_SOURCE}/test/include")
target_compile_definitions(position_startup_tests PRIVATE HARDWARE_TEST_URDF="${HARDWARE_SOURCE}/test/fr3.urdf")
target_compile_definitions(position_startup_tests PRIVATE CONTROLLER_PROFILE="${CMAKE_CURRENT_SOURCE_DIR}/../../config/real_controllers.yaml")

ament_add_gtest(command_continuity_tests command_continuity_test.cpp)
ament_target_dependencies(command_continuity_tests franka_hand_controllers joint_trajectory_controller rclcpp)
target_compile_definitions(command_continuity_tests PRIVATE
  HARDWARE_TEST_URDF="${HARDWARE_SOURCE}/test/fr3.urdf"
  CONTROLLER_PROFILE="${CMAKE_CURRENT_SOURCE_DIR}/../../config/real_controllers.yaml")

ament_add_gtest(rolling_controller_tests rolling_controller_test.cpp)
ament_target_dependencies(rolling_controller_tests franka_hand_controllers joint_trajectory_controller rclcpp)
target_compile_definitions(rolling_controller_tests PRIVATE
  HARDWARE_TEST_URDF="${HARDWARE_SOURCE}/test/fr3.urdf"
  CONTROLLER_PROFILE="${CMAKE_CURRENT_SOURCE_DIR}/../../config/real_controllers.yaml")

target_compile_definitions(rolling_controller_tests PRIVATE
  MOTION_PROFILE="${CMAKE_CURRENT_SOURCE_DIR}/../../test/fixtures/rolling_profile.yaml")

ament_add_gtest(joint_feedback_tests joint_feedback_test.cpp)
ament_target_dependencies(joint_feedback_tests franka_hand_controllers rclcpp)

find_package(rclcpp_components REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
ament_add_gtest(gripper_feedback_tests gripper_feedback_test.cpp
  "${HARDWARE_SOURCE}/../franka_gripper/src/gripper_action_server.cpp")
target_include_directories(gripper_feedback_tests BEFORE PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/fake_gripper" "${HARDWARE_SOURCE}/../franka_gripper/include")
target_compile_definitions(gripper_feedback_tests PRIVATE Gripper=OfflineGripper)
ament_target_dependencies(gripper_feedback_tests franka_msgs rclcpp rclcpp_action
  rclcpp_components control_msgs sensor_msgs std_srvs)
target_link_libraries(gripper_feedback_tests Franka::Franka)

find_package(moveit_core REQUIRED)
ament_add_gtest(path_clearance_tests path_clearance_test.cpp)
ament_target_dependencies(path_clearance_tests franka_hand_controllers moveit_core)
