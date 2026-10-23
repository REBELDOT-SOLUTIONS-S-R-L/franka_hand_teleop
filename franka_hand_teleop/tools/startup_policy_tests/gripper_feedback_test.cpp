#include <gtest/gtest.h>
#include <franka_gripper/gripper_action_server.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

class GripperFeedbackTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { if(!rclcpp::ok()) rclcpp::init(0,nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override {
    franka::offline::fail_reads=false;
    franka::offline::motion_calls=0;
    auto options=rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("robot_ip","offline-fixture"),
      rclcpp::Parameter("joint_names",std::vector<std::string>{"fr3_finger_joint1","fr3_finger_joint2"}),
      rclcpp::Parameter("state_publish_rate",100)});
    driver=std::make_shared<franka_gripper::GripperActionServer>(options);
    observer=std::make_shared<rclcpp::Node>("offline_gripper_observer");
    subscription=observer->create_subscription<sensor_msgs::msg::JointState>(
      "/franka_gripper_node/joint_states",10,
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg){++messages;last=*msg;});
    executor.add_node(driver);executor.add_node(observer);
  }
  void spin(double duration) {
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::duration<double>(duration);
    while(std::chrono::steady_clock::now()<deadline) executor.spin_once(std::chrono::milliseconds(2));
  }
  void TearDown() override {
    EXPECT_EQ(franka::offline::motion_calls,0u);
    executor.remove_node(driver);executor.remove_node(observer);
    subscription.reset();driver.reset();observer.reset();
  }
  rclcpp::executors::SingleThreadedExecutor executor;
  std::shared_ptr<franka_gripper::GripperActionServer> driver;
  rclcpp::Node::SharedPtr observer;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription;
  unsigned messages{0};
  sensor_msgs::msg::JointState last;
};
TEST_F(GripperFeedbackTest, SuccessfulReadsPublishMeasuredWidthWithoutMotion) {
  spin(.15);
  ASSERT_GT(messages,0u);
  ASSERT_EQ(last.position.size(),2u);
  EXPECT_DOUBLE_EQ(last.position[0],.02);
  EXPECT_DOUBLE_EQ(last.position[1],.02);
  EXPECT_GT(rclcpp::Time(last.header.stamp).nanoseconds(),0);
}
TEST_F(GripperFeedbackTest, FailedReadsNeverPublishTheOldWidthWithANewTimestamp) {
  franka::offline::fail_reads=true;
  spin(.15);
  EXPECT_EQ(messages,0u);
}
