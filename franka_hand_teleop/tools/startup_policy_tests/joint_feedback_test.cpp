#include <gtest/gtest.h>
#include "franka_hand_controllers/joint_feedback.hpp"

using franka_hand_controllers::JointFeedback;
namespace {
const std::vector<std::string> arm_names{"fr3_joint1","fr3_joint2","fr3_joint3","fr3_joint4","fr3_joint5","fr3_joint6","fr3_joint7"};
sensor_msgs::msg::JointState sample(const std::vector<std::string>& names, double time, double q=0.) {
  sensor_msgs::msg::JointState msg;
  msg.name=names; msg.position.assign(names.size(),q);
  const auto ns=static_cast<int64_t>(std::llround(time*1e9));
  msg.header.stamp.sec=ns/1000000000; msg.header.stamp.nanosec=ns%1000000000;
  return msg;
}
sensor_msgs::msg::JointState hand(double time) {
  return sample({"fr3_finger_joint1","fr3_finger_joint2"},time,.02);
}
}
TEST(JointFeedback, CompleteSnapshotsRetainTheOldestActualSourceStamp) {
  JointFeedback feedback(arm_names);
  feedback.fingers(hand(10.),10.);
  for (int tick=1; tick<=125; ++tick) {
    const double now=10.+tick*.001;
    feedback.arm(sample(arm_names,now,tick*.00001),now);
    ASSERT_TRUE(feedback.reason(now).empty());
    const auto combined=feedback.snapshot();
    ASSERT_EQ(combined.name.size(),9u);
    ASSERT_EQ(combined.position.size(),9u);
    EXPECT_EQ(combined.header.stamp,hand(10.).header.stamp);
    EXPECT_DOUBLE_EQ(combined.position[7],.02);
    EXPECT_DOUBLE_EQ(combined.position[0],tick*.00001);
  }
}
TEST(JointFeedback, ArmTrafficAndRepeatedFingerMessagesCannotRefreshExpiredHandData) {
  JointFeedback feedback(arm_names);
  feedback.fingers(hand(10.),10.);
  for (int tick=1; tick<=1000; ++tick) {
    const double now=10.+tick*.001;
    feedback.arm(sample(arm_names,now),now);
    feedback.fingers(hand(10.),now);
    if (tick>=151) EXPECT_NE(feedback.reason(now).find("Gripper"),std::string::npos);
  }
}
TEST(JointFeedback, FreshGripperDataCannotHideArmLossAtTheFastDeadline) {
  JointFeedback feedback(arm_names);
  feedback.arm(sample(arm_names,10.),10.);
  feedback.fingers(hand(10.101),10.101);
  EXPECT_NE(feedback.reason(10.101).find("Arm"),std::string::npos);
}
TEST(JointFeedback, MalformedAndFutureInputsFailClosedWithoutDefaultFingerPositions) {
  for (int scenario=0; scenario<5; ++scenario) {
    JointFeedback feedback(arm_names);
    feedback.arm(sample(arm_names,10.),10.);
    auto msg=hand(10.);
    if(scenario==0) msg.position.pop_back();
    if(scenario==1) msg.name[1]=msg.name[0];
    if(scenario==2) msg.position[0]=NAN;
    if(scenario==3) msg.position[0]=.041;
    if(scenario==4) msg=hand(10.1);
    feedback.fingers(msg,10.);
    EXPECT_NE(feedback.reason(10.).find("Gripper"),std::string::npos);
  }
}
TEST(JointFeedback, LateMessagesDoNotReplaceNewerJointValues) {
  JointFeedback feedback(arm_names);
  feedback.arm(sample(arm_names,10.02,.1),10.02);
  feedback.arm(sample(arm_names,10.01,.9),10.02);
  feedback.fingers(hand(10.02),10.02);
  EXPECT_TRUE(feedback.reason(10.02).empty());
  EXPECT_DOUBLE_EQ(feedback.snapshot().position[0],.1);
}
