#include <gtest/gtest.h>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include "franka_hand_controllers/rolling_controller.hpp"
using namespace franka_hand_controllers;
const auto limits=load_motion_profile(MOTION_PROFILE).limits;
class InspectRolling : public GuardedRollingController {
 public:
  void enable(uint64_t e,bool value){controls_.writeFromNonRT(Control{e,value});}
  void send(const trajectory_msgs::msg::JointTrajectory& m){receive(m);}
  uint64_t sequence() const{return sequence_;}
  uint64_t epoch() const{return epoch_;}
  bool enabled() const{return enabled_;}
  double configured_tool_speed() const{return limits_.tool_speed;}
  unsigned rejection_reason() const{return static_cast<unsigned>(rejection_detail_[3]);}
  double rejection_lead() const{return rejection_detail_[9];}
};
struct Harness {
  InspectRolling c;
  Joints q{.1,-.2,.3,-1.7,.5,1.6,.7},actual=q,velocity{};
  double robot_time{0.};
  rclcpp::Time host{100,0,RCL_ROS_TIME};
  std::vector<hardware_interface::CommandInterface> commands;
  std::vector<hardware_interface::StateInterface> states;
  Harness(const std::string& profile=MOTION_PROFILE) {
    std::ifstream f(HARDWARE_TEST_URDF);std::stringstream xml;xml<<f.rdbuf();
    auto options=rclcpp::NodeOptions().arguments({"--ros-args","--params-file",CONTROLLER_PROFILE})
      .parameter_overrides({rclcpp::Parameter("motion_profile",profile)});
    EXPECT_EQ(c.init("fr3_arm_controller",xml.str(),1000,"/rolling_test",options),controller_interface::return_type::OK);
    EXPECT_EQ(c.configure().id(),lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    for(size_t i=0;i<7;++i){auto name="fr3_joint"+std::to_string(i+1);commands.emplace_back(name,"position",&q[i]);states.emplace_back(name,"position",&actual[i]);states.emplace_back(name,"velocity",&velocity[i]);}
    states.emplace_back("fr3","robot_time",&robot_time);
    std::vector<hardware_interface::LoanedCommandInterface> cmd;
    std::vector<hardware_interface::LoanedStateInterface> st;
    for(auto& x:commands)cmd.emplace_back(x);for(auto& x:states)st.emplace_back(x);
    c.assign_interfaces(std::move(cmd),std::move(st));
    EXPECT_EQ(c.get_node()->activate().id(),lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    step();c.enable(10,true);step();EXPECT_TRUE(c.enabled());
  }
  ~Harness(){c.get_node()->deactivate();c.release_interfaces();}
  void step(bool jitter=false,double bias=.0002,double lag=0.) {
    robot_time+=.001;host=host+rclcpp::Duration::from_seconds(jitter?.005:.001);
    EXPECT_EQ(c.update(host,rclcpp::Duration::from_seconds(jitter?.005:.001)),controller_interface::return_type::OK);
    for(size_t i=0;i<7;++i)actual[i]+=(lag>0.?1.-std::exp(-.001/lag):1.)*(q[i]-bias-actual[i]);
  }
  void send(const RollingCurve& curve,uint64_t seq,uint64_t epoch=10,uint64_t parent=0) {
    trajectory_msgs::msg::JointTrajectory m;
    m.header.frame_id="rolling1/"+std::to_string(epoch)+"/"+std::to_string(seq)+"/"+std::to_string(parent);
    m.header.stamp=rclcpp::Time(static_cast<int64_t>(std::llround(curve.start*1e9)),RCL_ROS_TIME);
    for(size_t i=0;i<7;++i)m.joint_names.push_back("fr3_joint"+std::to_string(i+1));
    trajectory_msgs::msg::JointTrajectoryPoint a,b;
    a.positions.assign(curve.initial.q.begin(),curve.initial.q.end());a.velocities.assign(curve.initial.v.begin(),curve.initial.v.end());a.accelerations.assign(curve.initial.a.begin(),curve.initial.a.end());
    b.positions.assign(curve.target.begin(),curve.target.end());b.velocities=b.accelerations=std::vector<double>(7,0.);b.time_from_start=rclcpp::Duration::from_seconds(curve.duration);
    m.points={a,b};c.send(m);
  }
};
struct ProfileFile {
  std::string path;
  ProfileFile(double speed) {
    char pattern[]="/tmp/rolling-motion-XXXXXX";
    const int fd=mkstemp(pattern);
    if(fd<0) throw std::runtime_error("Could not create mock profile");
    close(fd);path=pattern;
    auto profile=YAML::LoadFile(MOTION_PROFILE);
    profile["motion"]["max_speed"]=speed;
    std::ofstream out(path);out<<profile;
  }
  ~ProfileFile(){std::remove(path.c_str());}
};
TEST(RollingConfiguration, ThreeMetresPerSecondLoadsInActualControllerWithoutRecompile) {
  ProfileFile profile(3.);
  const auto configured=load_motion_profile(profile.path).limits;
  EXPECT_DOUBLE_EQ(configured.tool_speed,3.);
  Harness h(profile.path);
  EXPECT_DOUBLE_EQ(h.c.configured_tool_speed(),3.);
  EXPECT_TRUE(h.c.enabled());
  // Same geometry and derivatives, only the profile speed differs.
  auto permissive=configured;permissive.tool_travel=.7;
  RollingCurve curve;curve.initial.q=h.q;curve.target=h.q;
  curve.target[0]+=.35;curve.duration=.73;curve.start=h.robot_time+.15;curve.build();
  ASSERT_TRUE(curve.valid(1.722834753163608,permissive));
  auto previous=permissive;previous.tool_speed=1.44;
  EXPECT_FALSE(curve.valid(1.722834753163608,previous));
}
TEST(RollingConfiguration, MissingInvalidAndExcessiveSettingsFailClosed) {
  EXPECT_THROW(load_motion_profile(""),std::invalid_argument);
  ProfileFile excessive(3.001);
  EXPECT_THROW(load_motion_profile(excessive.path),std::invalid_argument);
  ProfileFile invalid(0.);
  EXPECT_THROW(load_motion_profile(invalid.path),std::invalid_argument);
  auto missing=YAML::LoadFile(MOTION_PROFILE);missing["motion"].remove("joint_jerk");
  {std::ofstream out(invalid.path);out<<missing;}
  EXPECT_THROW(load_motion_profile(invalid.path),YAML::Exception);
  RollingCurve hold;hold.duration=.2;hold.build();
  EXPECT_FALSE(hold.valid(1.722834753163608,RollingLimits{}));
}
TEST(RollingController, RepeatedHandoffsStayMovingAndPreserveAllCommandDerivatives) {
  Harness h;RollingCurve previous;bool have=false;uint64_t sequence=0;
  Joints old=h.q,old_v{},old_a{};double peak_v=0.,peak_a=0.,peak_j=0.,minimum_moving_speed=1.;
  for(int tick=0;tick<6000;++tick) {
    if(tick%250==0 && tick<4500) {
      RollingCurve curve;curve.start=h.robot_time+.15;curve.duration=1.2;
      curve.initial=have?previous.at(curve.start):RollingState{h.q,{},{}};
      curve.target=curve.initial.q;curve.target[0]=.1+.004*h.robot_time;
      curve.build();ASSERT_TRUE(curve.valid(1.722834753163608,limits));
      ++sequence; h.send(curve,sequence,10,sequence-1);previous=curve;have=true;
    }
    h.step(tick%337==0,tick%2?.0002:-.0002,.04);
    for(size_t i=0;i<7;++i) {
      const double v=(h.q[i]-old[i])/.001,a=(v-old_v[i])/.001,j=(a-old_a[i])/.001;
      peak_v=std::max(peak_v,std::abs(v));peak_a=std::max(peak_a,std::abs(a));peak_j=std::max(peak_j,std::abs(j));
      if(i==0&&tick>1500&&tick<4000)minimum_moving_speed=std::min(minimum_moving_speed,std::abs(v));
      old[i]=h.q[i];old_v[i]=v;old_a[i]=a;
    }
  }
  EXPECT_EQ(h.c.sequence(),sequence);EXPECT_GT(minimum_moving_speed,.002);
  EXPECT_LT(peak_v,.1);EXPECT_LT(peak_a,.3);EXPECT_LT(peak_j,1.);
  EXPECT_NEAR(h.q[0],previous.target[0],1e-12);
  std::cout<<"ROLLING sampled v/a/j="<<peak_v<<","<<peak_a<<","<<peak_j<<"; minimum moving speed="<<minimum_moving_speed<<"; accepted="<<h.c.sequence()<<"\n";
}
TEST(RollingController, ShorterVariableHorizonsPreserveContinuityAcrossReversalAndJitter) {
  Harness h;
  RollingCurve previous;
  bool have=false, used_short=false;
  uint64_t sequence=0;
  Joints old=h.q,old_v{},old_a{};
  double peak_v=0.,peak_a=0.,peak_j=0.;
  for(int tick=0;tick<6500;++tick) {
    if(tick%70==0 && tick<4500) {
      RollingCurve curve;
      curve.start=h.robot_time+.05;
      curve.initial=have?previous.at(curve.start):RollingState{h.q,{},{}};
      curve.target=curve.initial.q;
      curve.target[0]=.1+.012*(h.robot_time<2.5?h.robot_time:5.-h.robot_time);
      bool valid=false;
      for(double duration:{.1,.2,.3,.4,.5,.6,.9,1.2,1.4}) {
        curve.duration=duration;curve.build();
        if(curve.valid(1.722834753163608,limits)){valid=true;break;}
      }
      ASSERT_TRUE(valid);
      used_short=used_short || curve.duration<1.2;
      ++sequence;h.send(curve,sequence,10,sequence-1);
      previous=curve;have=true;
    }
    h.step(tick%337==0,tick%2?.0002:-.0002,.04);
    for(size_t i=0;i<7;++i) {
      const double v=(h.q[i]-old[i])/.001,a=(v-old_v[i])/.001,j=(a-old_a[i])/.001;
      peak_v=std::max(peak_v,std::abs(v));peak_a=std::max(peak_a,std::abs(a));
      peak_j=std::max(peak_j,std::abs(j));
      old[i]=h.q[i];old_v[i]=v;old_a[i]=a;
    }
  }
  EXPECT_TRUE(used_short);EXPECT_EQ(h.c.sequence(),sequence);
  EXPECT_LT(peak_v,limits.joint_speed);EXPECT_LT(peak_a,limits.joint_acceleration);
  EXPECT_LT(peak_j,limits.joint_jerk);
  for(size_t i=0;i<7;++i)EXPECT_NEAR(h.q[i],previous.target[i],1e-12);
  std::cout<<"SHORT HORIZONS sampled v/a/j="<<peak_v<<","<<peak_a<<","<<peak_j
           <<"; accepted="<<h.c.sequence()<<"\n";
}
TEST(RollingController, HundredMillimetreCurveRetainsBoundsAndContinuityAtHigherCaps) {
  Harness h;
  RollingCurve curve; curve.initial.q=h.q; curve.target=h.q;
  const Joints pattern{1.,-.5,.25,-.5,.25,-.5,.25};
  const double scale=.1/(1.722834753163608*3.25);
  for(size_t i=0;i<7;++i)curve.target[i]+=scale*pattern[i];
  curve.start=h.robot_time+.15;curve.duration=1.4;curve.build();
  ASSERT_TRUE(curve.valid(1.722834753163608,limits));
  h.send(curve,1);h.step();ASSERT_EQ(h.c.sequence(),1u);
  Joints previous=h.q,previous_v{},previous_a{};
  double peak_v=0.,peak_a=0.,peak_j=0.;
  for(int tick=0;tick<1800;++tick) {
    h.step(tick%337==0,.0002,.04);
    for(size_t i=0;i<7;++i) {
      const double v=(h.q[i]-previous[i])/.001;
      const double a=(v-previous_v[i])/.001;
      const double j=(a-previous_a[i])/.001;
      peak_v=std::max(peak_v,std::abs(v));peak_a=std::max(peak_a,std::abs(a));peak_j=std::max(peak_j,std::abs(j));
      previous[i]=h.q[i];previous_v[i]=v;previous_a[i]=a;
    }
  }
  EXPECT_LT(peak_v,limits.joint_speed);EXPECT_LT(peak_a,limits.joint_acceleration);EXPECT_LT(peak_j,limits.joint_jerk);
  for(size_t i=0;i<7;++i)EXPECT_NEAR(h.q[i],curve.target[i],1e-12);
  auto oversized=curve;
  for(size_t i=0;i<7;++i)oversized.target[i]=curve.initial.q[i]+2.*scale*pattern[i];
  oversized.build();EXPECT_FALSE(oversized.valid(1.722834753163608,limits));
  std::cout<<"100 MM sampled v/a/j="<<peak_v<<","<<peak_a<<","<<peak_j<<"\n";
}
TEST(RollingController, TripleSpeedCurveRetainsSampledDerivativeAndCollisionSamplingBounds) {
  Harness h;
  RollingCurve curve;
  curve.initial.q=h.q;curve.target=h.q;curve.target[0]+=.01;
  curve.start=h.robot_time+.05;curve.duration=.3;curve.build();
  ASSERT_TRUE(curve.valid(1.722834753163608,limits));
  const double speed_bound=curve.speed_bound(1.722834753163608);
  EXPECT_LE(speed_bound,limits.tool_speed);
  h.send(curve,1);
  Joints old=h.q,old_v{},old_a{};
  double peak_v=0.,peak_a=0.,peak_j=0.;
  for(int tick=0;tick<900;++tick) {
    h.step(tick%137==0,.0002,.04);
    const auto state=curve.at(h.robot_time);
    double speed=0.;
    for(size_t i=0;i<7;++i) {
      speed+=std::abs(state.v[i]);
      const double v=(h.q[i]-old[i])/.001,a=(v-old_v[i])/.001,j=(a-old_a[i])/.001;
      peak_v=std::max(peak_v,std::abs(v));peak_a=std::max(peak_a,std::abs(a));
      peak_j=std::max(peak_j,std::abs(j));
      old[i]=h.q[i];old_v[i]=v;old_a[i]=a;
    }
    EXPECT_LE(1.722834753163608*speed,speed_bound+1e-12);
  }
  EXPECT_EQ(h.c.sequence(),1u);
  EXPECT_GT(peak_v,.06);EXPECT_GT(peak_j,1.);
  EXPECT_LT(peak_v,limits.joint_speed);EXPECT_LT(peak_a,limits.joint_acceleration);
  EXPECT_LT(peak_j,limits.joint_jerk);
  EXPECT_NEAR(h.q[0],curve.target[0],1e-12);
  auto stationary=curve;stationary.target=stationary.initial.q;stationary.build();
  EXPECT_NEAR(stationary.speed_bound(1.722834753163608),0.,1e-12);
  std::cout<<"TRIPLE SPEED sampled v/a/j="<<peak_v<<","<<peak_a<<","<<peak_j<<"\n";
}

TEST(RollingController, FiftyPercentFasterCurveKeepsDerivativeBounds) {
  Harness h;
  RollingCurve curve;
  curve.initial.q=h.q;curve.target=h.q;curve.target[0]+=.01;
  curve.start=h.robot_time+.15;curve.duration=.2;curve.build();
  ASSERT_TRUE(curve.valid(1.722834753163608,limits));
  EXPECT_DOUBLE_EQ(limits.joint_speed,1.5);
  EXPECT_DOUBLE_EQ(limits.joint_acceleration,4.5);
  EXPECT_DOUBLE_EQ(limits.joint_jerk,91.125);
  h.send(curve,1);h.step();ASSERT_EQ(h.c.sequence(),1u);
  for(int tick=0;tick<500;++tick)h.step();
  EXPECT_NEAR(h.q[0],curve.target[0],1e-12);
  auto too_fast=curve;too_fast.duration=.1;too_fast.build();
  EXPECT_FALSE(too_fast.valid(1.722834753163608,limits));
}

TEST(RollingController, LateMismatchedReorderedAndOldSessionPacketsCannotExtendMotion) {
  Harness h;RollingCurve curve;curve.initial.q=h.q;curve.target=h.q;curve.target[0]+=.005;curve.start=h.robot_time+.15;curve.duration=1.2;curve.build();
  h.send(curve,1);h.step();ASSERT_EQ(h.c.sequence(),1u);
  while(h.robot_time<.30)h.step();
  auto bad=curve;bad.start=h.robot_time-.01;h.send(bad,2,10,1);h.step();EXPECT_EQ(h.c.sequence(),1u);
  EXPECT_NE(h.c.rejection_reason()&32u,0u);EXPECT_LT(h.c.rejection_lead(),.020);
  bad=curve;bad.start=h.robot_time+.15;h.send(bad,2,10,1);h.step();EXPECT_EQ(h.c.sequence(),1u);
  EXPECT_NE(h.c.rejection_reason()&128u,0u);EXPECT_EQ(h.c.rejection_reason()&32u,0u);
  bad.initial=curve.at(bad.start);bad.build();h.send(bad,3,10,1);h.step();EXPECT_EQ(h.c.sequence(),1u);
  EXPECT_NE(h.c.rejection_reason()&16u,0u);
  h.c.enable(11,false);h.step();EXPECT_FALSE(h.c.enabled());
  h.send(bad,2,10,1);h.step();EXPECT_EQ(h.c.sequence(),0u);
  EXPECT_NE(h.c.rejection_reason()&1u,0u);EXPECT_NE(h.c.rejection_reason()&4u,0u);
  h.c.enable(12,true);h.step();EXPECT_FALSE(h.c.enabled());
  while(h.robot_time<1.5)h.step();EXPECT_NEAR(h.q[0],curve.target[0],1e-12);
  h.c.enable(13,true);h.step();EXPECT_TRUE(h.c.enabled());
  h.send(bad,2,10,1);h.step();EXPECT_EQ(h.c.sequence(),0u);
}
TEST(RollingController, DisarmingBeforeHandoffDiscardsPendingExtension) {
  Harness h;auto hold=h.q;RollingCurve curve;curve.initial.q=h.q;curve.target=h.q;curve.target[0]+=.005;curve.start=h.robot_time+.15;curve.duration=1.2;curve.build();
  h.send(curve,1);h.step();ASSERT_EQ(h.c.sequence(),1u);
  h.c.enable(11,false);for(int i=0;i<1600;++i)h.step();
  EXPECT_EQ(h.q,hold);EXPECT_FALSE(h.c.enabled());
}
TEST(RollingController, InvalidRobotClockDoesNotWriteNewPosition) {
  Harness h;auto previous=h.q;h.robot_time-=.001;
  EXPECT_EQ(h.c.update(h.host,rclcpp::Duration::from_seconds(.001)),controller_interface::return_type::ERROR);EXPECT_EQ(h.q,previous);
}
int main(int argc,char** argv){testing::InitGoogleTest(&argc,argv);rclcpp::init(0,nullptr);const int result=RUN_ALL_TESTS();rclcpp::shutdown();return result;}
