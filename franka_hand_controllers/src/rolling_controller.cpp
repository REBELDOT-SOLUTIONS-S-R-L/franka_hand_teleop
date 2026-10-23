#include "franka_hand_controllers/rolling_controller.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace franka_hand_controllers {
controller_interface::CallbackReturn GuardedRollingController::on_configure(const rclcpp_lifecycle::State& state) {
  const auto result=RobotClockTrajectoryController::on_configure(state);
  if (result!=controller_interface::CallbackReturn::SUCCESS || dof_!=7) return controller_interface::CallbackReturn::ERROR;
  try {
    const auto path=get_node()->declare_parameter<std::string>("motion_profile", "");
    limits_=load_motion_profile(path).limits;
    RCLCPP_INFO(get_node()->get_logger(), "Motion profile: %s; tool speed=%.3f m/s acceleration=%.3f m/s2",path.c_str(),limits_.tool_speed,limits_.tool_acceleration);
  } catch(const std::exception& error) {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid motion profile: %s", error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  reach_=get_node()->declare_parameter<double>("rolling_reach",1.722834753163608);
  if (!std::isfinite(reach_) || reach_<1. || reach_>3.) return controller_interface::CallbackReturn::ERROR;
  // No legacy topic/action may bypass the session-fenced rolling protocol.
  joint_command_subscriber_.reset(); action_server_.reset(); query_state_srv_.reset();
  rolling_sub_=get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>("~/rolling_trajectory",rclcpp::QoS(1).reliable(),
    [this](trajectory_msgs::msg::JointTrajectory::ConstSharedPtr message){receive(*message);});
  enable_=get_node()->create_service<std_srvs::srv::SetBool>("~/stream_enabled",
    [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,std::shared_ptr<std_srvs::srv::SetBool::Response> response){
      const auto epoch=++epoch_counter_; controls_.writeFromNonRT(Control{epoch,request->data});
      response->success=true; response->message=std::to_string(epoch);
    });
  status_=std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(
    get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/rolling_state",rclcpp::QoS(1).best_effort()));
  status_->msg_.data.resize(60,0.);
  // Fixed-size RT snapshot; formatting/file logging happens in the Python bridge.
  rejection_status_=std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(
    get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/rolling_rejections",rclcpp::QoS(10).reliable()));
  rejection_status_->msg_.data.resize(16,0.);
  return result;
}
void GuardedRollingController::receive(const trajectory_msgs::msg::JointTrajectory& msg) {
  auto p=std::make_shared<RollingPacket>();
  if (!rolling_id(msg.header.frame_id,*p) || msg.joint_names!=params_.joints || msg.points.size()!=2) {
    RCLCPP_WARN(get_node()->get_logger(), "ROLLING INPUT REJECTION: invalid session ID, joint names or point count");
    return;
  }
  const auto& a=msg.points[0]; const auto& b=msg.points[1];
  if (a.positions.size()!=7||a.velocities.size()!=7||a.accelerations.size()!=7||
      b.positions.size()!=7||b.velocities.size()!=7||b.accelerations.size()!=7||
      rclcpp::Duration(a.time_from_start).nanoseconds()!=0) {
    RCLCPP_WARN(get_node()->get_logger(), "ROLLING INPUT REJECTION: invalid q/v/a dimensions or initial time");
    return;
  }
  p->curve.start=rclcpp::Time(msg.header.stamp).seconds();
  p->curve.duration=rclcpp::Duration(b.time_from_start).seconds();
  for(size_t i=0;i<7;++i) {
    if (b.velocities[i]!=0. || b.accelerations[i]!=0.) {
      RCLCPP_WARN(get_node()->get_logger(), "ROLLING INPUT REJECTION: endpoint is not at rest");
      return;
    }
    p->curve.initial.q[i]=a.positions[i]; p->curve.initial.v[i]=a.velocities[i]; p->curve.initial.a[i]=a.accelerations[i]; p->curve.target[i]=b.positions[i];
  }
  p->curve.build();
  if (p->curve.valid(reach_,limits_)) packets_.writeFromNonRT(p);
  else RCLCPP_WARN(get_node()->get_logger(), "ROLLING INPUT REJECTION: derivative, travel or horizon bound");
}
controller_interface::CallbackReturn GuardedRollingController::on_activate(const rclcpp_lifecycle::State& state) {
  auto result=RobotClockTrajectoryController::on_activate(state);
  if(result!=controller_interface::CallbackReturn::SUCCESS) return result;
  for(auto& item:state_interfaces_) if(item.get_name()=="fr3/robot_time") clock_=&item;
  if(!clock_) return controller_interface::CallbackReturn::ERROR;
  for(size_t i=0;i<7;++i) command_.q[i]=last_commanded_state_.positions[i];
  command_.v.fill(0.); command_.a.fill(0.);
  epoch_=++epoch_counter_; controls_.initRT(Control{epoch_,false});
  enabled_=has_active_=has_pending_=false; sequence_=rejected_=ticks_=0; previous_time_=-1.; seen_.reset();
  reported_rejection_=0; rejection_detail_.fill(0.);
  accepted_.initial=command_; accepted_.target=command_.q; accepted_.start=0.; accepted_.duration=.05; accepted_.build();
  return result;
}
void GuardedRollingController::report(double time) {
  if(ticks_%20 || !status_->trylock()) return;
  auto& d=status_->msg_.data;
  d[0]=1.; d[1]=time; d[2]=epoch_; d[3]=sequence_; d[4]=enabled_; d[5]=rejected_;
  d[6]=accepted_.start; d[7]=accepted_.duration; d[8]=reach_;
  for(size_t i=0;i<7;++i) {
    d[9+i]=accepted_.initial.q[i]; d[16+i]=accepted_.initial.v[i]; d[23+i]=accepted_.initial.a[i]; d[30+i]=accepted_.target[i];
    d[37+i]=command_.q[i]; d[44+i]=command_.v[i]; d[51+i]=command_.a[i];
  }
  d[58]=has_pending_; d[59]=has_active_?active_.start+active_.duration:time;
  status_->unlockAndPublish();
  if(rejected_!=reported_rejection_ && rejection_status_->trylock()) {
    std::copy(rejection_detail_.begin(),rejection_detail_.end(),rejection_status_->msg_.data.begin());
    reported_rejection_=rejected_;
    rejection_status_->unlockAndPublish();
  }
}
controller_interface::return_type GuardedRollingController::update(const rclcpp::Time&,const rclcpp::Duration&) {
  const auto reading=clock_?clock_->get_optional():std::nullopt;
  if(!reading || !std::isfinite(*reading) || *reading<0. || *reading>1e9 ||
      (previous_time_>=0. && (*reading<=previous_time_ || *reading-previous_time_>=.020))) {
    const double current=reading.value_or(NAN);
    RCLCPP_ERROR(get_node()->get_logger(),
      "Rolling robot clock invalid/repeated or gap >=20 ms; refusing update; current=%.9f previous=%.9f delta_ms=%.3f",
      current,previous_time_,1000.*(current-previous_time_));
    return controller_interface::return_type::ERROR;
  }
  const double now=*reading; previous_time_=now; ++ticks_;
  if(has_active_) command_=active_.at(now);
  const auto control=*controls_.readFromRT();
  if(control.epoch!=epoch_) {
    bool rest=true;
    for(size_t i=0;i<7;++i) rest=rest&&std::abs(command_.v[i])<1e-10&&std::abs(command_.a[i])<1e-8;
    epoch_=control.epoch; sequence_=0; enabled_=control.enabled&&rest; has_pending_=false;
    accepted_=has_active_?active_:accepted_;
  }
  if(has_pending_ && now>=pending_.start) {active_=pending_;has_active_=true;has_pending_=false;command_=active_.at(now);}
  const auto packet=*packets_.readFromRT();
  if(packet && packet!=seen_) {
    seen_=packet;
    const auto& curve=packet->curve;
    const auto expected=has_active_?active_.at(curve.start):command_;
    // Preserve every admission condition; expose which one refused the packet.
    unsigned reason=0;
    if(!enabled_) reason|=1;
    if(has_pending_) reason|=2;
    if(packet->epoch!=epoch_) reason|=4;
    if(packet->parent!=sequence_) reason|=8;
    if(packet->sequence!=sequence_+1) reason|=16;
    if(curve.start<now+.020) reason|=32;
    if(curve.start>now+.30) reason|=64;
    if(!matching(expected,curve.initial)) reason|=128;
    if(!reason) {
      pending_=curve;accepted_=curve;has_pending_=true;sequence_=packet->sequence;
    } else {
      ++rejected_;
      double q_error=0.,v_error=0.,a_error=0.;
      for(size_t i=0;i<7;++i) {
        q_error=std::max(q_error,std::abs(expected.q[i]-curve.initial.q[i]));
        v_error=std::max(v_error,std::abs(expected.v[i]-curve.initial.v[i]));
        a_error=std::max(a_error,std::abs(expected.a[i]-curve.initial.a[i]));
      }
      rejection_detail_={1.,double(rejected_),now,double(reason),double(packet->epoch),double(packet->sequence),
        double(packet->parent),double(epoch_),double(sequence_),curve.start-now,q_error,v_error,a_error,
        double(has_pending_),double(enabled_),curve.start};
    }
  }
  for(size_t i=0;i<7;++i) {
    const auto measured=joint_state_interface_[0][i].get().get_optional();
    if(!measured||!std::isfinite(*measured)) return controller_interface::return_type::ERROR;
    if(std::abs(*measured-command_.q[i])>.03) {enabled_=false;has_pending_=false;}
    if(!joint_command_interface_[0][i].get().set_value(command_.q[i])) return controller_interface::return_type::ERROR;
  }
  report(now);
  return controller_interface::return_type::OK;
}
}
PLUGINLIB_EXPORT_CLASS(franka_hand_controllers::GuardedRollingController,controller_interface::ControllerInterface)
