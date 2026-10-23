#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

namespace franka_hand_controllers {
// Per-session settings come exclusively from the selected motion profile.
// NaN defaults deliberately reject motion when configuration is missing.
struct RollingLimits {
  double joint_step{NAN}, tool_travel{NAN}, tool_speed{NAN}, tool_acceleration{NAN};
  double joint_speed{NAN}, joint_acceleration{NAN}, joint_jerk{NAN}, maximum_horizon{NAN};
  bool valid() const {
    for (double v:{joint_step,tool_travel,tool_speed,tool_acceleration,joint_speed,
                   joint_acceleration,joint_jerk,maximum_horizon})
      if (!std::isfinite(v) || v<=0.) return false;
    // FR3 manufacturer envelope; joint speed uses the lowest nominal axis maximum.
    // Position-dependent limits and the installed robot model still apply.
    return tool_speed<=3. && tool_acceleration<=9. && joint_speed<=2.62 &&
      joint_acceleration<=10. && joint_jerk<=5000. &&
      maximum_horizon>=.05 && maximum_horizon<=1.5;
  }
};
using Joints = std::array<double, 7>;
struct RollingState { Joints q{}, v{}, a{}; };
inline double choose(int n, int k) {
  double value = 1.;
  for (int i=1; i<=k; ++i) value *= double(n-i+1)/i;
  return value;
}
inline std::vector<double> bernstein(const std::vector<double>& c) {
  const int n = c.size()-1;
  std::vector<double> b(c.size(), 0.);
  for (int k=0; k<=n; ++k)
    for (int j=0; j<=k; ++j) b[k] += c[j]*choose(k,j)/choose(n,j);
  return b;
}
inline double hull_peak(const std::vector<double>& b, int depth=5) {
  if (!depth) {
    double peak=0.;
    for (auto x:b) peak=std::max(peak,std::abs(x));
    return peak;
  }
  auto row=b;
  std::vector<double> left{row.front()}, right{row.back()};
  while (row.size()>1) {
    for (size_t i=0;i+1<row.size();++i) row[i]=(row[i]+row[i+1])*.5;
    row.pop_back(); left.push_back(row.front()); right.push_back(row.back());
  }
  std::reverse(right.begin(),right.end());
  return std::max(hull_peak(left,depth-1),hull_peak(right,depth-1));
}
struct RollingCurve {
  RollingState initial;
  Joints target{};
  double start{0.}, duration{0.};
  std::array<std::array<double,6>,7> c{};
  void build() {
    for (size_t i=0;i<7;++i) {
      const double c0=initial.q[i], c1=initial.v[i]*duration, c2=.5*initial.a[i]*duration*duration;
      const double d=target[i]-c0-c1-c2, dv=-c1-2*c2, da=-2*c2;
      c[i]={c0,c1,c2,10*d-4*dv+.5*da,-15*d+7*dv-da,6*d-3*dv+.5*da};
    }
  }
  RollingState at(double time) const {
    const double x=std::clamp((time-start)/duration,0.,1.);
    RollingState state;
    for (size_t i=0;i<7;++i) {
      double q=0.,v=0.,a=0.;
      for (int j=5;j>=0;--j) q=q*x+c[i][j];
      for (int j=5;j>=1;--j) v=v*x+j*c[i][j];
      for (int j=5;j>=2;--j) a=a*x+j*(j-1)*c[i][j];
      state.q[i]=q; state.v[i]=v/duration; state.a[i]=a/(duration*duration);
    }
    if (time>=start+duration) { state.q=target; state.v.fill(0.); state.a.fill(0.); }
    return state;
  }
  bool valid(double reach, const RollingLimits& limits) const {
    if (!limits.valid()||!std::isfinite(reach)||!std::isfinite(start)||!std::isfinite(duration)||duration<.05||duration>limits.maximum_horizon||reach<1.) return false;
    double velocity_sum=0.,acceleration_sum=0.,travel_sum=0.;
    for (size_t i=0;i<7;++i) {
      for (auto x:c[i]) if (!std::isfinite(x)) return false;
      std::vector<double> coefficients(c[i].begin(),c[i].end());
      const auto controls=bernstein(coefficients);
      double travel=0.;
      for (size_t j=1;j<controls.size();++j) travel+=std::abs(controls[j]-controls[j-1]);
      if (travel>limits.joint_step+1e-12) return false;
      travel_sum+=travel;
      for (int order=1;order<=3;++order) {
        for (size_t j=1;j<coefficients.size();++j) coefficients[j-1]=j*coefficients[j];
        coefficients.pop_back();
        const double peak=hull_peak(bernstein(coefficients))/std::pow(duration,order);
        if (peak>(order==1?limits.joint_speed:order==2?limits.joint_acceleration:limits.joint_jerk)+1e-10) return false;
        if (order==1) velocity_sum+=peak;
        if (order==2) acceleration_sum+=peak;
      }
    }
    return reach*travel_sum<=limits.tool_travel+1e-10 && reach*velocity_sum<=limits.tool_speed+1e-10 &&
      reach*(acceleration_sum+velocity_sum*velocity_sum)<=limits.tool_acceleration+1e-10;
  }
  double speed_bound(double reach) const {
    double sum=0.;
    for(const auto& coefficients:c) {
      std::vector<double> velocity;
      for(size_t j=1;j<coefficients.size();++j) velocity.push_back(j*coefficients[j]);
      sum+=hull_peak(bernstein(velocity))/duration;
    }
    return reach*sum;
  }
};
struct RollingPacket { uint64_t epoch{0}, sequence{0}, parent{0}; RollingCurve curve; };
inline bool rolling_id(const std::string& id, RollingPacket& packet) {
  std::istringstream s(id); std::string prefix; char a,b;
  std::getline(s,prefix,'/');
  if (prefix!="rolling1") return false;
  if (!(s>>packet.epoch>>a>>packet.sequence>>b>>packet.parent) || a!='/' || b!='/' || !s.eof()) return false;
  return packet.epoch>0 && packet.sequence>0 && packet.epoch<9007199254740992ULL && packet.sequence<9007199254740992ULL;
}
inline bool matching(const RollingState& a,const RollingState& b) {
  for (size_t i=0;i<7;++i)
    if (std::abs(a.q[i]-b.q[i])>1e-9 || std::abs(a.v[i]-b.v[i])>1e-8 || std::abs(a.a[i]-b.a[i])>1e-7) return false;
  return true;
}
}  // namespace franka_hand_controllers
