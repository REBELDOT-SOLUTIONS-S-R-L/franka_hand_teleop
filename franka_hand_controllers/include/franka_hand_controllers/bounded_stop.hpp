#pragma once

#include <algorithm>
#include <cmath>

namespace franka_hand_controllers {
struct ProgressStop {
  double duration{0.};
  double endpoint{0.};
};

// Integrate a cubic velocity ramp matching the current velocity/acceleration
// and ending at rest. Reject reversal, overshoot, or derivatives larger than
// the original rest-to-rest quintic. Common progress preserves its joint path.
inline bool choose_progress_stop(double s, double v, double a, double original_duration,
                                 double remaining, ProgressStop& result) {
  if (!std::isfinite(s) || !std::isfinite(v) || !std::isfinite(a) ||
      original_duration <= 0. || remaining <= .001 || v < -1e-10 || s < -1e-10 || s > 1.+1e-10)
    return false;
  s = std::clamp(s, 0., 1.);
  const double vmax = 1.875/original_duration;
  const double amax = (10./std::sqrt(3.))/(original_duration*original_duration);
  const double jmax = 60./std::pow(original_duration, 3);
  for (int milliseconds = 1; milliseconds*.001 < remaining; ++milliseconds) {
    const double t = milliseconds*.001;
    const double endpoint = s + .5*v*t + a*t*t/12.;
    if (3*v+a*t < -1e-12 || endpoint < s-1e-12 || endpoint > 1.+1e-12)
      continue;
    const double b = -6*v/t-4*a, c = 6*v/t+3*a;
    double peak_a = std::max(std::abs(a), std::abs(a+b+c));
    if (std::abs(c) > 1e-15) {
      const double r = -b/(2*c);
      if (r > 0. && r < 1.)
        peak_a = std::max(peak_a, std::abs(a+b*r+c*r*r));
    }
    if (peak_a > amax+1e-10 || std::max(std::abs(b/t), std::abs((b+2*c)/t)) > jmax+1e-9)
      continue;
    auto velocity = [=](double r) { return (1-r)*(1-r)*(v*(1+2*r)+a*t*r); };
    double peak_v = std::max(v, 0.);
    const double discriminant = b*b-4*c*a;
    if (std::abs(c) > 1e-15 && discriminant >= 0.) {
      for (double r : {(-b+std::sqrt(discriminant))/(2*c), (-b-std::sqrt(discriminant))/(2*c)})
        if (r > 0. && r < 1.)
          peak_v = std::max(peak_v, velocity(r));
    } else if (std::abs(b) > 1e-15) {
      const double r = -a/b;
      if (r > 0. && r < 1.)
        peak_v = std::max(peak_v, velocity(r));
    }
    if (peak_v <= vmax+1e-10) {
      result = {t, std::clamp(endpoint, s, 1.)};
      return true;
    }
  }
  return false;
}
}  // namespace franka_hand_controllers
