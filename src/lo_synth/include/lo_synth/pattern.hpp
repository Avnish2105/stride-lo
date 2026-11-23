// Mid-360-like non-repetitive scan pattern (an approximation, not Livox's proprietary one):
// fixed 200 kHz emission, 4 interleaved lines, 360 deg azimuth, -7..+52 deg elevation,
// mutually incommensurate frequencies so every frame lands somewhere new. Same construction as
// the GPS-denied repo's emulator (sim_sensors/livox_pattern.hpp, documented in its README).
#pragma once

#include <cmath>

#include "lo_core/types/se3.hpp"

namespace lo::synth {

struct Mid360Pattern
{
  double rate = 200000.0;        ///< points per second
  int points_per_scan = 20000;
  double el_min_deg = -7.0;
  double el_max_deg = 52.0;
  double f_az = 137.508;         ///< rotor azimuth frequency [Hz]
  double f_el = 4.7214;          ///< elevation triangle frequency [Hz]
  double f_wobble = 31.83;
  double wobble = 0.0618;

  /// Unit direction (sensor frame) of a point emitted at absolute time t [s] on `line`.
  Vec3 direction(double t, int line) const
  {
    const double lq = 0.25 * static_cast<double>(line);
    const double az = 2.0 * M_PI * frac(f_az * t + lq) - M_PI;
    const double u = f_el * t + lq + wobble * std::sin(2.0 * M_PI * f_wobble * t);
    const double s = 1.0 - std::abs(2.0 * frac(u) - 1.0);
    const double smin = std::sin(el_min_deg * kDegToRad);
    const double smax = std::sin(el_max_deg * kDegToRad);
    const double el = std::asin(smin + s * (smax - smin));   // uniform per solid angle
    return Vec3(std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el));
  }

  static double frac(double x) {return x - std::floor(x);}
};

}  // namespace lo::synth
