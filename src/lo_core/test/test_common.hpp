// Helpers shared by the lo_core unit tests (header-only, test-only).
#pragma once

#include <random>
#include <vector>

#include "lo_core/types/points.hpp"
#include "lo_core/types/se3.hpp"

namespace lo::test {

inline SE3 randomPose(std::mt19937_64 & rng, double trans = 2.0, double rot = 1.0)
{
  std::normal_distribution<double> n(0.0, 1.0);
  Vec6 xi;
  xi << rot * n(rng), rot * n(rng), rot * n(rng), trans * n(rng), trans * n(rng), trans * n(rng);
  return expRT(xi);
}

/// Points on the 6 faces of a box room (inside), plus two pillars: well-constrained geometry.
inline std::vector<Vec3> roomPoints(double sx, double sy, double sz, double step,
  std::mt19937_64 & rng, double noise = 0.0)
{
  std::normal_distribution<double> n(0.0, noise);
  std::vector<Vec3> pts;
  const double hx = sx / 2, hy = sy / 2;
  for (double a = -hx; a <= hx; a += step) {
    for (double z = 0.0; z <= sz; z += step) {
      pts.emplace_back(a, -hy + n(rng), z);
      pts.emplace_back(a, hy + n(rng), z);
    }
  }
  for (double b = -hy; b <= hy; b += step) {
    for (double z = 0.0; z <= sz; z += step) {
      pts.emplace_back(-hx + n(rng), b, z);
      pts.emplace_back(hx + n(rng), b, z);
    }
  }
  for (double a = -hx; a <= hx; a += step) {
    for (double b = -hy; b <= hy; b += step) {
      pts.emplace_back(a, b, 0.0 + n(rng));
      pts.emplace_back(a, b, sz + n(rng));
    }
  }
  for (double ang = 0.0; ang < 2 * M_PI; ang += step / 0.3) {
    for (double z = 0.0; z <= sz; z += step) {
      pts.emplace_back(1.0 + 0.3 * std::cos(ang), 0.7 + 0.3 * std::sin(ang), z);
      pts.emplace_back(-1.5 + 0.2 * std::cos(ang), -0.8 + 0.2 * std::sin(ang), z);
    }
  }
  return pts;
}

}  // namespace lo::test
