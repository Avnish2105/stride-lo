// lo_gtsam_adapter: lo_msgs/LoFactor -> gtsam::BetweenFactor<Pose3> (docs/DESIGN.md "Interfaces").
//
// Conventions (CLAUDE.md rule 3): LoFactor carries T_ij (base_i -> base_j) and a 6x6 Sigma in
// [rx ry rz tx ty tz] order with RIGHT perturbation, T_ij,true = T_ij * Exp(xi), xi ~ N(0, Sigma).
// GTSAM's Pose3 (built with POSE3_EXPMAP) uses the same tangent (rotation first, local/right
// perturbation), so Sigma maps 1:1 onto noiseModel::Gaussian::Covariance in the base frame.
// A backend whose state frame is not `base` (factor_graph_estimator: the IMU frame) passes
// body_T_base:  T_body,ij = body_T_base * T_ij * body_T_base^-1,
//               Sigma_body = Ad(body_T_base) Sigma Ad(body_T_base)^T.
#pragma once

#include <cstdint>
#include <optional>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/slam/BetweenFactor.h>
#include <lo_msgs/msg/lo_factor.hpp>

namespace lo_gtsam {

using LoFactor = lo_msgs::msg::LoFactor;

/// Nanoseconds of a builtin_interfaces/Time.
inline std::int64_t stampNs(const builtin_interfaces::msg::Time & t)
{
  return static_cast<std::int64_t>(t.sec) * 1000000000LL + static_cast<std::int64_t>(t.nanosec);
}

struct Relative
{
  gtsam::Pose3 T_ij;             ///< body_i -> body_j
  gtsam::Matrix6 covariance;     ///< [rx ry rz tx ty tz], right perturbation, body frame
};

/// T_ij and Sigma of the message, expressed in the backend body frame.
Relative toRelative(const LoFactor & msg, const gtsam::Pose3 & body_T_base = gtsam::Pose3());

/// BetweenFactor<Pose3>(key_i, key_j) with the full covariance; nullopt if Sigma is not
/// symmetric positive definite (never add such a factor).
std::optional<gtsam::BetweenFactor<gtsam::Pose3>> toBetweenFactor(
  const LoFactor & msg, gtsam::Key key_i, gtsam::Key key_j,
  const gtsam::Pose3 & body_T_base = gtsam::Pose3());

/// Factors only connect frames of one chain (chain_id = stamp of the chain's first frame; a new
/// chain means the LO restarted its map). The tracker classifies each incoming factor:
///  FIRST      first factor seen
///  CONTINUE   same chain, starts where the previous factor ended
///  GAP        same chain, starts later (a factor message was lost; still valid)
///  NEW_CHAIN  a newer chain: no factor will ever connect it to the previous chain
///  STALE      an older chain: drop
///  OUT_OF_ORDER  same chain, ends at or before the last one: drop (duplicate / reordered)
class ChainTracker
{
public:
  enum class Decision { FIRST, CONTINUE, GAP, NEW_CHAIN, STALE, OUT_OF_ORDER };

  Decision observe(const LoFactor & msg);
  static bool usable(Decision d) {return d != Decision::STALE && d != Decision::OUT_OF_ORDER;}
  std::uint64_t chain() const {return chain_;}
  std::uint64_t breaks() const {return breaks_;}

private:
  bool have_{false};
  std::uint64_t chain_{0};
  std::int64_t last_j_{0};
  std::uint64_t breaks_{0};
};

const char * toString(ChainTracker::Decision d);

}  // namespace lo_gtsam
