#include "lo_gtsam_adapter/adapter.hpp"

#include <Eigen/Cholesky>
#include <gtsam/linear/NoiseModel.h>

namespace lo_gtsam {

namespace {

gtsam::Pose3 fromPose(const geometry_msgs::msg::Pose & p)
{
  return gtsam::Pose3(
    gtsam::Rot3::Quaternion(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z),
    gtsam::Point3(p.position.x, p.position.y, p.position.z));
}

}  // namespace

Relative toRelative(const LoFactor & msg, const gtsam::Pose3 & body_T_base)
{
  Relative r;
  const gtsam::Pose3 T = fromPose(msg.t_ij);
  r.covariance = Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(msg.covariance.data());
  r.T_ij = body_T_base * T * body_T_base.inverse();
  const gtsam::Matrix6 Ad = body_T_base.AdjointMap();
  r.covariance = Ad * r.covariance * Ad.transpose();
  r.covariance = 0.5 * (r.covariance + r.covariance.transpose());
  return r;
}

std::optional<gtsam::BetweenFactor<gtsam::Pose3>> toBetweenFactor(
  const LoFactor & msg, gtsam::Key key_i, gtsam::Key key_j, const gtsam::Pose3 & body_T_base)
{
  const Relative r = toRelative(msg, body_T_base);
  if (!r.covariance.allFinite()) {return std::nullopt;}
  Eigen::LLT<gtsam::Matrix6> llt(r.covariance);
  if (llt.info() != Eigen::Success) {return std::nullopt;}
  return gtsam::BetweenFactor<gtsam::Pose3>(key_i, key_j, r.T_ij,
           gtsam::noiseModel::Gaussian::Covariance(r.covariance));
}

ChainTracker::Decision ChainTracker::observe(const LoFactor & msg)
{
  const std::int64_t ti = stampNs(msg.stamp_i), tj = stampNs(msg.stamp_j);
  if (!have_) {
    have_ = true;
    chain_ = msg.chain_id;
    last_j_ = tj;
    return Decision::FIRST;
  }
  if (msg.chain_id < chain_) {return Decision::STALE;}
  if (msg.chain_id > chain_) {
    chain_ = msg.chain_id;
    last_j_ = tj;
    ++breaks_;
    return Decision::NEW_CHAIN;
  }
  if (tj <= last_j_) {return Decision::OUT_OF_ORDER;}
  const Decision d = ti == last_j_ ? Decision::CONTINUE : Decision::GAP;
  last_j_ = tj;
  return d;
}

const char * toString(ChainTracker::Decision d)
{
  switch (d) {
    case ChainTracker::Decision::FIRST: return "first";
    case ChainTracker::Decision::CONTINUE: return "continue";
    case ChainTracker::Decision::GAP: return "gap";
    case ChainTracker::Decision::NEW_CHAIN: return "new_chain";
    case ChainTracker::Decision::STALE: return "stale";
    case ChainTracker::Decision::OUT_OF_ORDER: return "out_of_order";
  }
  return "?";
}

}  // namespace lo_gtsam
