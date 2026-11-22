#include "lo_core/registration/registration.hpp"

#include <algorithm>
#include <cmath>

#include <small_gicp/registration/rejector.hpp>

#include "lo_core/util/parallel.hpp"

namespace lo {

double AdaptiveGate::tau() const
{
  if (n_ == 0) {return std::clamp(g_.initial, g_.min, g_.max);}
  const double sigma = std::sqrt(sse_ / static_cast<double>(n_));
  return std::clamp(g_.sigma_factor * sigma, g_.min, g_.max);
}

void AdaptiveGate::update(const SE3 & deviation, double motion)
{
  if (motion < g_.min_motion) {return;}
  const double theta = angleOf(deviation);
  const double err = deviation.translation().norm() + 2.0 * max_range_ * std::sin(0.5 * theta);
  sse_ += err * err;
  ++n_;
}

Mat6 Registration::planeHessian(std::size_t n, int & inliers) const
{
  constexpr int kMaxThreads = 64;
  Mat6 parts[kMaxThreads];
  int counts[kMaxThreads];
  const int threads = std::clamp(cfg_.threads.num_threads, 1, kMaxThreads);
  for (int t = 0; t < threads; ++t) {parts[t].setZero(); counts[t] = 0;}
  const int team = parallelBlocks(static_cast<std::int64_t>(n), threads,
    [&](std::int64_t begin, std::int64_t end, int t) {
      Mat6 sum = Mat6::Zero();
      int c = 0;
      for (std::int64_t i = begin; i < end; ++i) {
        const GicpFactor & f = factors_[static_cast<std::size_t>(i)];
        if (!f.inlier()) {continue;}
        sum += f.weight * f.jp * f.jp.transpose();
        ++c;
      }
      parts[t] = sum;
      counts[t] = c;
    });
  Mat6 total = Mat6::Zero();
  inliers = 0;
  for (int t = 0; t < team; ++t) {total += parts[t]; inliers += counts[t];}
  return total;
}

Registration::Registration(const Config & cfg, std::size_t capacity)
: cfg_(cfg)
{
  factors_.reserve(capacity);
  params_.min_cos = cfg.gicp.orientation_min_cos;
  params_.min_consistency = cfg.gicp.normal_consistency_min;
  reduction_.num_threads = cfg.threads.num_threads;
}

int Registration::optimize(
  const VoxelMap & map, const Cloud & cloud, SE3 & T, double gate, int max_iters, bool project,
  const SE3 * reset_seed, double L, double step_max_t, double step_max_r, AlignResult & out,
  bool & converged)
{
  converged = false;
  if (max_iters <= 0 || map.size() == 0) {return 0;}
  small_gicp::DistanceRejector rejector;
  rejector.max_dist_sq = gate * gate;
  params_.kernel_c = std::max(1e-3, cfg_.kernel.scale_ratio * gate);
  const std::size_t n = cloud.size();
  factors_.resize(n, GicpFactor(GicpFactor::Setting{&params_}));
  for (auto & f : factors_) {f.params = &params_;}

  bool reset_done = (reset_seed == nullptr);
  params_.reuse = false;
  int it = 0;
  for (; it < max_iters; ++it) {
    auto [H, b, e] = reduction_.linearize(map, cloud, map, rejector, toIsometry(T), factors_);
    (void)e;
    if (!H.allFinite() || !b.allFinite()) {
      out.numerical_ok = false;
      break;
    }
    int inliers = 0;
    const Mat6 Hp = planeHessian(n, inliers);
    DegeneracyInfo d;
    if (project) {
      d = analyzeDegeneracy(Hp, L, std::max(inliers, 1), cfg_.degeneracy.lambda_min,
        cfg_.degeneracy.max_directions);
      if (!d.valid) {
        out.numerical_ok = false;
        break;
      }
      if (!reset_done) {
        // value along unobservable axes = seed (DECISIONS I5)
        reset_done = true;
        if (d.num_degenerate > 0) {
          const Vec6 xi = logRT(reset_seed->inverse() * T);
          T = (*reset_seed) * expRT(removeDegenerate(xi, d));
          --it;   // re-linearise at the reset pose without spending an iteration
          continue;
        }
      }
    }
    Vec6 delta;
    if (project) {
      delta = projectedStep(H, b, d, cfg_.gicp.damping);
    } else {
      const Mat6 A = H + cfg_.gicp.damping * Mat6::Identity();
      delta = A.ldlt().solve(-b);
    }
    out.H = H;
    out.Hp = Hp;
    out.b = b;
    out.degeneracy = d;
    if (!delta.allFinite()) {
      out.numerical_ok = false;
      break;
    }
    const double dr = delta.head<3>().norm();
    const double dt = delta.tail<3>().norm();
    out.max_step_rot = std::max(out.max_step_rot, dr);
    out.max_step_trans = std::max(out.max_step_trans, dt);
    if (dt > step_max_t || dr > step_max_r) {
      out.blowup = true;
      break;
    }
    T = T * expRT(delta);
    if (dr < cfg_.gicp.rot_eps && dt < cfg_.gicp.trans_eps) {
      converged = true;
      ++it;
      break;
    }
    // close to the optimum, or late in the budget: freeze correspondences so Gauss-Newton
    // converges on a fixed set (re-searching can oscillate between voxels, DECISIONS I18)
    if ((dr < 10.0 * cfg_.gicp.rot_eps && dt < 10.0 * cfg_.gicp.trans_eps) ||
      it + 1 >= cfg_.gicp.freeze_after)
    {
      params_.reuse = true;
    }
  }
  params_.reuse = false;
  return it;
}

void Registration::collectStats(const Cloud & cloud, AlignResult & out) const
{
  const std::size_t n = cloud.size();
  int inliers = 0, orient = 0;
  double sum_wm = 0.0, sum_pd2 = 0.0;
  for (std::size_t i = 0; i < n && i < factors_.size(); ++i) {
    const GicpFactor & f = factors_[i];
    if (f.orientation_rejected) {++orient;}
    if (!f.inlier()) {continue;}
    ++inliers;
    sum_wm += f.weight * f.maha2;
    sum_pd2 += f.plane_dist * f.plane_dist;
  }
  out.num_points = static_cast<int>(n);
  out.num_inliers = inliers;
  out.num_orientation_rejected = orient;
  out.sum_weighted_maha = sum_wm;
  out.residual_rms = inliers > 0 ? std::sqrt(sum_pd2 / inliers) : 0.0;
}

AlignResult Registration::align(
  const LocalMap & map, const Cloud & cloud, const SE3 & T_seed, double tau, double L,
  double coarse_gate, int coarse_iters)
{
  AlignResult out;
  SE3 T = T_seed;
  bool conv = false;
  // Blow-up limits (DESIGN: step > 0.5 m / 10 deg). A wide recovery gate legitimately takes
  // larger coarse steps, so the coarse limit grows with the gate (DECISIONS I13).
  const double step_t = cfg_.integrity.step_max_trans;
  const double step_r = cfg_.integrity.step_max_rot_deg * kDegToRad;
  const double coarse_scale = std::max(1.0, coarse_gate / std::max(cfg_.gate.max, 1e-3));
  out.iterations_coarse = optimize(map.coarse(), cloud, T, coarse_gate, coarse_iters, false,
    nullptr, L, step_t * coarse_scale, std::min(M_PI, step_r * coarse_scale), out, conv);
  if (!out.numerical_ok || out.blowup) {
    out.T = T;
    collectStats(cloud, out);
    return out;
  }
  out.max_step_rot = 0.0;
  out.max_step_trans = 0.0;
  out.iterations_fine = optimize(map.fine(), cloud, T, tau, cfg_.gicp.fine_iters, true, &T_seed,
    L, step_t, step_r, out, conv);
  out.converged = conv && out.numerical_ok && !out.blowup;
  out.T = T;
  collectStats(cloud, out);
  return out;
}

}  // namespace lo
