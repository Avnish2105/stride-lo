#!/usr/bin/env python3
"""Evaluate a STRIDE output bag (lo_replay --out DIR, or a recording of /stride_lo/* plus GT).

  ros2 run lo_tools evaluate.py OUT_DIR [--gt /ground_truth/odom] [--segment 10]
                                [--fastlio /Odometry --fastlio-frame livox_frame_imu]
                                [--json metrics.json] [--plot plot.png] [--kappa 20]

Metrics (docs/DESIGN.md "Validation and acceptance criteria"):
  RPE translation % and rotation deg/m over SEGMENT-metre GT path segments (LO debug odometry,
  within one chain), factor NEES (6-DoF, GTSAM order, right perturbation, base frame), false
  accepts (NEES > 40), availability (requests answered), and a kappa suggestion (NEES ~ 6).
"""
import argparse
import json
import math
import sys

import numpy as np

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError as e:  # pragma: no cover
    sys.exit(f'evaluate.py needs a sourced ROS 2 environment: {e}')

FALSE_ACCEPT_NEES = 40.0


# ---------------------------------------------------------------- SE(3), rotation first
def quat_to_R(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def pose_to_T(p):
    T = np.eye(4)
    T[:3, :3] = quat_to_R(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
    T[:3, 3] = [p.position.x, p.position.y, p.position.z]
    return T


def so3_log(R):
    c = max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0))
    th = math.acos(c)
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if th < 1e-9:
        return 0.5 * w
    if math.pi - th < 1e-6:   # near pi: use the symmetric part
        A = (R + np.eye(3)) / 2.0
        k = int(np.argmax(np.diag(A)))
        v = A[:, k] / math.sqrt(max(A[k, k], 1e-12))
        return th * v
    return th / (2.0 * math.sin(th)) * w


def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def se3_log(T):
    """Rotation-first tangent [omega; rho] (GTSAM Pose3::Logmap)."""
    w = so3_log(T[:3, :3])
    th = np.linalg.norm(w)
    W = skew(w)
    if th < 1e-9:
        Vinv = np.eye(3) - 0.5 * W
    else:
        Vinv = np.eye(3) - 0.5 * W + (1.0 / th ** 2) * (1 - (th * math.sin(th)) / (2 * (1 - math.cos(th)))) * W @ W
    return np.concatenate([w, Vinv @ T[:3, 3]])


def inv(T):
    Ti = np.eye(4)
    Ti[:3, :3] = T[:3, :3].T
    Ti[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return Ti


def slerp_R(R0, R1, a):
    w = so3_log(R0.T @ R1)
    th = np.linalg.norm(w)
    if th < 1e-12:
        return R0
    k = w / th
    K = skew(k)
    t = a * th
    return R0 @ (np.eye(3) + math.sin(t) * K + (1 - math.cos(t)) * K @ K)


class Trajectory:
    def __init__(self, stamps, poses):
        order = np.argsort(stamps)
        self.t = np.asarray(stamps, dtype=np.int64)[order]
        self.T = [poses[i] for i in order]

    def __len__(self):
        return len(self.t)

    def at(self, t, max_gap_ns=200_000_000):
        i = int(np.searchsorted(self.t, t))
        if i < len(self.t) and self.t[i] == t:
            return self.T[i]
        if i == 0 or i >= len(self.t):
            return None
        t0, t1 = self.t[i - 1], self.t[i]
        if t1 - t0 > max_gap_ns:
            return None
        a = (t - t0) / (t1 - t0)
        T = np.eye(4)
        T[:3, :3] = slerp_R(self.T[i - 1][:3, :3], self.T[i][:3, :3], a)
        T[:3, 3] = (1 - a) * self.T[i - 1][:3, 3] + a * self.T[i][:3, 3]
        return T


def static_transform(tf_msgs, parent, child):
    """T_parent_child composed along the /tf_static tree (None if not connected)."""
    edges = {}
    for tfm in tf_msgs:
        for t in tfm.transforms:
            q, p = t.transform.rotation, t.transform.translation
            T = np.eye(4)
            T[:3, :3] = quat_to_R(q.x, q.y, q.z, q.w)
            T[:3, 3] = [p.x, p.y, p.z]
            edges[t.child_frame_id] = (t.header.frame_id, T)
    T = np.eye(4)
    frame = child
    while frame != parent:
        if frame not in edges:
            return None
        frame, T_up = edges[frame][0], edges[frame][1]
        T = T_up @ T
    return T


def path_length(traj):
    return sum(float(np.linalg.norm(traj.T[k][:3, 3] - traj.T[k - 1][:3, 3])) for k in range(1, len(traj)))


def stamp_ns(s):
    return s.sec * 1_000_000_000 + s.nanosec


def read_bag(path, topics):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in topics if t in types]))
    out = {t: [] for t in topics}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        out[topic].append(deserialize_message(data, get_message(types[topic])))
    return out, types


def rpe(lo, gt, segment, chain_of=None):
    """Relative translation error [%] and rotation error [deg/m] over GT segments of `segment`
    metres (one every 5 poses): (mean trans, mean rot, count, per-segment trans list, rot list)."""
    pairs = [(t, T, gt.at(t)) for t, T in zip(lo.t, lo.T)]
    pairs = [(t, T, G) for t, T, G in pairs if G is not None]
    if len(pairs) < 2:
        return float('nan'), float('nan'), 0, [], []
    dist = [0.0]
    for k in range(1, len(pairs)):
        dist.append(dist[-1] + float(np.linalg.norm(pairs[k][2][:3, 3] - pairs[k - 1][2][:3, 3])))
    et, er = [], []
    j = 0
    for i in range(0, len(pairs), 5):
        j = max(j, i)
        while j < len(pairs) and dist[j] - dist[i] < segment:
            j += 1
        if j >= len(pairs):
            break
        if chain_of is not None and chain_of(pairs[i][0]) != chain_of(pairs[j][0]):
            continue
        d_gt = inv(pairs[i][2]) @ pairs[j][2]
        d_lo = inv(pairs[i][1]) @ pairs[j][1]
        e = inv(d_gt) @ d_lo
        L = dist[j] - dist[i]
        et.append(float(np.linalg.norm(e[:3, 3]) / L * 100.0))
        er.append(float(math.degrees(np.linalg.norm(so3_log(e[:3, :3]))) / L))
    if not et:
        return float('nan'), float('nan'), 0, [], []
    return float(np.mean(et)), float(np.mean(er)), len(et), et, er


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('--gt', default='/ground_truth/odom')
    ap.add_argument('--factor', default='/stride_lo/factor')
    ap.add_argument('--odom', default='/stride_lo/odometry')
    ap.add_argument('--requests', default='/stride_lo/node_request')
    ap.add_argument('--fastlio', default='/Odometry')
    ap.add_argument('--fastlio-frame', default='livox_frame_imu', help='URDF frame FAST-LIO "body" is')
    ap.add_argument('--base-frame', default='base')
    ap.add_argument('--segment', type=float, default=10.0)
    ap.add_argument('--kappa', type=float, default=None, help='kappa used for this run (for the suggestion)')
    ap.add_argument('--json', default=None)
    ap.add_argument('--plot', default=None)
    a = ap.parse_args()

    data, _ = read_bag(a.bag, [a.factor, a.odom, a.gt, a.requests, a.fastlio, '/tf_static'])
    gt = Trajectory([stamp_ns(m.header.stamp) for m in data[a.gt]], [pose_to_T(m.pose.pose) for m in data[a.gt]])
    lo = Trajectory([stamp_ns(m.header.stamp) for m in data[a.odom]], [pose_to_T(m.pose.pose) for m in data[a.odom]])
    if len(gt) < 2:
        sys.exit(f'no ground truth on {a.gt}')

    # chain timeline from the factors
    chains = sorted({(stamp_ns(f.stamp_j), f.chain_id) for f in data[a.factor]})
    chain_t = np.array([c[0] for c in chains], dtype=np.int64)
    chain_id = [c[1] for c in chains]

    def chain_of(t):
        if not chains:
            return 0
        i = int(np.searchsorted(chain_t, t, side='right')) - 1
        return chain_id[max(i, 0)]

    rpe_t, rpe_r, nseg, seg_t, seg_r = rpe(lo, gt, a.segment, chain_of)

    nees, nees_t, healths, false_accepts, worst = [], [], [], [], []
    for f in data[a.factor]:
        ti, tj = stamp_ns(f.stamp_i), stamp_ns(f.stamp_j)
        Gi, Gj = gt.at(ti), gt.at(tj)
        if Gi is None or Gj is None:
            continue
        xi = se3_log(inv(inv(Gi) @ Gj) @ pose_to_T(f.t_ij))
        C = np.array(f.covariance).reshape(6, 6)
        try:
            n = float(xi @ np.linalg.solve(C, xi))
            nt = float(xi[3:] @ np.linalg.solve(C[3:, 3:], xi[3:]))
        except np.linalg.LinAlgError:
            n, nt = float('inf'), float('inf')
        nees.append(n)
        nees_t.append(nt)
        healths.append(f.health)
        if n > FALSE_ACCEPT_NEES:
            false_accepts.append((ti * 1e-9, tj * 1e-9, n, f.health, f.flags))
        worst.append((n, tj * 1e-9))
    req = {stamp_ns(r.stamp) for r in data[a.requests]}
    answered = {stamp_ns(f.stamp_j) for f in data[a.factor]} & req
    avail = len(answered) / len(req) if req else float('nan')

    # FAST-LIO comparison (body = fastlio-frame, mapped to base through /tf_static)
    fl = None
    if data[a.fastlio]:
        T_base_body = static_transform(data['/tf_static'], a.base_frame, a.fastlio_frame)
        if T_base_body is None:
            print(f'FAST-LIO: no /tf_static chain {a.base_frame} -> {a.fastlio_frame}; skipped')
        else:
            T_body_base = inv(T_base_body)
            fl_traj = Trajectory([stamp_ns(m.header.stamp) for m in data[a.fastlio]],
                                 [pose_to_T(m.pose.pose) @ T_body_base for m in data[a.fastlio]])
            fl = rpe(fl_traj, gt, a.segment)

    nees_arr = np.array(nees) if nees else np.array([float('nan')])
    m = {
        'factors': len(data[a.factor]), 'factors_with_gt': len(nees),
        'rpe_trans_pct': rpe_t, 'rpe_rot_deg_per_m': rpe_r, 'rpe_segments': nseg,
        'rpe_trans_median_pct': float(np.median(seg_t)) if seg_t else float('nan'),
        'rpe_trans_p90_pct': float(np.percentile(seg_t, 90)) if seg_t else float('nan'),
        'rpe_trans_max_pct': float(np.max(seg_t)) if seg_t else float('nan'),
        'rpe_rot_median_deg_per_m': float(np.median(seg_r)) if seg_r else float('nan'),
        'path_length_m': float(path_length(gt)),
        'mean_nees': float(np.mean(nees_arr)), 'median_nees': float(np.median(nees_arr)),
        'mean_nees_ok_health': float(np.mean([n for n, h in zip(nees, healths) if h == 0])) if any(h == 0 for h in healths) else float('nan'),
        'mean_nees_trans3': float(np.mean(nees_t)) if nees_t else float('nan'),
        'false_accepts': len(false_accepts), 'availability': avail, 'requests': len(req),
        'health_counts': {int(h): healths.count(h) for h in set(healths)},
    }
    if fl:
        m['fastlio_rpe_trans_pct'], m['fastlio_rpe_rot_deg_per_m'] = fl[0], fl[1]
    if a.kappa and nees:
        m['kappa_suggested'] = a.kappa * m['mean_nees'] / 6.0

    print(f"factors {m['factors']} ({m['factors_with_gt']} with GT)  RPE {rpe_t:.3f} %  {rpe_r:.4f} deg/m "
          f"({nseg} x {a.segment:g} m; median {m['rpe_trans_median_pct']:.3f} %, p90 "
          f"{m['rpe_trans_p90_pct']:.3f} %, max {m['rpe_trans_max_pct']:.3f} %; path {m['path_length_m']:.0f} m)")
    print(f"NEES mean {m['mean_nees']:.2f} median {m['median_nees']:.2f} (OK-health mean "
          f"{m['mean_nees_ok_health']:.2f}, trans-3 mean {m['mean_nees_trans3']:.2f})  "
          f"false accepts {len(false_accepts)}  availability {avail:.3f} of {len(req)} requests")
    if fl:
        print(f"FAST-LIO RPE {fl[0]:.3f} %  {fl[1]:.4f} deg/m")
    if 'kappa_suggested' in m:
        print(f"kappa suggestion for mean NEES 6: {m['kappa_suggested']:.2f} (now {a.kappa:g})")
    for fa in false_accepts[:10]:
        print(f"  FALSE ACCEPT t_i={fa[0]:.3f} t_j={fa[1]:.3f} NEES={fa[2]:.1f} health={fa[3]} flags=0x{fa[4]:x}")
    if a.json:
        with open(a.json, 'w') as f:
            json.dump(m, f, indent=2)
    if a.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(12, 5))
        G0 = gt.at(lo.t[0]) if len(lo) else None
        if G0 is not None and len(lo):
            align = G0 @ inv(lo.T[0])
            P = np.array([(align @ T)[:3, 3] for T in lo.T])
            ax[0].plot(P[:, 0], P[:, 1], label='STRIDE LO')
        Q = np.array([T[:3, 3] for T in gt.T])
        ax[0].plot(Q[:, 0], Q[:, 1], 'k--', label='ground truth')
        ax[0].axis('equal')
        ax[0].legend()
        ax[0].set_title(f'RPE {rpe_t:.2f} %  {rpe_r:.3f} deg/m')
        if worst:
            w = np.array(worst)
            ax[1].semilogy(w[:, 1], w[:, 0], '.', ms=2)
            ax[1].axhline(6, color='g', label='ideal 6')
            ax[1].axhline(FALSE_ACCEPT_NEES, color='r', label='false accept 40')
            ax[1].legend()
            ax[1].set_title(f'factor NEES (mean {m["mean_nees"]:.2f})')
        fig.tight_layout()
        fig.savefig(a.plot, dpi=120)
    return 0 if not false_accepts else 1


if __name__ == '__main__':
    sys.exit(main())
