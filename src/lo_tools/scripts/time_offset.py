#!/usr/bin/env python3
"""Estimate imu.time_offset without ground truth (docs/TUNING.md, DECISIONS I25).

  ros2 run lo_tools time_offset.py REPLAY_OUT BAG [--imu /livox/imu] [--lidar-frame livox_frame]
                                   [--range-ms 10] [--step-ms 0.5] [--gt TOPIC]

Integrates the bag's gyro over windows [t_i + d, t_j + d] between consecutive STRIDE odometry
poses (REPLAY_OUT = lo_replay --out, run with the current offsets) and compares the result with
LO's registered rotation over [t_i, t_j] in the IMU frame. Prints the RMS disagreement against
d. The raw bag IMU stamps are used, so the minimum gives imu.time_offset = -d directly (an
absolute value, whatever offset the replay ran with). LO rotation comes from scan-to-map
registration, so it is nearly independent of the deskew it was computed with. With --gt, the
reference is ground truth instead of LO (this is how the sim-bag offset was measured).
Assumes interval-mean IMU samples (imu.sample_convention), the same as the replay config.
"""
import argparse
import math
import sys

import numpy as np

sys.path.insert(0, __file__.rsplit('/', 1)[0])
import evaluate as ev  # noqa: E402

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError as e:  # pragma: no cover
    sys.exit(f'time_offset.py needs a sourced ROS 2 environment: {e}')


def expm(w):
    th = float(np.linalg.norm(w))
    if th < 1e-12:
        return np.eye(3)
    k = w / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('replay_out')
    ap.add_argument('bag')
    ap.add_argument('--imu', default='/livox/imu')
    ap.add_argument('--base-frame', default='base')
    ap.add_argument('--imu-frame', default='livox_frame')
    ap.add_argument('--range-ms', type=float, default=10.0)
    ap.add_argument('--step-ms', type=float, default=0.5)
    ap.add_argument('--gt', default=None, help='use this ground-truth odometry topic as reference')
    a = ap.parse_args()

    topics = ['/stride_lo/odometry', '/tf_static'] + ([a.gt] if a.gt else [])
    data, _ = ev.read_bag(a.replay_out, topics)
    T_base_imu = ev.static_transform(data['/tf_static'], a.base_frame, a.imu_frame)
    if T_base_imu is None:
        sys.exit(f'no /tf_static chain {a.base_frame} -> {a.imu_frame} in {a.replay_out}')
    R_bi = T_base_imu[:3, :3]
    ref_msgs = data[a.gt] if a.gt else data['/stride_lo/odometry']
    ref = ev.Trajectory([ev.stamp_ns(m.header.stamp) for m in ref_msgs],
                        [ev.pose_to_T(m.pose.pose) for m in ref_msgs])
    lo = data['/stride_lo/odometry']
    t_lo = [ev.stamp_ns(m.header.stamp) for m in lo]

    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=a.bag, storage_id=''), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {m.name: m.type for m in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[a.imu]))
    ts, W = [], []
    while r.has_next():
        _, d, _ = r.read_next()
        m = deserialize_message(d, get_message(types[a.imu]))
        ts.append(ev.stamp_ns(m.header.stamp))
        W.append([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
    ts = np.array(ts, dtype=np.int64)
    W = np.array(W)
    if len(ts) < 100:
        sys.exit('too few IMU samples')
    # bias: median gyro over the first 2 s (the robot stands at start-up for INIT)
    bias = np.median(W[ts < ts[0] + 2_000_000_000], axis=0)

    def gyro_rotation(t0, t1):
        """Interval-mean samples: sample k covers (ts[k-1], ts[k]]; exact on partial overlaps."""
        k0 = int(np.searchsorted(ts, t0, side='right'))
        R = np.eye(3)
        k = k0
        while k < len(ts) and ts[k - 1] < t1:
            lo_t, hi_t = max(ts[k - 1], t0), min(ts[k], t1)
            if hi_t > lo_t:
                R = R @ expm((W[k] - bias) * (hi_t - lo_t) * 1e-9)
            k += 1
        return R

    pairs = []
    for ti, tj in zip(t_lo[:-1], t_lo[1:]):
        if not 50_000_000 <= tj - ti <= 200_000_000:
            continue
        A, B = ref.at(ti), ref.at(tj)
        if A is None or B is None:
            continue
        R_ref = R_bi.T @ (A[:3, :3].T @ B[:3, :3]) @ R_bi   # IMU frame
        if np.linalg.norm(ev.so3_log(R_ref)) < 0.01:      # needs rotation to be informative
            continue
        pairs.append((ti, tj, R_ref))
    if len(pairs) < 50:
        sys.exit(f'only {len(pairs)} informative intervals (the robot must turn / sway)')
    print(f'{len(pairs)} intervals with rotation >= 0.01 rad; reference: {a.gt or "LO odometry"}')
    best = None
    for d_ms in np.arange(-a.range_ms, a.range_ms + 1e-9, a.step_ms):
        d = int(round(d_ms * 1e6))
        err = [np.linalg.norm(ev.so3_log(gyro_rotation(ti + d, tj + d).T @ R)) for ti, tj, R in pairs]
        rms = math.sqrt(float(np.mean(np.square(err)))) * 1e3
        print(f'  shift {d_ms:+6.2f} ms: rms {rms:.3f} mrad')
        if best is None or rms < best[1]:
            best = (d_ms, rms)
    print(f'best: gyro window shifted {best[0]:+.2f} ms -> imu.time_offset: {-best[0] / 1000.0:+.5f}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
