#!/usr/bin/env python3
"""Inject faults into a bag (MCAP) and write a new bag (docs/DESIGN.md fault-injection suite).

  ros2 run lo_tools fault_inject.py IN_BAG OUT_BAG --fault TYPE[:key=val,...] [--fault ...]

Faults (times in s relative to the first LiDAR stamp of the bag):
  imu_dropout:start=S,duration=D          no samples on the deskew IMU topic in the window
  clock_jump:start=S,duration=D,offset=O  LiDAR stamps shifted by O seconds inside the window
  ptp_drift:start=S,duration=D,rate=R     LiDAR stamps drift by R s/s inside the window (PTP lost)
  point_dropout:fraction=F[,start,duration]  fraction F of the points become no-returns
  lens_blocked:start=S,duration=D         the forward half of the FoV returns dust at 0.15-0.4 m
  leg_points:start=S,duration=D[,x,y,z]   a body-close cluster (a leg in the FoV, joints unknown)
  imu_vibration:accel_std=A[,gyro_std=G]  white vibration noise on the deskew IMU
Topics: --lidar /livox/lidar --imu /livox/imu (defaults). Every other topic is copied verbatim.
The output is deterministic for a given --seed.
"""
import argparse
import random
import sys

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message, serialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError as e:  # pragma: no cover
    sys.exit(f'fault_inject.py needs a sourced ROS 2 environment: {e}')

KINDS = ('imu_dropout', 'clock_jump', 'ptp_drift', 'point_dropout', 'lens_blocked', 'leg_points',
         'imu_vibration')


def parse_fault(spec):
    kind, _, rest = spec.partition(':')
    if kind not in KINDS:
        sys.exit(f'unknown fault "{kind}" (one of {", ".join(KINDS)})')
    params = {}
    for kv in filter(None, rest.split(',')):
        k, _, v = kv.partition('=')
        params[k] = float(v)
    return kind, params


def active(p, t):
    s = p.get('start', 0.0)
    return s <= t < s + p.get('duration', 1e12)


def stamp_s(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def open_reader(uri):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id=''),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    return reader


def corrupt_scan(msg, faults, rel, rng, counts):
    """Applies the point faults in place; returns the stamp offset (s) of the clock faults."""
    offset = 0.0
    for kind, p in faults:
        if not active(p, rel):
            continue
        if kind == 'clock_jump':
            offset += p['offset']
        elif kind == 'ptp_drift':
            offset += p['rate'] * (rel - p.get('start', 0.0))
        elif kind == 'point_dropout':
            f = p.get('fraction', 0.5)
            for pt in msg.points:
                if rng.random() < f:
                    pt.x = pt.y = pt.z = 0.0
        elif kind == 'lens_blocked':
            for pt in msg.points:
                n = (pt.x * pt.x + pt.y * pt.y + pt.z * pt.z) ** 0.5
                if pt.x > 0.0 and n > 0.0:
                    r = 0.15 + 0.25 * rng.random()
                    pt.x, pt.y, pt.z = pt.x / n * r, pt.y / n * r, pt.z / n * r
                    pt.reflectivity = 12
        elif kind == 'leg_points':
            x, y, z = p.get('x', 0.35), p.get('y', 0.15), p.get('z', -0.2)
            for k in range(0, len(msg.points), 25):
                pt = msg.points[k]
                pt.x = x + 0.03 * rng.gauss(0, 1)
                pt.y = y + 0.03 * rng.gauss(0, 1)
                pt.z = z + 0.10 * rng.gauss(0, 1)
        else:
            continue
        counts[kind] = counts.get(kind, 0) + 1
    return offset


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('inp')
    ap.add_argument('out')
    ap.add_argument('--fault', action='append', default=[], required=True)
    ap.add_argument('--lidar', default='/livox/lidar')
    ap.add_argument('--imu', default='/livox/imu')
    ap.add_argument('--seed', type=int, default=1)
    a = ap.parse_args()
    faults = [parse_fault(f) for f in a.fault]
    rng = random.Random(a.seed)

    # first LiDAR stamp = time origin of the fault windows
    reader = open_reader(a.inp)
    metas = reader.get_all_topics_and_types()
    types = {m.name: m.type for m in metas}
    if a.lidar not in types:
        sys.exit(f'{a.inp}: no topic {a.lidar}')
    lidar_t = get_message(types[a.lidar])
    imu_t = get_message(types[a.imu]) if a.imu in types else None
    reader.set_filter(rosbag2_py.StorageFilter(topics=[a.lidar]))
    t_first = None
    if reader.has_next():
        _, data, _ = reader.read_next()
        m = deserialize_message(data, lidar_t)
        t_first = m.timebase * 1e-9 if m.timebase else stamp_s(m.header)
    del reader
    if t_first is None:
        sys.exit(f'{a.inp}: {a.lidar} is empty')

    reader = open_reader(a.inp)
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=a.out, storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    for m in metas:
        writer.create_topic(m)
    counts = {}
    vibration = [p for k, p in faults if k == 'imu_vibration']
    dropouts = [p for k, p in faults if k == 'imu_dropout']
    while reader.has_next():
        topic, data, recv = reader.read_next()
        if topic == a.lidar:
            msg = deserialize_message(data, lidar_t)
            t0 = msg.timebase * 1e-9 if msg.timebase else stamp_s(msg.header)
            offset = corrupt_scan(msg, faults, t0 - t_first, rng, counts)
            if offset:
                new_t = int(round((t0 + offset) * 1e9))
                msg.timebase = new_t
                msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(new_t, 1_000_000_000)
            writer.write(topic, serialize_message(msg), recv)
        elif topic == a.imu and imu_t is not None and (vibration or dropouts):
            msg = deserialize_message(data, imu_t)
            rel = stamp_s(msg.header) - t_first
            if any(active(p, rel) for p in dropouts):
                counts['imu_dropout'] = counts.get('imu_dropout', 0) + 1
                continue
            for p in vibration:
                sa, sg = p.get('accel_std', 0.0), p.get('gyro_std', 0.0)
                msg.linear_acceleration.x += rng.gauss(0, sa)
                msg.linear_acceleration.y += rng.gauss(0, sa)
                msg.linear_acceleration.z += rng.gauss(0, sa)
                msg.angular_velocity.x += rng.gauss(0, sg)
                msg.angular_velocity.y += rng.gauss(0, sg)
                msg.angular_velocity.z += rng.gauss(0, sg)
                counts['imu_vibration'] = counts.get('imu_vibration', 0) + 1
            writer.write(topic, serialize_message(msg), recv)
        else:
            writer.write(topic, data, recv)
    del writer
    print(f'fault_inject: wrote {a.out}; affected messages {counts}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
