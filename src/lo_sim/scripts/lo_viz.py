#!/usr/bin/env python3
"""lo_viz: makes STRIDE's outputs readable in RViz. Visualisation only: it reads STRIDE's
outputs and ground truth and never publishes anything STRIDE consumes.

  ros2 run lo_sim lo_viz.py --ros-args -p gt_topic:=/ground_truth/odom -p use_sim_time:=true

Publishes
  /tf             stride_odom -> base           (from /stride_lo/odometry: robot model, live scan;
                                                 publish_base_tf:=false if another node owns base)
  /tf_static      world -> stride_odom          (GT(t0) * LO(t0)^-1 at the first LO pose, so LO
                  world -> graph_odom            and ground truth start together and drift shows
                                                 as separation; identity without ground truth)
  ~/lo_path       nav_msgs/Path, LO odometry (stride_odom)
  ~/gt_path       nav_msgs/Path, ground truth (world)
  ~/status        visualization_msgs/Marker, text above the robot: state, health, flags, error
"""
import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, TransformStamped
from lo_msgs.msg import LoFactor
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker

HEALTH = {0: 'OK', 1: 'DEGRADED', 2: 'LOW CONFIDENCE'}
FLAGS = [(1, 'IMU missing'), (2, 'deskew fallback'), (4, 'seed fallback'), (8, 'seed no graph'),
         (16, 'seed identity'), (32, 'sensor blocked'), (64, 'joints missing'),
         (128, 'degenerate'), (256, 'spans gap'), (512, 'after recovery'),
         (1024, 'gyro gate off'), (4096, 'bias uncalibrated')]


# ---- minimal SE(3) on (x, y, z, qx, qy, qz, qw) tuples ------------------------------------
def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def qrot(q, v):
    x, y, z, _ = qmul(qmul(q, (v[0], v[1], v[2], 0.0)), (-q[0], -q[1], -q[2], q[3]))
    return (x, y, z)


def compose(a, b):
    """a * b, poses as (t, q)."""
    r = qrot(a[1], b[0])
    return ((a[0][0] + r[0], a[0][1] + r[1], a[0][2] + r[2]), qmul(a[1], b[1]))


def inverse(a):
    qi = (-a[1][0], -a[1][1], -a[1][2], a[1][3])
    t = qrot(qi, a[0])
    return ((-t[0], -t[1], -t[2]), qi)


def from_pose(p):
    return ((p.position.x, p.position.y, p.position.z),
            (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))


def stamp_ns(s):
    return s.sec * 1_000_000_000 + s.nanosec


class LoViz(Node):
    def __init__(self):
        super().__init__('lo_viz')
        self.gt_topic = self.declare_parameter('gt_topic', '/ground_truth/odom').value
        self.world = self.declare_parameter('world_frame', 'world').value
        self.max_poses = int(self.declare_parameter('max_path_poses', 20000).value)
        self.base_tf = bool(self.declare_parameter('publish_base_tf', True).value)
        self.tf = TransformBroadcaster(self)
        self.tf_static = StaticTransformBroadcaster(self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.lo_path_pub = self.create_publisher(Path, '~/lo_path', latched)
        self.gt_path_pub = self.create_publisher(Path, '~/gt_path', latched)
        self.status_pub = self.create_publisher(Marker, '~/status', 10)
        self.create_subscription(Odometry, '/stride_lo/odometry', self.on_lo, 50)
        self.create_subscription(LoFactor, '/stride_lo/factor', self.on_factor, 100)
        self.create_subscription(DiagnosticArray, '/diagnostics', self.on_diag, 10)
        self.create_subscription(Path, '/lo_graph_demo/path', self.on_graph, 2)
        if self.gt_topic:
            self.create_subscription(Odometry, self.gt_topic, self.on_gt, 200)
        self.gt = []              # (stamp_ns, pose) for alignment and error
        self.lo_path = Path()
        self.gt_path = Path()
        self.world_T_lo = None
        self.graph_aligned = False
        self.static = {}
        self.state = 'waiting for STRIDE'
        self.factors = 0
        self.health = None
        self.flags = 0
        self.error = None
        self.last_lo = None
        self.create_timer(0.5, self.publish_paths)

    # ---- ground truth ----------------------------------------------------------------------
    def on_gt(self, m):
        self.gt.append((stamp_ns(m.header.stamp), from_pose(m.pose.pose)))
        if len(self.gt) > 200000:
            del self.gt[:100000]
        last = self.gt_path.poses[-1].pose.position if self.gt_path.poses else None
        p = m.pose.pose.position
        if last is None or math.dist((p.x, p.y, p.z), (last.x, last.y, last.z)) > 0.05:
            ps = PoseStamped()
            ps.header = m.header
            ps.pose = m.pose.pose
            self.gt_path.poses.append(ps)
            del self.gt_path.poses[:-self.max_poses]
        self.gt_path.header.frame_id = self.world    # GT poses define `world` (see align)
        for ps in self.gt_path.poses[-1:]:
            ps.header.frame_id = self.world

    def gt_at(self, t):
        """Ground-truth pose nearest to t (ns), within 50 ms."""
        if not self.gt:
            return None
        lo, hi = 0, len(self.gt) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if self.gt[mid][0] < t:
                lo = mid + 1
            else:
                hi = mid
        best = min((i for i in (lo - 1, lo) if 0 <= i < len(self.gt)),
                   key=lambda i: abs(self.gt[i][0] - t))
        return self.gt[best][1] if abs(self.gt[best][0] - t) < 50_000_000 else None

    # ---- alignment -------------------------------------------------------------------------
    def send_static(self, child, T):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.world
        t.child_frame_id = child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = T[0]
        (t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z,
         t.transform.rotation.w) = T[1]
        self.static[child] = t
        self.tf_static.sendTransform(list(self.static.values()))

    def align(self, frame, t, pose):
        gt = self.gt_at(t) if self.gt_topic else None
        if self.gt_topic and gt is None:
            return None                       # wait for ground truth around this stamp
        T = compose(gt, inverse(pose)) if gt else ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        self.send_static(frame, T)
        return T

    # ---- STRIDE outputs --------------------------------------------------------------------
    def on_lo(self, m):
        pose = from_pose(m.pose.pose)
        t = stamp_ns(m.header.stamp)
        if self.world_T_lo is None:
            self.world_T_lo = self.align(m.header.frame_id, t, pose)
        if self.base_tf:
            tf = TransformStamped()
            tf.header = m.header
            tf.child_frame_id = m.child_frame_id
            tf.transform.translation.x = m.pose.pose.position.x
            tf.transform.translation.y = m.pose.pose.position.y
            tf.transform.translation.z = m.pose.pose.position.z
            tf.transform.rotation = m.pose.pose.orientation
            self.tf.sendTransform(tf)
        ps = PoseStamped()
        ps.header = m.header
        ps.pose = m.pose.pose
        self.lo_path.header.frame_id = m.header.frame_id
        self.lo_path.poses.append(ps)
        del self.lo_path.poses[:-self.max_poses]
        if self.world_T_lo is not None:
            gt = self.gt_at(t) if self.gt_topic else None
            if gt:
                w = compose(self.world_T_lo, pose)[0]
                self.error = math.dist(w, gt[0])
        self.last_lo = (m.header, pose)
        self.publish_status()

    def on_graph(self, m):
        if not self.graph_aligned and m.poses:
            p = m.poses[0]
            self.graph_aligned = self.align(m.header.frame_id, stamp_ns(p.header.stamp),
                                            from_pose(p.pose)) is not None

    def on_factor(self, m):
        self.factors += 1
        self.health = m.health
        self.flags = m.flags

    def on_diag(self, m):
        for s in m.status:
            if 'stride_lo' in s.name:
                for kv in s.values:
                    if kv.key == 'state':
                        self.state = kv.value

    # ---- outputs ---------------------------------------------------------------------------
    def publish_paths(self):
        now = self.get_clock().now().to_msg()
        for path, pub in ((self.lo_path, self.lo_path_pub), (self.gt_path, self.gt_path_pub)):
            if path.poses:
                path.header.stamp = now
                pub.publish(path)
        if self.last_lo is None:
            self.publish_status()

    def publish_status(self):
        lines = [f'STRIDE: {self.state}']
        if self.health is not None:
            lines.append(f'factor health: {HEALTH.get(self.health, self.health)}   '
                         f'factors: {self.factors}')
        flags = [n for b, n in FLAGS if self.flags & b]
        if flags:
            lines.append('flags: ' + ', '.join(flags))
        if self.error is not None:
            lines.append(f'LO vs ground truth: {self.error:.2f} m')
        mk = Marker()
        if self.last_lo:
            mk.header.frame_id = self.last_lo[0].frame_id
            mk.header.stamp = self.last_lo[0].stamp
            p = self.last_lo[1][0]
            mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = p[0], p[1], p[2] + 1.4
        else:
            mk.header.frame_id = self.world
            mk.pose.position.z = 1.4
        mk.pose.orientation.w = 1.0
        mk.ns = 'status'
        mk.type = Marker.TEXT_VIEW_FACING
        mk.scale.z = 0.25
        ok = self.state in ('TRACKING', 'waiting for STRIDE') and self.health in (None, 0)
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = (1.0, 1.0, 1.0, 1.0) if ok else \
            (1.0, 0.75, 0.2, 1.0)
        if self.state in ('LOST', 'FAULT'):
            mk.color.r, mk.color.g, mk.color.b = 1.0, 0.3, 0.3
        mk.text = '\n'.join(lines)
        self.status_pub.publish(mk)


def main():
    rclpy.init()
    node = LoViz()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass                                  # Ctrl-C / launch shutdown: the context is gone
    except rclpy._rclpy_pybind11.RCLError:
        if rclpy.ok():                        # shutdown race: context invalidated mid-spin
            raise
    finally:
        if rclpy.ok():
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
