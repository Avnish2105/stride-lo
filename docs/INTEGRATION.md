# Integrating STRIDE into your stack

This guide is for anyone putting STRIDE LiDAR odometry into their own robot or estimator, for example in place of FAST-LIO. It covers what STRIDE needs, what it gives back, how a factor-graph backend should consume it, and what has been validated so far.

## 1. What STRIDE is (and is not)

STRIDE is a **measurement generator**, not an estimator. For each graph node time you ask for, it emits one **relative-pose factor**, `lo_msgs/LoFactor`:
- `T_ij`: motion of the base frame between two node times;
- a calibrated 6×6 covariance;
- a health value.

The factor contains no IMU information. The gyro is used only to deskew scans, seed registration, and reject bad matches. STRIDE prefers dropping a factor to emitting one that is confidently wrong: every failed check means either no factor or a factor with inflated Σ.

**It is not a drop-in FAST-LIO replacement.**

| | FAST-LIO | STRIDE |
|---|---|---|
| Output | `/Odometry`: absolute pose, IMU-fused | `~/factor`: relative pose between *your* node times + Σ |
| IMU in output | yes (tightly coupled) | no: your backend fuses its own IMU |
| Covariance | not calibrated | NEES-calibrated, inflated along unobservable directions |
| Consumer | anything that reads odometry | a factor-graph backend (GTSAM adapter provided) |

`/stride_lo/odometry` exists for RViz and debugging only. **Never fuse it.** It is LO dead-reckoning with no integrity semantics.

If you have no backend yet, `lo_graph_demo` (section 5) is a minimal iSAM2 graph built from STRIDE factors only. It publishes `~/pose` (with marginal covariance) and `~/path`, so it can stand in for FAST-LIO's odometry while you build a real backend.

## 2. Inputs

| Topic (param) | Type | Required | Notes |
|---|---|---|---|
| `topics.lidar` | `livox_ros_driver2/CustomMsg` | yes | Livox Mid-360, `xfer_format: 1` (per-point time offsets). Other LiDARs are not supported. |
| `topics.imu` | `sensor_msgs/Imu` | yes (degrades without it) | Gyro for deskew and seed. The Mid-360's internal IMU is fine; set `imu.accel_scale` 9.80665 (the driver reports g). |
| `topics.joints` | `sensor_msgs/JointState` | no | Leg self-filter by forward kinematics (needs a URDF) and standstill detection. |
| `topics.graph_odom` | `nav_msgs/Odometry` | no | Your backend's odometry, used **only** to seed translation. It is sanity-gated, and ignored for 2 s once it fails. |
| `topics.node_request` | `lo_msgs/NodeRequest` | for request mode | One message per graph node time (section 4). |
| `topics.ptp_offset` | `std_msgs/Float64` | no | PTP offset [s] from your PTP monitor (clock-drift check). |
| TF | `/tf_static` | if `extrinsics.source: tf` | `base → lidar` and `lidar → imu`. Alternatively set `extrinsics.source: params` with `extrinsics.base_lidar` / `extrinsics.lidar_imu` as `[x y z qx qy qz qw]`. |

All decisions use message stamps. LiDAR and IMU must share a clock (PTP, or the Livox internal IMU).

## 3. Outputs

| Topic | Type | Use |
|---|---|---|
| `/stride_lo/factor` | `lo_msgs/LoFactor` | **The** output. Reliable QoS, depth 100. |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Health state (INIT / TRACKING / DEGRADED / LOST / RECOVERING), timing, reject reasons. |
| `/stride_lo/odometry` | `nav_msgs/Odometry` | Debug only. Never fuse. |
| `/stride_lo/deskewed`, `/stride_lo/local_map` | `PointCloud2` | Debug only. |

### LoFactor fields

- **`stamp_i`, `stamp_j`.** Node times. In request mode `stamp_j` is exactly a time you requested, and `stamp_i` is the previous emitted one.
- **`t_ij`.** Base-frame relative pose `T_ij = T_wi⁻¹ · T_wj`.
- **`covariance`.**
  - Row-major 6×6, **order `[rx ry rz tx ty tz]`** (rotation first).
  - **Right (body-frame) perturbation**: `T_true = T_ij · Exp(ξ)`, `ξ ~ N(0, Σ)`, in the base frame.
  - This matches GTSAM `Pose3`, so Σ goes straight into `noiseModel::Gaussian::Covariance`.
  - The order is *not* the ROS `PoseWithCovariance` order `[x y z rx ry rz]`.
- **`health`.**
  - `OK`.
  - `DEGRADED`: some directions are unobservable, e.g. a corridor. Σ is set to 1 m / 0.5 rad along `degenerate_directions`, and the value there is the seed.
  - `LOW_CONFIDENCE`: the whole of Σ is inflated, e.g. after a deskew fallback, IMU loss, a blocked sensor or recovery.
  - All three are safe to add, because Σ already reflects the doubt. Dropping `LOW_CONFIDENCE` is a policy choice.
- **`chain_id`.** Increments when STRIDE restarts its map, after LOST or after a long gap. **No factor ever connects two chains.** Your backend must bridge a chain break with its own IMU or leg factors.
- **`flags`.** Bitfield with the reasons (see `lo_msgs/msg/LoFactor.msg`). Log them; you don't need them for correctness.
- **`hessian_eigenvalues`, `degenerate_directions`, `inlier_ratio`, `residual_rms`.** For diagnostics.

## 4. Emit modes

- **`request` (recommended for graphs).**
  - Your backend publishes `NodeRequest{stamp = t}` the moment it creates a node at time t.
  - STRIDE deskews the scan whose time span contains t to exactly t, and emits a factor ending there.
  - Requests may arrive after their scan. STRIDE keeps the last `limits.frame_records` (8) accepted scans and answers from them.
  - Requests older than `emit.request_max_age` (1 s) are missed.
  - Publish at node creation, not after optimisation: early requests are answered at the exact time.
  - Launch with `requests:=true`, or set `emit.mode: request` and `topics.node_request`.
- **`overlap` (default).** STRIDE emits whenever overlap with the last emitted frame drops below 90 %. Use this when your backend creates nodes *from* LiDAR factors rather than requesting them.

## 5. Consuming factors in a GTSAM backend

`lo_gtsam_adapter` (C++, depends on GTSAM and `lo_msgs` only) does the conversion:

```cmake
find_package(lo_gtsam_adapter REQUIRED)   # brings lo_msgs and GTSAM
target_link_libraries(my_backend lo_gtsam_adapter::lo_gtsam_adapter)
```

```cpp
#include <lo_gtsam_adapter/adapter.hpp>

lo_gtsam::ChainTracker tracker;                  // one per STRIDE instance
gtsam::Pose3 body_T_base = /* your state frame (e.g. IMU) <- STRIDE base, from your URDF */;

void onFactor(const lo_msgs::msg::LoFactor & f) {
  const auto d = tracker.observe(f);
  if (!lo_gtsam::ChainTracker::usable(d)) return;          // stale or out of order: drop
  // d == NEW_CHAIN: fine. Do NOT link it to the old chain; your IMU factors carry the graph.
  auto ki = keyAt(lo_gtsam::stampNs(f.stamp_i));            // your node lookup, ~1 ms tolerance
  auto kj = keyAt(lo_gtsam::stampNs(f.stamp_j));
  if (!ki || !kj) return;
  if (auto between = lo_gtsam::toBetweenFactor(f, *ki, *kj, body_T_base))
    graph.add(*between);                                    // nullopt => Sigma not SPD: never add
}
```

`toBetweenFactor` takes `body_T_base` for a backend whose state is not the base frame:
- `T_body = E · T_ij · E⁻¹`
- `Σ_body = Ad(E) Σ Ad(E)ᵀ`
- with `E = body_T_base`.

These are covered by `src/lo_gtsam_adapter/test/test_adapter.cpp`.

Working references:
- [`examples/minimal_backend`](../examples/minimal_backend): about 100 lines covering request publishing, chain tracking, node matching and conversion, with no solver. Copy it as the skeleton of a backend plugin. `scripts/check_consumer.sh` builds it out of tree and runs it against a bag.
- [`src/lo_gtsam_adapter/src/lo_graph_demo.cpp`](../src/lo_gtsam_adapter/src/lo_graph_demo.cpp): the same, plus an iSAM2 graph, marginals, and chain-break and solver-failure handling.

Not using GTSAM? Use the message directly: `t_ij` and `covariance` with the right-perturbation, rotation-first convention above. Re-order Σ if your library is translation-first. In Sophus, for example, the tangent is `[t, r]`, so swap the 3×3 blocks.

## 6. Building against stride-lo

stride-lo is a set of ROS 2 Jazzy ament packages. Consumers need `lo_msgs`, plus `lo_gtsam_adapter` if they use GTSAM. Either:
- build stride-lo and source its `install/setup.bash` as an **underlay** before building your workspace (this is what `scripts/check_consumer.sh` checks), or
- add the stride-lo repo to your workspace `src/` (for example as a git submodule) and build everything together.

Dependencies come from the `stride-lo:dev` image (`docker/Dockerfile`): GTSAM 4.3.0, Sophus 1.22, `livox_ros_driver2`, small_gicp (vendored and patched by `scripts/build.sh`; build with it, not with plain `colcon build`).

If your backend links a different GTSAM build, build `lo_gtsam_adapter` against *that* GTSAM. Mixing two GTSAM ABIs in one process is undefined.

## 7. New-robot checklist

1. Copy [`src/lo_ros/config/robot_template.yaml`](../src/lo_ros/config/robot_template.yaml) into your repo and fill in every `REQUIRED` field: topics, frames, extrinsics, body box.
2. Launch it:
   ```bash
   ros2 launch lo_ros stride_lo.launch.py robot:=default config:=/abs/path/my_robot.yaml \
     requests:=true rviz:=false
   ```
3. Keep `scan.time_quantum: 0` and start with `imu.time_offset: 0`. The non-zero values in `anymal_c.yaml` calibrate the GPS-denied *simulator*, not real hardware.
4. Self-filter: give a URDF (`self_filter.robot_description_topic` or `self_filter.urdf_file`) plus joints if the legs are in view. Otherwise set a `leg_box`.
5. Record a bag with ground truth (mocap, RTK or a survey) and measure `imu.time_offset` with `lo_tools/scripts/time_offset.py`. Then tune `cov.kappa` and `degeneracy.lambda_min` per [TUNING.md](TUNING.md) with `bag_suite.py` / `evaluate.py`. Target: mean NEES 4–9 and 0 false accepts.
6. On a Jetson, run `src/lo_ros/scripts/orin_setup.sh` and check timing (median ≤ 25 ms, p99 ≤ 50 ms) in `/diagnostics`.
7. Keep your old odometry (FAST-LIO) behind a launch switch until the A/B on your own bags is done.

## 8. Validation status

Validated (details in [PLAN.md](PLAN.md)):
- **Simulated data.** 25 GPS-denied sim bags (trotting ANYmal C, emulated Mid-360):
  - RPE 0.125 % (FAST-LIO 0.255 % on the same segments);
  - 0.0081 °/m;
  - availability 0.999;
  - **0 false accepts**;
  - mean NEES 2.2. Σ is conservative, so a graph under-weights LO slightly but safely.
- **Synthetic scenarios and faults.** 13 synthetic scenarios and the fault suites give 0 false accepts.
- **Stability.**
  - An 8 h soak keeps memory flat.
  - ASan, UBSan and TSan are clean.
  - Replay is bit-identical.
  - There are 0 heap allocations per scan.
- **Timing.** On a 2-thread x86 laptop: median 10.3 ms, p99 19.0 ms.

**Not yet validated:**
- real Mid-360 data;
- Jetson Orin timing;
- PTP drift, which needs the hardware PTP monitor;
- κ / λ_min on real bags.

Do those (checklist steps 5–6) before relying on STRIDE in the field.
