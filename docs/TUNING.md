# Tuning STRIDE on a robot

Order matters: timing and extrinsics first, then registration, then covariance. Every step
has a tool, and every change is judged by numbers (CLAUDE.md working style). Most parameters
live in `src/lo_ros/config/<robot>.yaml` (defaults in `default.yaml`; the schema with ranges and
units is `lo_core/config/schema.hpp`).

## 0. Record the bags

Record `/livox/lidar`, the IMU used for deskew, `/joint_states`, `/tf_static`, and the graph
odometry if the backend provides it. Include ground truth where you can (motion capture, a
surveyed loop or a total station). Start every bag with **≥ 2 s of standstill**: cold INIT
needs 1 s. Include fast turns and rough-terrain walking: the timing errors only show there.

## 1. Extrinsics and frames

`extrinsics.source: tf` reads `base → frames.lidar` and `frames.lidar → frames.imu` from
`/tf_static` (URDF). Check that the LiDAR mount's pitch and the lever arm are right: deskew rotates
points about the base origin with that lever arm (DECISIONS I28).

## 2. Clock and time offsets

- PTP: wire a PTP monitor to `topics.ptp_offset` (Float64, s); the limit is `clock.max_offset`
  (2 ms). The arrival-based skew proxy (`clock.check_skew`, `clock.max_skew`) assumes a stable
  transport-latency floor. Measure the floor first (I1): on the GPS-denied sim it wandered by
  13 ms, so the proxy is off there.
- **IMU time offset** (`imu.time_offset`): replay once, then run
  ```bash
  ros2 run lo_tools lo_replay --bag BAG --robot ROBOT --out out
  ros2 run lo_tools time_offset.py out BAG --imu /livox/imu      # LO as reference, no GT needed
  ```
  and set `imu.time_offset` to the printed value. Check `imu.sample_convention`
  (`instant` | `interval_mean`). The tool's minimum must be sharp: ±0.5 ms should change the
  RMS visibly. On the GPS-denied sim it gives −2.25 ms with LO alone, the same as with ground
  truth.
- `scan.time_offset` (LiDAR stamps) should be 0 with PTP. A sweep (`--set scan.time_offset=…`)
  with ground truth shows it.
- `scan.time_quantum` is for simulators that render geometry at a fixed rate. Keep it 0 on
  real sensors.

## 3. Registration and integrity

- Watch `rejects:` in the `lo_replay` summary (`--verbose` prints each scan, with seed deviation
  and step sizes). Healthy bags have 0–1 rejects per few minutes.
- `gicp.freeze_after` (10) of 15: rejects for `not_converged` with high inlier ratios mean
  correspondences oscillate; lower it.
- `degeneracy.lambda_min` (0.02): from the `emin` column (normalised point-to-plane eigenvalue)
  on corridor / open-field bags. Degenerate geometry is below it, normal geometry several times
  above it.
- Graph gate (`seed.graph_max_dev` 0.15 m/s over a ≥ 0.4 s window, `seed.graph_holdoff` 2 s):
  `graph_gated` in `summary.json` should be 0 on bags where the backend is healthy.

## 4. Covariance (κ, floor, weak directions)

Run the bag set with ground truth and read the table:

```bash
ros2 run lo_tools bag_suite.py results BAG_DIR... --robot ROBOT --gt /ground_truth/odom
```

- **False accepts must be 0** (NEES > 40). This outranks NEES calibration (hard rule 2).
- `evaluate.py --kappa K` prints a κ suggestion for mean NEES 6. Lower κ only while false
  accepts stay at 0. On the sim bags they rose as soon as κ dropped (DECISIONS I32).
- `cov.floor_*` (spec: 0.05°, 5 mm) protects the tail. Change it only with real ground truth.
- `cov.weak_lambda` / `cov.weak_sigma` add variance along weakly observed directions (I31). If
  false accepts sit in feature-poor scans with `emin` just above `lambda_min`, raise
  `weak_sigma`.

## 5. Jetson Orin

`src/lo_ros/scripts/orin_setup.sh` (nvpmodel MAXN, jetson_clocks, performance governor).
Pin the node with `runtime.cpu_affinity` and optionally `runtime.sched_fifo_priority`. Check
`/diagnostics` timing: median ≤ 25 ms, p99 ≤ 50 ms (x86 laptop reference: 10.6 / 19.8 ms).
