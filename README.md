# stride-lo

STRIDE: integrity-checked LiDAR odometry for quadruped robots in GPS-denied environments.
Livox Mid-360 in, IMU-free relative-pose factors (`lo_msgs/LoFactor`: T_ij in the base frame,
a 6×6 Σ in GTSAM `[rx ry rz tx ty tz]` order, health, flags) out, for a GTSAM backend.
Continuous-time deskew, coarse-to-fine GICP, Hessian degeneracy handling, NEES-checked
covariance, a health state machine, and industrial hardening (lifecycle, watchdogs, black box,
zero allocations, deterministic replay). ROS 2 Jazzy, CPU-only, Jetson Orin target.

- Spec: [docs/DESIGN.md](docs/DESIGN.md) (from `docs/design/*.pdf`)
- Decisions and interpretations: [docs/DECISIONS.md](docs/DECISIONS.md)
- Plan, status and results: [docs/PLAN.md](docs/PLAN.md)
- How it is built: [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)
- Bringing it up on a robot: [docs/TUNING.md](docs/TUNING.md)

## Results (GPS-denied sim bags, 25 runs, outdoor yard + rough terrain, trotting ANYmal C)

| | STRIDE | FAST-LIO (same segments) |
|---|---|---|
| RPE over 10 m (mean of bags) | **0.125 %** (0.085–0.154 %) | 0.255 % |
| Rotation | **0.0081 °/m** | 0.0230 °/m |
| Availability (graph-node requests answered) | 0.999 | – |
| False accepts (NEES > 40) | **0** | – |
| LOST events | 0 | – |
| Mean NEES (6-DoF) | 2.2 (conservative) | – |
| Timing, x86 laptop, 2 threads | median 10.6 ms, p99 19.8 ms | – |

Evaluated against the bags' 1 kHz ground truth, with node requests at the estimator's keyframe
times. Synthetic suite (13 scenarios incl. the 11 fault cases): 0 false accepts.

## Build and test

The host needs only Docker. Everything runs in the `stride-lo:dev` image (Jazzy, GTSAM 4.3.0,
Livox SDK2 + driver, Sophus, Gazebo Harmonic) through `scripts/dc`. The repo is mounted at its
own path; bags are mounted read-only at `/data/bags` (`STRIDE_BAGS_DIR`).

```bash
scripts/dc build dev
scripts/dc run --rm dev scripts/build.sh                      # Release, -Wall -Werror
scripts/dc run --rm dev scripts/test.sh                       # 62 tests
scripts/dc run --rm dev scripts/build.sh --sanitizer=address  # also undefined | thread
```

## Run

```bash
# on the robot (or with ros2 bag play --clock and use_sim_time:=true)
ros2 launch lo_ros stride_lo.launch.py robot:=anymal_c requests:=true
# a demo GTSAM backend: graph nodes every 100 ms on IMU stamps, answered by STRIDE
ros2 launch lo_gtsam_adapter graph_demo.launch.py imu_topic:=/imu/data
# live synthetic rig, optionally with Gazebo (world, clock and kinematic rig)
ros2 launch lo_sim sim.launch.py scenario:=crowd gazebo:=true lo:=true record:=true
```

Backend side: `lo_gtsam::toBetweenFactor(msg, X(i), X(j), body_T_base)` and
`lo_gtsam::ChainTracker` (never connect factors across a `chain_id` change).

## Evaluate

```bash
ros2 run lo_tools lo_replay --bag BAG --robot anymal_c --out out \
    --requests-from /estimator/odometry_keyframe --request-latency 0.005 --gt /sim/raw/odom
ros2 run lo_tools evaluate.py out --gt /sim/raw/odom --kappa 20
ros2 run lo_tools bag_suite.py results /data/bags/batch_1118/outdoor_yard   # table of a bag set
ros2 run lo_tools live_check.sh BAG live_out 60                             # live node + demo graph
```

## Layout

`src/lo_core` (pipeline, ROS-free), `src/lo_msgs`, `src/lo_ros` (node, supervisor, configs,
launch), `src/lo_gtsam_adapter`, `src/lo_synth` (simulator), `src/lo_tools`, `src/lo_sim`,
`src/third_party/small_gicp` (pristine; patches in `patches/`), `docker/`, `scripts/`, `docs/`.

License: Apache-2.0 (small_gicp: MIT).
