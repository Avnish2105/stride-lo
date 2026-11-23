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
- **Using it in your own stack: [docs/INTEGRATION.md](docs/INTEGRATION.md)**

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

## Integrating into your stack

STRIDE is **not a drop-in FAST-LIO replacement**. It emits relative-pose factors for *your*
factor-graph backend; it does not publish a fused pose (its `~/odometry` is debug only). Wiring:
your backend publishes `lo_msgs/NodeRequest` at each node time and adds each `LoFactor` as a
`BetweenFactor<Pose3>` via `lo_gtsam_adapter`. No backend yet: `lo_graph_demo` gives a pose.
Start from [docs/INTEGRATION.md](docs/INTEGRATION.md), the robot file template
[`src/lo_ros/config/robot_template.yaml`](src/lo_ros/config/robot_template.yaml) and the
out-of-tree consumer [`examples/minimal_backend`](examples/minimal_backend)
(`scripts/check_consumer.sh` builds and runs it against an installed stride-lo).
Validated on simulated data only so far: real Mid-360 tuning and Orin timing are still open.

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

## Demo: buttons in VS Code

Open the repo in the dev container (VS Code: **Reopen in Container**). The status bar then has
five buttons (from `.vscode/tasks.json`, also under *Terminal → Run Task*):

| Button | What it does |
|---|---|
| ▶ Launch | starts the shared screen, opens it in your browser and opens the launcher window there (builds the workspace first if needed) |
| 🔨 Build | `scripts/build.sh` |
| ✓ Test | `scripts/test.sh` |
| 🖥 Screen | opens the shared screen in the browser |
| ■ Stop | stops the running launch (Gazebo, RViz, STRIDE); `scripts/stop_all.sh --screen` also closes the screen |

**The shared screen.** Gazebo, RViz and the launcher window run on a virtual display in the
container (`scripts/screen`: Xvfb `:99` → x11vnc → noVNC). You watch and use it in a browser tab at
http://localhost:6080 (VS Code forwards the port). Claude sees the same screen with
`scripts/screen shot [all|rviz|gazebo|launcher]`. Nothing draws on your real desktop, and no GPU is
needed (Mesa llvmpipe).

The launcher window has every option:

- **Simulation**: scenario (`crowd room warehouse corridor open_field stairs turn180`), seed,
  rate, Gazebo world, Gazebo window, RViz, STRIDE + GTSAM graph demo, record a bag (into `data/`).
  No recorded data is needed: `lo_sim` generates the Mid-360 scans, the IMU and ground truth live.
- **Replay a bag**: pick a bag directory. Robot, ground-truth topic and IMU topic are filled in from
  its `metadata.yaml`. STRIDE runs live on it, and the bag's own recorded STRIDE outputs are not
  replayed.

**Launch** starts everything. **Stop** (in the window or the status bar) stops `ros2 launch`
and the recorder first (SIGINT), so nodes shut down in order and a recorded bag is finalised,
then anything left in the process group (SIGKILL only if it hangs), so nothing keeps running.

What RViz shows (fixed frame `world`, camera follows the robot):

| Layer | Meaning |
|---|---|
| green path | ground truth |
| orange path | STRIDE LiDAR odometry (debug output, aligned to ground truth at the first pose) |
| blue path | GTSAM graph built from STRIDE's factors (`lo_graph_demo`) |
| coloured cloud | STRIDE's local map, coloured by height |
| white points | the current deskewed scan |
| robot model | the rig at STRIDE's pose (legs from `/joint_states`) |
| text above the robot | STRIDE state, factor health, active flags, distance to ground truth |

Gazebo shows the same scene: grey/brown static geometry, blue cylinders = people, orange box
= the rig, camera following it. Gazebo is display only. The rig's clock drives everything, and
Gazebo follows it (DECISIONS I37), so a slow Gazebo window can't disturb STRIDE.

Without the launcher, the same thing by hand (in the container):

```bash
scripts/screen start && export DISPLAY=:99
ros2 launch lo_sim sim.launch.py scenario:=crowd                    # Gazebo + RViz + STRIDE
ros2 launch lo_sim sim.launch.py scenario:=crowd gazebo:=false      # RViz only
```

## Run

```bash
# on the robot (rviz:=false when headless), or with ros2 bag play --clock and use_sim_time:=true
ros2 launch lo_ros stride_lo.launch.py robot:=anymal_c requests:=true
ros2 launch lo_ros stride_lo.launch.py robot:=anymal_c requests:=true rviz:=false
# a demo GTSAM backend: graph nodes every 100 ms on IMU stamps, answered by STRIDE
ros2 launch lo_gtsam_adapter graph_demo.launch.py imu_topic:=/imu/data
# a recorded bag, live: play it with its clock next to the node (RViz on by default)
ros2 bag play BAG --clock &
ros2 launch lo_ros stride_lo.launch.py robot:=anymal_c use_sim_time:=true
# live synthetic rig; defaults: gazebo:=true gui:=true rviz:=true lo:=true
ros2 launch lo_sim sim.launch.py scenario:=crowd record:=true
ros2 launch lo_sim sim.launch.py scenario:=crowd gazebo:=false rviz:=false   # fully headless
```

Launch arguments: `stride_lo.launch.py` takes `robot` (`anymal_c | stride_rig | default`),
`config`, `use_sim_time`, `requests`, `supervisor`, `log_level`, `rviz`, `respawn` (crash
respawn, default true); `sim.launch.py` takes
`scenario`, `seed`, `gazebo`, `gui`, `rviz`, `lo`, `record`, `rate`.

## Evaluate

```bash
ros2 run lo_tools lo_replay --bag BAG --robot anymal_c --out out \
    --requests-from /estimator/odometry_keyframe --request-latency 0.005 --gt /sim/raw/odom
ros2 run lo_tools evaluate.py out --gt /sim/raw/odom --kappa 20
ros2 run lo_tools bag_suite.py results /data/bags/batch_1118/outdoor_yard   # table of a bag set
ros2 run lo_tools live_check.sh BAG live_out 60                             # live node + demo graph
```

## Scripts and tools

| Command | What it does |
|---|---|
| `scripts/dc` | `docker compose` wrapper (host side): mounts the repo at its own path, bags from `STRIDE_BAGS_DIR` at `/data/bags` (read-only), runs as your uid. `dc build dev`, `dc run --rm dev <cmd>`. |
| `scripts/build.sh` | colcon Release build with `-Wall -Werror`; `--sanitizer=address\|undefined\|thread` builds into `build_<san>/`; extra args go to colcon. Applies the small_gicp patches first. |
| `scripts/test.sh` | `colcon test` + results for the Release or a sanitizer build. |
| `scripts/check_consumer.sh [BAG]` | builds `examples/minimal_backend` in a fresh workspace against `install/` (proves the package exports); with a bag, runs it live against `stride_lo` and checks the factors. |
| `scripts/tidy.sh` | clang-tidy over STRIDE's own C++ (needs `compile_commands.json` from a build). |
| `scripts/patch_small_gicp.sh` | copies the pristine `src/third_party/small_gicp` to `.stride_patched/` and applies `patches/small_gicp/*.patch` (called by `build.sh`). |
| `scripts/vendor_small_gicp.sh [tag]` | re-vendors upstream small_gicp (default `v1.0.1`). |
| `scripts/launcher` | ▶ Launch button: shared screen + the launcher window on it (builds first if needed). |
| `scripts/screen` | the shared screen: `start`, `stop`, `status`, `url`, `shot [all\|rviz\|gazebo\|launcher]`. |
| `scripts/stop_all.sh` | ■ Stop button: stops the running launch; `--screen` also closes the screen. |
| `stride_launcher.py` | the launcher window itself. |
| `lo_viz.py` (lo_sim) | RViz helper: aligns LO/graph to ground truth, paths, robot TF, status text. Viz only. |
| `lo_replay` | runs the pipeline over a bag offline, deterministic, writes an output bag + `summary.json`. |
| `lo_synth_bag <scenario> <out>` | writes a synthetic scenario as an MCAP bag with the real drivers' topics. |
| `lo_synth_run suite` | the 13 synthetic scenarios incl. 11 fault cases, no ROS graph needed. |
| `evaluate.py OUT` | RPE, NEES, false accepts, availability against ground truth (`--plot`, `--json`). |
| `bag_suite.py OUT BAG...` | replay + evaluate a set of bags into one `SUITE.md` table. |
| `fault_inject.py IN OUT --fault ...` | IMU dropout, clock jump, PTP drift, point dropout, blocked lens, leg points, vibration. |
| `run_fault_suite.sh [BAG]` | synthetic fault suite, plus every injectable fault on a bag; fails on any false accept. |
| `soak.py --hours H` | long synthetic run; fails if memory grows more than 5 %. |
| `time_offset.py OUT BAG` | estimates `imu.time_offset` without ground truth. |
| `live_check.sh BAG [OUT] [S]` | live end to end: bag play → node + graph demo → record → evaluate. |

`lo_tools` entries run as `ros2 run lo_tools <name>`; `lo_synth_run` is on `PATH` after `source install/setup.bash`.

## Data

Two sources: the synthetic simulator (`lo_synth`, `lo_sim`) and recorded/simulated bags from the
GPS-denied estimator project (25 trotting ANYmal C runs, used for the results above). Bags are
read from `STRIDE_BAGS_DIR`, which defaults to `~/GPS-denied_State_estimation_Qudrapad/rosbags`
if that exists and to `./data` otherwise. `data/` is gitignored local scratch: replay
outputs, `SUITE.md`/`*.eval.json` results of suite runs, sweep and kappa runs, and build/test
logs. Nothing in the repo needs it.

## Layout

`src/lo_core` (pipeline, ROS-free), `src/lo_msgs`, `src/lo_ros` (node, supervisor, configs,
launch), `src/lo_gtsam_adapter`, `src/lo_synth` (simulator), `src/lo_tools`, `src/lo_sim`,
`src/third_party/small_gicp` (pristine; patches in `patches/`), `docker/`, `scripts/`, `docs/`.

License: Apache-2.0 (small_gicp: MIT).
