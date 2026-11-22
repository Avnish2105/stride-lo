# Implementation plan and status

Approved 2026-11-21. Updated at every phase commit. Spec: [DESIGN.md](DESIGN.md);
decisions/interpretations: [DECISIONS.md](DECISIONS.md).

## Status

| Phase | Scope | Gate | Status |
|---|---|---|---|
| 0 | Context docs, Docker image, workspace, vendored small_gicp + patches, lo_msgs, skeletons, scripts | Image builds, workspace builds, unit tests run | done (CI workflows land with phase 4) |
| 1 | Core LO: S0, S2, S5, S6, S8, lo_replay, lo_synth basics | No divergence, RPE ≤ 2% (synthetic room, outdoor_yard bags) | done (2026-11-22) |
| 2 | Deskew + self-filter: S1, S3, S4, bias estimator, URDF kinematic model | RPE ≤ 1% on trotting bags | done (2026-11-22): all 25 bags ≤ 0.154% |
| 3 | Integrity, covariance, LoFactor, adapter, demo graph, evaluate.py, κ/λ_min tuning | Mean NEES ∈ [4, 9], 0 false accepts | done with a recorded deviation (2026-11-22): 0 false accepts on 25 bags, mean NEES 2.2 (conservative; band not reachable together with 0 false accepts on sim data, I32) |
| 4 | State machine, watchdogs, black box, lifecycle + supervisor, fault injector + suite, sanitizers, soak | Fault suite green, memory flat ±5% | pending |
| 5 | lo_sim Gazebo rig + scenes, Orin tooling, TUNING.md | Full acceptance table on x86 (Orin timing reported only) | pending |

## Results log

(numbers per phase: RPE, NEES, availability, timing, false accepts)

- **Phase 0** (2026-11-22): `stride-lo:dev` image builds (6.75 GB: Jazzy, GTSAM 4.3.0, Livox SDK2 +
  driver, Sophus 1.22.10, Gazebo Harmonic). small_gicp v1.0.1 vendored, 4 patches apply.
  `lo_msgs` builds. Note: `LoFactor.T_ij` is spelled `t_ij` (ROS 2 lowercase field rule, I11).
- **Phases 1–2** (2026-11-22). lo_core S0–S9 + state machine + covariance + emitter, lo_synth,
  lo_ros node, lo_tools (replay, evaluate, fault injector, bag suite, soak). 48 unit/synthetic
  tests green (zero allocations after warm-up, bit-identical determinism).
  - Synthetic (13 scenarios, 0 false accepts): room RPE 0.097 %, warehouse 0.069 %, crowd
    0.090 %, turn180 0.062 %, stairs 0.081 %; corridor / open field DEGRADED by design.
  - GPS-denied sim bags, 25 runs (`bag_suite.py`, requests = keyframe stamps + 5 ms, NEES vs
    1 kHz truth, κ = 20). Progression of the same suite while fixing (DECISIONS I1, I23–I30):

    | suite | RPE mean of bags | deg/m | mean NEES | false accepts | availability | LOSTs |
    |---|---|---|---|---|---|---|
    | v1 first run | 1.165 % | 0.0153 | 6.30 | 547 | 0.925 | 142 |
    | v2 sim calibration, graph gate, late requests | 0.191 % | 0.0085 | 4.09 | 158 | 0.994 | 1 |
    | **v3** lever-arm deskew, velocity baselines, freeze | **0.125 %** | **0.0081** | 3.06 | 1 | 0.999 | 0 |
    | FAST-LIO on the same segments | 0.255 % | 0.0230 | – | – | – | – |

    v3 per bag: RPE 0.085–0.154 %, availability 0.999, 0–1 rejects, 0 LOSTs; yard NEES 2.8–3.1,
    rough 2.5–4.3 (Σ conservative: κ tuning is phase 3). Timing on the x86 laptop (2 threads):
    median 10.1 ms, p99 18.5 ms (Orin to be measured).
- **Phase 3** (2026-11-22). lo_gtsam_adapter (LoFactor → BetweenFactor<Pose3>, body-frame change,
  ChainTracker) with GTSAM tests (marginal = Σ, Mahalanobis of the right perturbation, adjoint =
  lo_core's), lo_graph_demo (NodeRequests every 100 ms on IMU stamps, iSAM2). Covariance:
  weak-direction term (I31).
  - κ sweep, 6 bags (weak term on): κ 20/8/5 → NEES 2.6/3.2/3.4, false accepts 0/3/3; half floor,
    κ 20/10 → NEES 5.6/7.4, false accepts 20/60. Kept κ = 20 + spec floor (I32).
  - 25-bag suite v4: RPE 0.125 %, 0.0081 deg/m, availability 0.999, **0 false accepts**, mean
    NEES 2.22 (1.74–3.36), t_med 10.6 ms / p99 19.8 ms.
  - Live: `live_check.sh` (ros2 bag play --clock → lifecycle node + supervisor ↔ lo_graph_demo),
    yard 1, 60 s: RPE 0.175 %, NEES 2.29, 0 false accepts, all factors on graph nodes.

---

# Approved plan (verbatim)

## Context
This repo, `stride-lo`, currently holds only a README, an Apache-2.0 LICENSE and `.gitignore`. It must become a complete ROS 2 Jazzy implementation of *LiDAR Odometry Stack — Final Design* (Nov 21 2026).

The stack takes Livox Mid-360 scans and emits an IMU-free `LoFactor` (T_ij in the base frame, a 6×6 Σ in GTSAM `[rx ry rz tx ty tz]` order, and a health value) for a GTSAM backend. The design's commitments:
- It is never confidently wrong.
- Its covariance is NEES-calibrated.
- It runs CPU-only on 2 cores, with 0 per-scan allocations and bit-identical replay.

The GPS-denied repo (`~/GPS-denied_State_estimation_Qudrapad`) is used **read-only** and is not modified:
- `factor_graph_estimator` is the future consumer. It is IMU-centric (MTi `/imu/data`), uses GTSAM 4.3 and pluginlib sensor plugins, and creates keyframes every 100 ms on IMU stamps.
- 25 sim bags with all STRIDE inputs plus ground truth: `rosbags/batch_1118/outdoor_{yard,rough}`. They contain `/livox/lidar`, `/livox/imu`, `/imu/data`, `/joint_states`, `/ground_truth/odom`, `/tf_static`, `/estimator/odometry[_keyframe]` and FAST-LIO's `/Odometry`.

This host runs Ubuntu 26.04 with no ROS and no compiler, so everything is built and tested in Docker.

## Decisions
| Topic | Decision (from your answers) |
|---|---|
| Graph-node requests | New `lo_msgs/NodeRequest` topic in this repo. The scan whose time span contains a requested time is deskewed to that time and emitted. With no request topic configured, overlap mode applies: emit when overlap drops below 90%. The GPS-denied repo is untouched. An adapter and a demo graph node live here. |
| Re-INIT after a chain break | Keep the gyro bias and gravity estimate and start a new map immediately, even while moving. Only a cold start (boot or crash restart) needs 1 s of standstill. |
| Build / CI | Standalone. Own multi-stage Dockerfile on multi-arch `ros:jazzy-ros-base`. It includes its own GTSAM 4.3.0 (same commit `71a25ca3` and flags as the GPS-denied `colcon.meta`), plus Livox-SDK2 and `livox_ros_driver2`, built as an underlay. Also a devcontainer and GitHub Actions. |
| Test data | `lo_synth` C++ simulator (used by CI), a Python bag fault injector, and a standalone Gazebo sensor rig `lo_sim`, all in this repo. |
| small_gicp | Pinned upstream copied in unmodified. `patches/small_gicp/*.patch` are applied idempotently by `scripts/build.sh`, the same convention as the GPS-denied repo. |
| Fine map voxel | 0.5·v (parameter table). Coarse map at 4v. The 1.0·v registration cloud, with its kNN covariances, is what gets inserted. |
| Health | DEGRADED: at least one degenerate direction, Σ inflated only on those axes. LOW_CONFIDENCE: inflation of the whole factor (deskew fallback, IMU lost, sensor blocked, factor spans a reject gap, first factor after RECOVERING). If both apply, the higher value wins. |

**Defaults I chose (no question needed):**
- **Deskew/seed IMU:** `/livox/imu` by default. The doc's rule is to use the internal IMU when the backend fuses the MTi, and it does. Accel is scaled by 9.80665 because the driver publishes in g. `/imu/data` stays selectable by config.
- **Thread pool:** OpenMP with a fixed team of 2 and static scheduling. TBB isn't installed, small_gicp supports OpenMP natively, and static scheduling is deterministic.
- **Sophus:** apt `libsophus-dev`, or a pinned header-only install in the image if apt lacks it.
  - All lo_core tangents are `[rot, trans]`.
  - Sophus uses `[trans, rot]` internally, so it is wrapped once in `lo_core/types/se3.hpp` and never exposed.
- **Extrinsics:** read from `/tf_static` (URDF), with a YAML fallback, validated at configure.
- **chain_id:** the stamp (ns) of the chain's first frame. It strictly increases, stays unique across process restarts and is deterministic on replay.
- **Names:** node `stride_lo`. Topics `~/factor`, `~/odometry` (debug), `~/deskewed`, `~/local_map`, `~/node_request` (optional input), plus `/diagnostics`.

## Spec interpretations (veto at approval)
1. **PTP offset.** No ROS topic carries it.
   - Clock health is estimated per stream as a rolling minimum of (arrival − stamp), for LiDAR and IMU.
   - Skew is the drift of their difference relative to the INIT baseline.
   - An optional `ptp_offset_topic` (Float64) can also be wired in.
   - Above 2 ms, the stack sets a flag and stops emitting.
2. **Standstill.** Uses 0.2 s block means of joint velocity plus LO motion < 1 mm per scan. Block means are needed because the rl_physics robot sways while standing.
3. **IMU lost.** "Tighten by 2×" is applied to all four motion-plausibility limits (`imu_lost.plausibility_scale: 0.5`).
4. **Covariance scale.** σ̂² = Σ wₖ rₖᵀWₖrₖ / (N_inliers − 6). Geman-McClure weights come from the Euclidean residual with scale τ/3, and H includes those weights.
5. **Degenerate directions.**
   - σ = 1 m / 0.5 rad, combined through the scaled eigenvector.
   - Reported as unit 6-vectors in base-frame `[rot, trans]`.
   - Fine-stage updates are projected onto the observable subspace.
   - The degenerate components of the coarse result are reset to the seed before the fine stage, so the value along those axes equals the seed.
6. **Gyro check interval.** From the previous processed scan's reference time to the current one. The check can only reject, never change the value.
7. **Map eviction.** "Unseen for 30 s" means no insert has hit the voxel for 30 s of message time. LRU also evicts when the pool is full.
8. **Overlap.** The share of current registration points (in the odom frame) that land in fine voxels occupied by the last emitted frame.
9. **IMU timing.** Parameter `instant | interval_mean`. The Xsens and sim convention is a mean over (t_prev, t], so those samples are placed at mid-interval.
10. **False accept** (fault suite): an emitted factor whose NEES against ground truth exceeds 40, the χ²₆ tail at 1e-6.
11. **LoFactor fields** stay exactly as in the doc. Only constants are added (health values and flag bits such as IMU_MISSING, DESKEW_FALLBACK and SEED_FALLBACK).

## Where Claude runs, and how context survives containers
- **I never "reopen in container".** This session stays on the host: VS Code, the Claude extension, `~/.claude` (this plan, memory, login).
  - I edit files on the host.
  - Every build, test and replay runs via `docker compose -f docker/compose.yml run --rm dev <cmd>` from the host shell; you're in the `docker` group, so no sudo is needed.
  - The repo is bind-mounted at the **same absolute path** as on the host, so paths in logs and `compile_commands.json` match your editor.
- **The container user is `dev` with uid/gid 1000.** No root-owned `build/`, `install/` or `log/` (the GPS-denied repo has that problem).
- **The context lives in git**, so any later session (host, devcontainer, another machine) starts with everything:
  - `docs/design/LiDAR_Odometry_Stack_Final_Design.pdf`: copied from `~/Downloads`.
  - `docs/DESIGN.md`: a faithful markdown transcription of it, easy to search and diff.
  - `docs/DECISIONS.md`: every answer you gave plus the interpretations below.
  - `docs/PLAN.md`: this plan with a phase-status table, updated at each phase commit.
  - `CLAUDE.md`: the hard rules, pointing to all of the above.
  - After approval I also save project memory entries on the host.
- **Devcontainer, in case you open it later:**
  - `anthropic.claude-code` is in its extension list.
  - The workspace is mounted at the same host path, so Claude's per-project memory key matches.
  - Optional bind mounts of host `~/.claude` and `~/.claude.json` bring logins, plans and memory along.
  - Bags are mounted read-only at `/data/bags`, from `STRIDE_BAGS_DIR`; the default is the GPS-denied `rosbags/`.

## Repository layout (colcon workspace root)
```
src/third_party/small_gicp/  pinned upstream, unmodified (MIT)
src/lo_core/        plain CMake, ROS-free C++17: Eigen, Sophus, small_gicp, OpenMP
  include/lo_core/{types,preprocess,deskew,registration,map,integrity,state,emit,config}/ src/ test/
src/lo_synth/       plain CMake, ROS-free: scenes, ray caster, Mid-360-like pattern, trajectories,
                    IMU/joint synthesis, faults (Eigen, yaml-cpp)
src/lo_msgs/        LoFactor.msg (as in the doc + constants), NodeRequest.msg
src/lo_ros/         lifecycle component node, supervisor, conversions, diagnostics, black box,
                    launch/, config/{default,anymal_c,stride_rig}.yaml, rviz/, scripts/orin_setup.sh
src/lo_gtsam_adapter/  LoFactor -> BetweenFactor<Pose3>, chain tracker, lo_graph_demo, tests
src/lo_tools/       lo_replay, lo_synth_run, fault_inject.py, evaluate.py, fault suite, soak
src/lo_sim/         Gazebo Harmonic rig: worlds, xacro, gz system plugin, emulator nodes, launch
patches/small_gicp/*.patch + patches/README.md
scripts/{build.sh,test.sh,vendor_small_gicp.sh}
docker/{Dockerfile (targets deps|ci|dev), compose.yml}   .devcontainer/devcontainer.json   colcon.meta
.github/workflows/{ci,sanitizers,soak}.yml   .clang-format  .clang-tidy
docs/design/*.pdf   docs/{DESIGN,DECISIONS,PLAN,ARCHITECTURE,SMALL_GICP,TUNING}.md
CLAUDE.md (hard rules + pointers)   README.md
```
`lo_synth` and `lo_sim` are additions to the doc's package list. `lo_synth` is ROS-free so that `lo_core` tests, `lo_tools` and `lo_sim` can share it without depending on each other.

## lo_core: the pipeline
- **API.** One `lo::Pipeline`, fully preallocated at construction.
  - Inputs: `pushImu`, `pushJoints`, `pushGraphOdom`, `pushNodeRequest`.
  - `coverageReady(scan)` says whether the scan can be processed yet.
  - `process(scan)` returns a `const ScanResult&`: verdict, health state, an optional `FactorOut`, the debug pose and per-stage timings.
  - Inputs are neutral structs; the ROS layer converts messages into them.
- **Config schema** (`config/`): one table of name, type, default, range and setter.
  - `lo_ros` declares its ROS parameters from this table.
  - `lo_tools` loads the same YAML through it.
  - The config hash is an FNV-1a over the sorted values, so both give the same hash.

**Stages:**
- **S0 validate.** Rejects:
  - backwards stamps;
  - a jump > 50 ms from the expected period;
  - fewer than 3 000 valid points;
  - point offsets outside [0, 120 ms].

  Also runs the clock-health tracker and a 2 s seqlock SPSC IMU ring, and waits until IMU samples cover the whole scan.
- **S1 self-filter.**
  - Static body crop box.
  - A `KinematicModel` (built from the URDF by `lo_ros`) runs FK at about 11 time samples across the scan, and capsule endpoints are interpolated to each point's time, inflated by 3 cm. Only points inside a body sphere are tested.
  - Blockage monitor: more than 30% of returns within 0.5 m sets `SENSOR_BLOCKED`.
  - If joint states are lost, a conservative static leg-exclusion box is used instead.
- **S2.**
  - Range gate [0.3, 60] m; NaN and zero-reflectivity points dropped; optional Livox noise-tag mask.
  - Adaptive base voxel v = clamp(0.1 · EMA(median range), 0.1, 0.5). The median uses `nth_element` on a preallocated buffer.
  - Voxel downsample at 1.0·v gives the registration cloud.
- **S3 deskew.**
  - Bias-corrected gyro, constant angular acceleration between IMU samples.
  - Translation from graph velocity (with the lever arm to the LiDAR), otherwise LO's last velocity.
  - Reference time is the requested node time, otherwise the scan end.
  - An IMU gap > 20 ms sets `DESKEW_FALLBACK`.
- **S4.** kNN with k = 10 using small_gicp's KdTree, rebuilt in place (patched). Covariances are plane-regularised to (1, 1, 1e-3), with view-oriented normals, and stored with the points for reuse in the map.
- **S5 seed.** Gyro rotation plus graph translation, falling back to LO velocity, then identity. The source is recorded in `flags`.
- **S6 registration.**
  - **Coarse:** 5 iterations against the 4v Gaussian voxels, with a wide gate (`gate.max`).
  - **Fine:** up to 15 projected Gauss-Newton iterations against the 0.5·v map.
    - KISS-style adaptive gate τ = 3σ_dev, starting at 1.0 m, clamped to [0.1, 2.0] m.
    - Geman-McClure kernel with scale τ/3.
    - Far-side-of-surface rejection via normal dot product.
    - Converged at |Δrot| < 1e-4 rad and |Δtrans| < 1e-4 m.
  - Built from small_gicp's GICP factor math with a deterministic reduction.
- **S7 integrity and degeneracy.**
  - Every check in the doc's integrity table produces ACCEPT, DEGRADED or REJECT.
  - Degeneracy: scale the rotation block by the median range L, eigen-decompose, and flag λ/N < λ_min (at most 3 directions reported).
  - Covariance: Σ = κσ̂²H⁻¹ + Σ_floor, then:
    - degenerate directions replaced;
    - ×4 on deskew fallback;
    - Σ_i + Σ_j (with adjoint transport) across a reject gap;
    - transformed to the base frame with Ad(T_base_lidar) in rotation-first order.
- **S8 map.**
  - Two fixed-pool Gaussian voxel maps.
  - Insert only accepted scans; a DEGRADED scan is inserted only with at most 1 bad axis.
  - Insert only after ≥ 0.1 m or 2° of motion since the last insert.
  - At most 20 contributions per voxel, 30 s time-to-live, 80 m radius crop.
- **S9 emit.** On a node request or an overlap trigger, emit T_ij between consecutive emitted frames, with chain_id and flags. Standstill evidence goes to the bias estimator; timings go to diagnostics.
- **State machine.** INIT, TRACKING, DEGRADED, LOST and RECOVERING with the 8 transitions in the doc's diagram, plus the re-INIT decision above. Five consecutive REJECTs move to LOST. RECOVERING first tries a wide-gate match against the kept map.

**Engineering rules:**
- Every buffer is sized from `limits.*`.
- Map pools are sized so the process stays under 500 MB.
- Status codes instead of exceptions in the hot path.
- Message time only in decisions; wall clock is used for timing statistics only.
- Static OpenMP scheduling with ordered partial sums.

## Other packages
- **lo_ros**
  - `lo_ros::LoNode`: a lifecycle node, registered as a component and launched as its own process with respawn.
    - **configure:** validate the schema (ranges, orthonormal extrinsics, files present), read extrinsics, load the URDF (`/robot_description` or a file), build the Pipeline, create subscriptions, start the worker thread. Worker threads can be pinned and optionally run with SCHED_FIFO.
    - **activate:** refused until INIT has finished.
  - Threading: the executor is the I/O thread. The scan slot holds one scan (newest wins). The worker plus one more OpenMP thread form the pool of 2.
  - Watchdogs follow the doc's hardening table.
  - `/diagnostics` at 1 Hz: state, chain, per-stage timing (median and p99), counts, reject reasons, availability, clock offsets, RSS, map fill.
  - Black box: 20 s rings of message pointers, written to MCAP via rosbag2 on LOST or ERROR, rate-limited. Stamped with the config hash and the git SHA (embedded at build time).
  - `lo_supervisor`: drives configure and activate (retrying until INIT is done), and drives them again after a respawn.
  - Configs: `config/anymal_c.yaml` for the GPS-denied bags; `stride_rig.yaml` for `lo_sim`. A kinematic-only ANYmal C URDF is generated once from the GPS-denied xacro, with meshes stripped and the BSD license noted, so S1 can run on those bags.
  - `scripts/orin_setup.sh`: nvpmodel, jetson_clocks, CPU governor.
- **lo_gtsam_adapter**
  - `toBetweenFactor(msg, key_i, key_j, body_T_base = I)` returns a Gaussian::Covariance factor, with an optional adjoint frame change.
  - `ChainTracker` drops a factor when chain_id differs between i and j.
  - `lo_graph_demo` mimics factor_graph_estimator: it publishes a NodeRequest every 100 ms on an IMU topic's stamps, builds iSAM2 from incoming LoFactors, and publishes the path and marginals.
  - Tests: a synthetic H written through LoFactor and the adapter gives Marginals equal to Σ; rotation-first ordering and right perturbation are checked numerically; `Pose3::AdjointMap` is checked against lo_core's adjoint.
- **lo_tools**
  - `lo_replay` (C++): reads MCAP and feeds lo_core in recorded order, single-threaded I/O, so output is bit-identical.
    - Requests come either from a topic (for example `/estimator/odometry_keyframe`, the backend's real node times) or are synthesized from IMU stamps.
    - Writes an output MCAP, a summary JSON and the SHA-256 of the output.
  - `fault_inject.py` (bags): IMU dropout or gap, clock jump, PTP offset drift, 50% point dropout, lens blocked, leg points, jitter.
  - `evaluate.py`: RPE in % over 10 m segments, rotation error in °/m, mean NEES with confidence interval, availability, false accepts, recovery time, timing, and a FAST-LIO comparison when present. Also suggests κ and writes a markdown report with plots.
  - Fault-suite runner and soak runner (8 h of synthetic data, with RSS sampled).
- **lo_synth**
  - Scenes from planes, boxes, cylinders, stairs and moving crowd objects, with analytic ray casting.
  - Mid-360-like non-repetitive pattern: 200 kHz, 4 lines, −7° to +52°, with each point's exact time and sensor pose.
  - Splined trajectories with analytic angular rate and specific force, turned into IMU data with noise, bias, random walk and vibration.
  - Trot-like joint trajectories and leg capsules.
  - All 11 doc scenarios, each with a fixed seed.
- **lo_sim** (Gazebo Harmonic, local only because it needs rendering)
  - A kinematic rig: a ~0.5 m body with 4 legs of 3 joints each.
    - The Mid-360 mount is a launch argument; default is front-top, dome up, pitched 20° down.
    - Carries a Livox IMU and an MTi-like IMU.
  - A gz-sim system plugin drives it exactly from a scenario YAML: rig pose, leg joints and crowd primitives.
  - `gpu_lidar` at 50 Hz feeds `mid360_emulator`. It uses the `lo_synth` pattern and compensates each point with the known trajectory, so per-point motion matches its timestamp. Output is CustomMsg.
  - IMU, ground truth, `/joint_states` and `/robot_description` come from the same trajectory.
  - Scenes: room, corridor, open_field, stairs, crowd, turn180.
  - `sim.launch.py scenario:= record:= lo:=`.

## small_gicp patches
1. `0001`: fixed-capacity incremental voxel map: preallocated pool, open-addressing hash, time-based LRU, max points per voxel, radius crop, ordered iteration.
2. `0002`: deterministic OpenMP reduction: static chunks, per-thread partial sums combined in a fixed order.
3. `0003`: KdTree rebuild in place without reallocating.
4. `0004`: Geman-McClure robust kernel.

`lo_core`'s CMake stops with an error if the patch marker header is missing, so an unpatched build can't go unnoticed. `scripts/vendor_small_gicp.sh <tag>` re-vendors upstream and checks that the patches still apply.

## Docker / CI
- **`docker/Dockerfile`**
  - **`deps`:** toolchain, clang-tidy, Eigen, Sophus, gtest, rosbag2 with MCAP storage, `diagnostic_updater`, `urdf`, tf2, numpy, scipy, matplotlib. Plus an underlay in `/opt/stride_underlay` with GTSAM 4.3.0, Livox-SDK2 and `livox_ros_driver2`, all pinned. No `-march=native`.
  - **`ci`:** `deps` as is.
  - **`dev`:** adds `ros-jazzy-ros-gz`, rviz2 and gdb.
  - Every target has a non-root `dev` user (uid/gid 1000).
- **`docker/compose.yml`** (used by me from the host and by the devcontainer): repo at the same absolute path, `STRIDE_BAGS_DIR` mounted read-only at `/data/bags`, host network and IPC.
- **`.devcontainer`** uses the `dev` target, with the Claude extension and the optional `~/.claude` mounts described above.
- **GitHub Actions**
  - `ci.yml`: cached image build; Release build with `-Wall -Werror`; all tests; fault suite; determinism check; clang-tidy; small_gicp patch check.
  - `sanitizers.yml`: ASan, UBSan and TSan build matrix.
  - `soak.yml`: run manually or nightly; processes 8 h of synthetic data, which fits inside the 6 h runner limit.

## Execution order
I'll work on branch `feat/stride-lo`, make one local commit per phase once its gate passes, and never push.
0. **Scaffold.**
   - Context files first: `docs/design/` PDF, `DESIGN.md`, `DECISIONS.md`, `PLAN.md`, `CLAUDE.md`, plus host memory entries.
   - Docker image and compose built; workspace; vendored small_gicp plus patches; `lo_msgs`; package skeletons; `build.sh`; CI files.
1. **Core LO** (S0, S2, S5, S6, S8), plus `lo_replay` and basic `lo_synth`. **Gate:** no divergence and RPE ≤ 2% on the synthetic room and the outdoor_yard bags.
2. **Deskew and self-filter** (S1, S3, S4, bias estimator, URDF kinematic model). **Gate:** RPE ≤ 1% on the trotting bags (outdoor_yard and outdoor_rough).
3. **Integrity, covariance, LoFactor, adapter, demo graph, `evaluate.py`.** Tune κ and λ_min on the synthetic corridor and open field and on the bags. **Gate:** mean NEES in [4, 9] and 0 false accepts.
4. **Hardening:** state machine, watchdogs, black box, lifecycle and supervisor, fault injector, fault suite, sanitizers, soak. **Gate:** fault suite green and memory flat within ±5%.
5. **Gazebo and deployment:** `lo_sim` rig and scenes, Orin tooling, `TUNING.md`. **Gate:** the full acceptance table on x86, except Orin timing. I'll report x86 timing; Orin timing can't be measured here.

After each phase I report before/after numbers.

## Verification
- **Build** (all run from the host shell):
  1. `docker compose -f docker/compose.yml build dev`
  2. `docker compose -f docker/compose.yml run --rm dev scripts/build.sh` for Release.
  3. The same with `--sanitizer=address|undefined|thread` for the sanitizer builds.
  4. Every command below is run the same way, through `... run --rm dev <cmd>`.
- **Unit tests:** `scripts/test.sh` (runs `colcon test` and `colcon test-result --verbose`). Coverage:
  - deskew against analytic motion;
  - GICP Jacobian against numeric derivatives;
  - SE(3) ordering and perturbation side;
  - degeneracy on a corridor (1 axis) and an open plane (x, y, yaw);
  - every state-machine transition;
  - each integrity reject path;
  - map TTL, capacity and determinism;
  - an allocation counter that must stay at 0 after warm-up;
  - running twice gives identical bytes.
- **Fault suite:** `ros2 run lo_tools run_fault_suite` runs all 11 synthetic scenarios against the acceptance table.
- **Bags** (GPS-denied `rosbags/batch_1118`, mounted read-only):
  1. `lo_replay --config anymal_c.yaml --requests-from /estimator/odometry_keyframe`
  2. `evaluate.py` for RPE, NEES and availability against ground truth and against FAST-LIO.
- **Live:**
  1. `ros2 bag play --clock`
  2. `ros2 launch lo_ros stride_lo.launch.py robot:=anymal_c use_sim_time:=true`
  3. `lo_graph_demo`, then check `/diagnostics` and `/stride_lo/factor`.
- **Gazebo:** `ros2 launch lo_sim sim.launch.py scenario:=crowd record:=true lo:=true`.

## Limits
- There's no Orin or real robot here. Orin timing and the final κ and λ_min on real bags are phase-5 work for you; the tools and scripts will be ready.
- The GPS-denied bags quantize motion to 20 ms renders, so the deskew gate on those bags is approximate. The synthetic sim and `lo_sim` are exact.
- INIT standstill thresholds may need tuning on rl_physics bags, where the robot sways while standing.
- The 8 h soak runs on synthetic data, about 1–1.5 h of wall time. A real-time soak on the robot is still needed.
