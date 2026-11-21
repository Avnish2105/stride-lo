# Decisions, interpretations and deviations

The design doc ([DESIGN.md](DESIGN.md)) is the specification. This file records every choice the
doc leaves open, every place it contradicts itself, and every deliberate deviation, with the
reason. If code and this file disagree, one of them is a bug.

Decided on 2026-11-21 with the project owner (Q&A before the first plan).

## Owner decisions

| # | Topic | Decision | Why |
|---|---|---|---|
| D1 | Graph-node requests | `lo_msgs/NodeRequest` input topic. For each requested time, the scan whose span contains it is deskewed to exactly that time and the factor ends there. Without a request topic, keyframes are emitted when overlap with the last emitted frame drops below 90%. | The target backend (`factor_graph_estimator`, GPS-denied repo) creates its own keyframes every 100 ms on Xsens IMU stamps, and plugins add factors at exactly those times. LoFactor stamps must equal node times (doc: "stamp_i = graph node time"). |
| D2 | Scope of integration | This repo only. The GPS-denied repo is not modified. `lo_gtsam_adapter` + `lo_graph_demo` (a node that mimics the backend's keyframing) live here. | Owner choice. Wiring `factor_graph_estimator` (publish pending keyframe times + a LoFactor plugin) is a later, separate change. |
| D3 | Re-INIT after a chain break | Keep the gyro bias and gravity estimate; start a new map from the next scan that passes the checks, even while moving. Only a cold start (boot, crash restart) needs 1 s of standstill. | A walking robot would otherwise get no factors until it stops. Bias still refreshes at every later 1 s standstill. |
| D4 | Build / CI | Standalone workspace: own Dockerfile (`ros:jazzy-ros-base`, multi-arch) with its own GTSAM 4.3.0 build, Livox-SDK2 and `livox_ros_driver2` in an underlay; devcontainer; GitHub Actions. | Owner choice. Same GTSAM commit (`71a25ca3`, tag 4.3.0) and core flags as the GPS-denied workspace so factors behave identically there. |
| D5 | Test data | `lo_synth` (C++ ray-cast simulator, runs in CI), a Python bag fault injector, and a standalone Gazebo Harmonic sensor rig `lo_sim`. | Bags can't live in git; the fault suite must run on every commit. Gazebo scenes add crowd/stairs/180° turn renders. |
| D6 | small_gicp | Pinned upstream (v1.0.1) vendored unmodified in `src/third_party/small_gicp`; fixes as `patches/small_gicp/*.patch`, applied idempotently by `scripts/build.sh` (GPS-denied repo convention). | Upstream `IncrementalVoxelMap` allocates per voxel and its OpenMP reduction is not deterministic: both break hard rules. |
| D7 | Fine map voxel | `0.5·v` (parameter table `voxel.map_ratio = 0.5`), coarse `4·v`. The `1.0·v` registration cloud, with its kNN covariances, is what gets inserted. | The doc says `0.5·v` in S2 and the parameter table but "fine v" in S6. Two of three agree; S4 says the registration cloud's covariances are reused in the map. |
| D8 | Health semantics | `DEGRADED` (1): at least one Hessian-degenerate direction, Σ inflated only on those axes. `LOW_CONFIDENCE` (2): whole-factor inflation — deskew fallback ×4, IMU lost, sensor blocked, factor spans a reject gap, first factor after RECOVERING. The higher value wins. | The doc defines the values but never says when `LOW_CONFIDENCE` is used. |

## Defaults chosen without a question

| # | Topic | Choice | Why |
|---|---|---|---|
| C1 | Deskew / seed IMU | `/livox/imu` (Mid-360 internal, 200 Hz, accel in g → ×9.80665). `/imu/data` (MTi) is a config option. | The doc's own rule: internal IMU when the backend fuses the MTi. `factor_graph_estimator` uses the MTi as its backbone. |
| C2 | Thread pool | OpenMP, fixed team of 2, static schedule, ordered partial sums. | Doc says "TBB or OpenMP (pick one)". TBB isn't in the image, small_gicp supports OpenMP, static scheduling is deterministic. |
| C3 | Sophus | 1.22.10 from source in the image. lo_core tangents are always `[rot, trans]`; Sophus's `[trans, rot]` is wrapped once in `lo_core/types/se3.hpp`. | Rotation-first ordering is one of the doc's "two most common silent bugs". |
| C4 | Extrinsics | From `/tf_static` (URDF) by default, YAML fallback; validated (orthonormal, finite) at configure. | Same source as the backend (no duplicated numbers); YAML for bags without TF. |
| C5 | `chain_id` | Stamp (ns) of the chain's first frame. | Strictly increasing, unique across process restarts (doc: "restart, new chain_id"), deterministic on replay. The doc says "increments"; the adapter only needs equality and monotonicity. |
| C6 | Names | Node `stride_lo`; topics `~/factor`, `~/odometry`, `~/deskewed`, `~/local_map`, `~/node_request`; `/diagnostics`. | |
| C7 | GTSAM options | Same core flags as GPS-denied `colcon.meta` (`POSE3_EXPMAP`, `ROT3_EXPMAP`, `USE_QUATERNIONS`, system Eigen, no march=native, no TBB). `GTSAM_BUILD_UNSTABLE=OFF`. | STRIDE needs no `gtsam_unstable`; saves build time/memory. |
| C8 | Livox driver | Built from a pinned commit in the underlay (not vendored in this repo). | No duplicate package when this repo is dropped into a workspace that already has the driver. |

## Interpretations of ambiguous spec points

| # | Doc text | Implementation |
|---|---|---|
| I1 | "Track the PTP offset and IMU–LiDAR clock skew" | No ROS topic carries the PTP offset. Per stream (LiDAR, IMU) a rolling minimum of (arrival − stamp) estimates latency + clock offset; skew = drift of the LiDAR−IMU difference vs the INIT baseline. Optional `ptp_offset_topic` (`std_msgs/Float64`, seconds) from a PTP monitor. Either > 2 ms → flag `CLOCK_UNSTABLE` and stop emitting. In replay, arrival = bag receive time (deterministic). |
| I2 | Standstill "joint velocities ≈ 0 and LO motion < 1 mm per scan" | Joints: 0.2 s signed block means of q̇ (the rl_physics robot sways while standing). LO: scan-to-scan registration jitter is 2–7 mm with real range noise (measured, synthetic room), so the 1 mm/scan limit is applied to the *mean* motion over the 1 s window: the mean LO pose of the window's second half minus the first half, divided by the scans between them (noise ≈ 0.4 mm/scan, a 1 cm/s creep shows 1 mm/scan). A per-scan gross gate of 20× the limits (2 cm, 1°) restarts the window on real motion. |
| I3 | IMU lost: "plausibility thresholds tighten by 2×" | All four motion-plausibility limits × `imu_lost.plausibility_scale` (0.5). |
| I4 | `σ̂² = Σρ(r_k)/(N−6)`; GM kernel scale τ/3 | `σ̂² = Σ_k w_k r_kᵀ W_k r_k / (N_inl − 6)`; H includes the weights. The Geman-McClure weight uses the **point-to-plane distance** with scale τ/3: on the Euclidean residual it also punished the expected in-plane offset to the voxel mean (weights ≈ 0.05 → fake degeneracy, 6–25 mm/scan error, measured). |
| I5 | Degeneracy test (`λ_k/N < λ_min` on H scaled by L) | Run on the **point-to-plane Hessian** Σ w (nᵀJ)ᵀ(nᵀJ) (what surfaces constrain; each point adds ≤ 1 to λ/N). On the GICP Hessian, the regularised in-plane "information" times long lever arms made yaw over a bare ground plane look observable → 0.8–2.7° errors with tight Σ (false accepts, measured). Step and covariance use the GICP Hessian restricted to the observable subspace; degenerate directions get σ = 1 m / 0.5 rad through the scaled eigenvector; reported as unit 6-vectors in base-frame `[rot, trans]`. Fine-stage updates are projected; degenerate components of the coarse result are reset to the seed first. `λ_min = 0.02` from synthetic spectra: corridor ≤ 0.010, open field 0.000, crowd p01 0.045, room p01 0.09, warehouse p01 0.10 (re-tune on real bags). |
| I6 | "Rotation vs gyro-integrated rotation over the interval" | Interval = previous processed scan's reference time → current reference time; bias-corrected gyro mapped into the LiDAR frame. Only rejects. |
| I7 | "LRU removal of voxels unseen for 30 s" | Unseen = no insert hit for 30 s of message time. When the pool is full, the least recently hit voxel is evicted. |
| I8 | "scan overlap with the last emitted frame drops below 90%" | Share of current registration points (odom frame) that land in **coarse** voxels (coarse_ratio·v) occupied by the last emitted frame. The Mid-360's non-repetitive pattern hits different fine voxels every scan even at standstill, so fine-voxel overlap would trigger every scan. |
| I9 | IMU sample timing | `imu.sample_convention: instant | interval_mean`. Xsens and the GPS-denied sim publish the mean over (t_prev, t] → sample placed at mid-interval. |
| I10 | "False accepts … never yield a tight wrong factor" | An emitted factor whose NEES vs ground truth exceeds 40 (χ²₆ tail probability ≈ 1e-6). |
| I11 | LoFactor definition | Fields as in the doc, except `T_ij` is spelled `t_ij`: ROS 2 IDL requires lowercase field names. Only constants added (health values, flag bits). |
| I12 | Request mode details | A request is answered by the first processed scan whose span (± `request.tolerance`) contains it. Two requests inside one scan: the earlier is answered, the other is counted as missed. In request mode the overlap trigger is off (factors must land on graph nodes). Requests for already-processed scans are counted as late. |
| I13 | Coarse "wide gate" / RECOVERING "wide-gate"; "no match 2 s" | Coarse stage uses `gate.max`; RECOVERING uses `recovery.gate` (2 × `gate.max`) and its coarse blow-up step limit scales with that gate (0.5 m / 10° rejected every recovery attempt, measured). The 2 s window starts at the first match attempt: scans dropped before matching (clock fault) do not use it up. The seed-deviation check is skipped while recovering (seeds span seconds). |
| I14 | Map content | Gaussian voxels (VGICP style): running sum of points and per-point covariances (S4), count capped at `map.max_pts_voxel` (then frozen, like KISS-ICP), oriented normal for the far-side check, last-hit stamp. |
| I15 | Process crash restart | The node runs as its own process with `respawn`; `lo_supervisor` re-drives configure/activate. A restarted process is a cold start (D3) with a new chain_id (C5). |
| I16 | 8 h soak | 8 h of synthetic data processed offline as fast as possible (fits in a 6 h CI job) with RSS sampled; the real-time soak happens on the robot. |
| I17 | KISS-style adaptive gate τ = 3σ_dev | σ_dev from seed-vs-result deviations of scans that moved ≥ 2 cm (`gate.min_motion`; 0.1 m per scan was never reached at 0.8 m/s). Effective gate = max(τ, `gate.voxel_floor` × fine voxel): correspondences go to Gaussian voxel *means*, up to ~a voxel from any point on the same surface (τ at its 0.1 m floor gave 0.3 inlier ratio in a 30 m hall, measured). Fine-level search is 3×3×3 voxels (fine voxels are 0.5·v, D7). |
| I18 | "Converged when \|Δrot\| < 1e-4 rad and \|Δtrans\| < 1e-4 m" | Once steps are below 10× those limits, correspondences are frozen so Gauss-Newton converges on a fixed set (re-searching every iteration chattered between voxels and missed the 1e-4 limit at standstill, measured). |
| I19 | INIT map | Every registered scan during the INIT standstill is inserted (a single-scan map at 0.5·v voxels gave 0.35 inlier ratio, measured) and the map is kept at the end of INIT; the gravity alignment becomes a fixed output transform (T_align) instead of a map rebuild. INIT holds its map with half the tracking inlier threshold. |
| I20 | DEGRADED map insertion "only with ≤ 1 degenerate direction" | Additionally no insertion while DESKEW_FALLBACK / IMU_MISSING: a constant-velocity deskew may leave rotational smear that must not shape the map later scans are measured against (it produced a false accept right after an IMU dropout, measured). |
| I21 | "Σ_ij ≈ Σ_j for consecutive emissions" | Also composed (Σ_i transported + Σ_j) when frame i itself was DEGRADED / LOW_CONFIDENCE: it is not the well-anchored frame the approximation assumes. |
| I22 | "jumps > 50 ms from the expected period" | One period: ±`scan.period_tolerance` (50 ms). n ≥ 2 periods (scans lost in transport): within ±`scan.drop_tolerance` (20 ms) of a multiple. Anything else is a clock jump. (With a 100 ms period a ±50 ms multiple rule would accept every gap.) |

## Deviations from the doc's package list

- `lo_synth` (ROS-free simulator library) and `lo_sim` (Gazebo rig) are added (D5).
- `src/third_party/small_gicp` + `patches/` (D6).
- `lo_gtsam_adapter` also hosts the `lo_graph_demo` node (D2).

## Environment facts the design depends on

- Target backend: `factor_graph_estimator` (GPS-denied repo). IMU-centric (MTi `/imu/data`
  500 Hz in sim), state = IMU frame, keyframes every 100 ms on IMU stamps, ISAM2 fixed-lag 3 s,
  waits up to `max_latency` 0.5 s for plugins, pluginlib sensor plugins. A future STRIDE plugin
  maps base-frame LoFactors into its IMU frame with the URDF extrinsic.
- Sim data: `~/GPS-denied_State_estimation_Qudrapad/rosbags/batch_1118/outdoor_{yard,rough}`
  (25 bags, ANYmal C, rl_physics trot): `/livox/lidar` CustomMsg 10 Hz (20 000 pts, no-returns
  as (0,0,0), synthetic pattern, 50 Hz renders → per-point motion quantised at 20 ms),
  `/livox/imu` 200 Hz in g (frame `livox_frame`), `/imu/data` 500 Hz, `/joint_states` ~1 kHz,
  `/ground_truth/odom` (world→base), `/tf_static`, `/estimator/odometry[_keyframe]`, FAST-LIO
  `/Odometry`. Mount: front, dome forward, 20° down (`base→livox_frame` xyz 0.60 0 0.08, rpy 0
  110° 0) → ~0% self-hits.
- This host: Ubuntu 26.04, no ROS/compiler; everything runs in `stride-lo:dev` via `scripts/dc`.
