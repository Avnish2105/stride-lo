# LiDAR Odometry Stack — Final Design

Nov 21, 2026 · @Avnish

> Faithful markdown transcription of [`design/LiDAR_Odometry_Stack_Final_Design.pdf`](design/LiDAR_Odometry_Stack_Final_Design.pdf)
> (the PDF is authoritative). Where the doc is ambiguous or self-contradictory, the resolution
> used by the implementation is recorded in [DECISIONS.md](DECISIONS.md), not here.

## Purpose and output contract

The stack turns Livox Mid-360 scans into relative-pose measurements, `T_ij ∈ SE(3)` between two
timestamps plus a 6×6 covariance and a health flag, for a downstream factor graph. It is a
measurement generator, not an estimator: it never fuses, filters or smooths with another sensor.

The contract the backend can rely on:

1. **No IMU information in the factor.** The IMU is used only to deskew points and to seed the
   registration. No IMU term enters the registration cost, and the local map is built only from
   LO's own poses. The backend can add IMU factors, or none, without double counting.
2. **Works with an IMU-free backend.** Gyro bias, gravity alignment and seeding are owned by the
   stack. Nothing it needs comes from the graph's IMU state.
3. **Never confidently wrong.** A scan that fails any integrity check produces no factor, or a
   factor whose covariance is inflated in the directions it cannot see. Missing factors are
   acceptable; wrong factors with tight covariance are not.
4. **Honest covariance.** Σ is calibrated against ground truth (NEES), expressed in GTSAM `Pose3`
   tangent order `[rx ry rz tx ty tz]`, with right (body-frame) perturbation.
5. **Relative, not absolute.** The factor is a `BetweenFactor<Pose3>` in the robot base frame.
   Drift in LO's own odom frame never reaches the graph.

## Design priorities and budgets

When two goals conflict, they rank in this order: **accuracy, then integrity, then compute.**
Lightweight is a hard budget the design must fit inside, not a goal traded against accuracy.

- **Accuracy:** GICP with per-point covariances, two-level coarse-to-fine registration,
  continuous-time deskew. These are the most accurate choices that still fit the budget.
- **Integrity:** every scan passes input, solver and plausibility checks before it can emit a
  factor or touch the map.
- **Lightweight:** CPU-only, no PCL or GTSAM in the hot path, preallocated buffers, bounded map.
  The Orin GPU stays free for perception and ML.

| Budget (Jetson AGX Orin 32 GB) | Target |
|---|---|
| Scan rate | 10 Hz |
| Processing time per scan, median | ≤ 25 ms |
| Processing time per scan, p99 | ≤ 50 ms |
| Latency, scan end → factor published | ≤ 60 ms |
| CPU cores | ≤ 2 (worker pool of 2 threads + 1 I/O thread) |
| Resident memory | ≤ 500 MB, flat over a 2 h run |
| Allocations in the per-scan path | 0 after warm-up |

These are design targets, to be confirmed by profiling on the Orin.

## Architecture

*IMU enters only at deskew and seed; the factor is LiDAR-only.* Per-scan pipeline at 10 Hz,
CPU-only. LO pipeline · 10 stages, 4 inputs, 1 output.

```
Mid-360 scans (10 Hz, per-point time) ──► S0 · Validate + time (stamps, PTP offset, point count)
Joint states (leg FK, standstill)    ──► S1 · Leg self-filter (FK leg capsules + body crop)
                                          S2 · Range gate + voxel (adaptive size from spaciousness)
Gyro (IMU) ≥ 200 Hz, deskew + seed   ──► S3 · Continuous-time deskew (gyro, constant angular accel)
                                          S4 · Per-point covariances (kNN once, reused in the map)
Graph odometry (optional, transl.)   ──► S5 · Seed (start point only) (gyro rotation + graph transl.)
         (gyro also feeds S5)            S6 · Coarse → fine GICP (adaptive gate, robust kernel) ◄── Local map
                                                                                    (2-level Gaussian voxels)
Reject (no factor, no map write)    ◄── S7 · Integrity gate (checks + Hessian degeneracy) ──► S8 · Map insert
                                                                                    (accepted scans only)
                                          S9 · Emit LoFactor (T_ij + Σ (rot-first) + health) ──► Graph backend
                                                                                    (Between factor + 6×6 Σ)
```

Scans flow top to bottom. The gyro feeds only S3 and S5. S7 also compares against gyro rotation,
but it can only discard a scan, so the emitted factor carries no IMU information. The local map
is written only from accepted scans and read only by S6.

## Interfaces

The stack runs as one ROS 2 Jazzy lifecycle node (composable, intra-process). It has four inputs
and one primary output.

| Direction | Topic / type | Rate | Required | Use |
|---|---|---|---|---|
| In | `livox_ros_driver2/CustomMsg` (per-point `offset_time`) | 10 Hz | Yes | Points |
| In | `sensor_msgs/Imu`: Mid-360 internal or MTi-680G (gyro only) | ≥ 200 Hz | Yes (degrades without) | Deskew, rotation seed |
| In | `sensor_msgs/JointState` | ≥ 100 Hz | Yes on robot | Leg self-filter, standstill detection |
| In | `nav_msgs/Odometry` from the graph (pose + velocity) | Any | Optional | Translation seed, deskew translation |
| Out | `lo_msgs/LoFactor` | Per accepted keyframe | — | Factor for the graph |
| Out | `diagnostic_msgs/DiagnosticArray` | 1 Hz | — | Health, timing, degeneracy |
| Out | `nav_msgs/Odometry` (LO frame) | 10 Hz | — | Debug and visualisation only. **Never fused.** |
| Out | `sensor_msgs/PointCloud2` deskewed scan, local map | 10 Hz / 0.5 Hz | — | Debug, decimated |

`LoFactor` message definition:

```
builtin_interfaces/Time stamp_i      # reference time of frame i (= graph node time)
builtin_interfaces/Time stamp_j      # reference time of frame j
uint64 chain_id                      # increments on every chain break (map reset)
geometry_msgs/Pose  T_ij             # relative pose, base frame, i -> j
float64[36] covariance               # row-major, order [rx ry rz tx ty tz]
uint8 health                         # 0 OK, 1 DEGRADED, 2 LOW_CONFIDENCE
float64[6] hessian_eigenvalues       # ascending, for backend diagnostics
float64[18] degenerate_directions    # up to 3 unit 6-vectors, unobservable axes
float32 inlier_ratio
float32 residual_rms
uint32  flags                        # bitfield: IMU_MISSING, DESKEW_FALLBACK, SEED_FALLBACK, ...
```

A backend adapter, outside this stack, turns `LoFactor` into `gtsam::BetweenFactor<Pose3>` with
a `noiseModel::Gaussian::Covariance`. It drops a factor if `chain_id` differs between i and j.
Keeping that adapter separate keeps the LO free of GTSAM.

## Pipeline stages

Each scan runs through ten stages in order. A failure in any stage stops that scan: no factor is
emitted and the map is not touched.

1. **S0. Input validation and time** *(industrial requirement)*
   - Reject a scan if timestamps go backwards, it jumps > 50 ms from the expected period, it has
     fewer than 3 000 valid points, or point times fall outside [0, 120 ms].
   - Track the PTP offset and IMU–LiDAR clock skew. If they exceed 2 ms, flag and stop emitting.
   - Buffer the IMU in a lock-free ring (2 s). Use a scan only once IMU samples cover its whole
     time span.
2. **S1. Self-filter** *(quadruped-specific)*
   - A static body crop box.
   - Leg capsules from forward kinematics on joint states, interpolated to each point's time and
     inflated by 3 cm.
   - Body-blockage monitor: if more than 30% of returns fall within 0.5 m, set the
     `SENSOR_BLOCKED` flag.
3. **S2. Range filter and downsampling** *(FAST-LIO2, KISS-ICP, DLIO)*
   - Range gate [0.3 m, 60 m], NaN and zero-reflectivity removal.
   - Two voxel sizes from one adaptive base `v`: the map uses `0.5·v`, registration uses `1.0·v`.
   - `v` follows spaciousness: `v = clamp(0.1 · median_range_smoothed, 0.1 m, 0.5 m)`.
4. **S3. Continuous-time deskew** *(DLIO)*
   - Integrate gyro between IMU samples. Between samples, use constant angular acceleration to
     place each point.
   - Translation comes from the graph velocity if available, else from LO's last relative velocity.
   - Deskew to the scan reference time, which is the end of the scan or the graph node time.
   - Gyro bias is owned by the stack: see S9 and the state machine section.
5. **S4. Per-point covariances** *(GLIM, DLIO)*
   - Run kNN (k = 10) once, on the deskewed registration cloud.
   - Regularise each covariance to a plane: eigenvalues set to (1, 1, 1e-3).
   - Keep covariances with the cloud so they are reused when it enters the map.
6. **S5. Seed** *(no information, start point only)*
   - Rotation from gyro integration, translation from graph odometry. Fall back to the last LO
     velocity, then to identity. The source is recorded in `flags`.
7. **S6. Registration, coarse then fine** *(GLIM, small_gicp, KISS-ICP)*
   - Target: an incremental Gaussian voxel map at two resolutions (fine `v`, coarse `4v`).
   - Coarse GICP: 5 iterations against the coarse map, wide gate, for basin widening.
   - Fine GICP: up to 15 iterations against the fine map, with the KISS-style adaptive gate
     `τ = 3σ_dev`.
   - Geman-McClure kernel with scale `τ/3`.
   - Surface-orientation check: drop correspondences on the far side of a surface.
   - Converged when `|Δrot| < 1e-4 rad` and `|Δtrans| < 1e-4 m`.
8. **S7. Integrity and degeneracy**
   - Compute quality metrics and run the Hessian eigen-analysis, then make the
     accept / degrade / reject decision. See the next section.
9. **S8. Map update** *(MOLA, KISS-ICP)*
   - Only for accepted scans, and only if motion since the last insert exceeds 0.1 m or 2°.
   - At most 20 points per voxel, LRU removal of voxels unseen for 30 s, radius crop at 80 m.
10. **S9. Factor emission and bookkeeping** *(GLIM)*
    - Emit `T_ij` between consecutive emitted frames, either at every graph-node request or when
      scan overlap with the last emitted frame drops below 90%.
    - Feed standstill evidence to the gyro bias estimator.
    - Push timing and metrics to diagnostics.

## Integrity and degeneracy

Every converged scan gets one of three verdicts. **ACCEPT** emits the factor and updates the map.
**DEGRADED** emits the factor with inflated Σ and updates the map only conditionally. **REJECT**
emits nothing and leaves the map untouched.

| Check | Threshold (default) | On failure |
|---|---|---|
| Converged within iteration cap | 15 fine iterations | REJECT |
| Inlier ratio (source points with a correspondence within τ) | ≥ 0.5 | REJECT |
| Residual RMS vs running median | ≤ 2.0× | REJECT |
| Rotation vs gyro-integrated rotation over the interval | ≤ 3° | REJECT |
| Motion plausibility | \|v\| ≤ 2.5 m/s, \|ω\| ≤ 4 rad/s, deviation from seed ≤ 0.3 m / 10° | REJECT |
| Numerical health | No NaN/Inf, Cholesky of H succeeds | REJECT |
| Hessian degeneracy | Normalised eigenvalue < λ_min | DEGRADED |
| Sensor blocked, deskew fallback | Flag set | DEGRADED |

The gyro check is a gate, not a measurement. It can only throw a factor away, never move its
value. Gyro rotation over 100 ms is accurate to well under 0.1°, so a 3° disagreement means ICP
fell into a wrong minimum.

**Degeneracy handling** (solution remapping, Zhang et al. 2016):

1. Scale the rotation block of H by the median range `L`, so rad and m are comparable. Then
   eigen-decompose: H = VΛVᵀ.
2. A direction `v_k` is degenerate when `λ_k / N_points < λ_min`. λ_min is tuned on corridor and
   open-field bags.
3. Project each fine-stage update onto the observable subspace only. The pose does not wander
   along unobservable axes.
4. In those directions the factor value equals the seed. The covariance in those directions is
   set to σ = 1 m / 0.5 rad, so the value carries effectively zero information. The graph fills
   them from leg odometry or other factors.
5. Map insertion in DEGRADED is allowed only with at most one degenerate direction. With more,
   the map would be built from poses that are mostly seed.

Five consecutive REJECTs (0.5 s) move the stack to LOST.

## Covariance model

The output covariance is the registration Hessian's inverse, scaled by the fit residual,
inflated by a calibrated factor, and floored. It is then expressed in the base frame in GTSAM
order.

```
Σ_j = κ · σ̂² · H⁻¹ + Σ_floor,        σ̂² = Σ_k ρ(r_k) / (N − 6)
```

- **H** is `Σ Jᵀ W J` at convergence of the fine stage, in the LiDAR frame, with right
  perturbation.
- **κ** is the inflation for correlated points, default 20. Tune it so the mean NEES ≈ 6 (6-DoF)
  on ground-truth bags, both sim and real.
- **Σ_floor** is `diag(0.05°, 0.05°, 0.05°, 5 mm, 5 mm, 5 mm)²`. It stops near-perfect fits
  claiming impossible precision.
- **Degenerate directions** are replaced as described in the previous section.
- **Deskew fallback** (IMU gap): Σ × 4.
- **Between covariance:** `Σ_ij ≈ Σ_j` when i and j are consecutive emissions, because both are
  registered to a map anchored near i. Across a gap from rejected scans, use `Σ_ij ≈ Σ_i + Σ_j`.
- **Frame change:** `Σ_base = Ad(T_base_lidar) · Σ_lidar · Ad(T_base_lidar)ᵀ`, with the adjoint in
  `[rot, trans]` order, matching `gtsam::Pose3::AdjointMap`.

Unit test this with a known synthetic H: write it in, read it back through the GTSAM adapter,
and check the marginals. Rotation-first ordering and perturbation side are the two most common
silent bugs.

## Health state machine

*Five rejects drop to LOST; recovery reuses the map before resetting it.*
LO health states · 5 states, 8 transitions.

| From | Event | To |
|---|---|---|
| INIT (stand still 1 s; gyro bias, gravity, map) | ready | TRACKING |
| TRACKING (full-weight factors; bias updates at rest) | degenerate | DEGRADED |
| DEGRADED (factor, Σ inflated; insert if ≤ 1 bad axis) | clear 1 s | TRACKING |
| TRACKING | 5 rejects | LOST |
| DEGRADED | 5 rejects | LOST |
| LOST (no factors emitted; black box dumped) | next scan | RECOVERING |
| RECOVERING (wide-gate coarse match against the kept map) | match found | TRACKING |
| RECOVERING | no match 2 s (reset map) | INIT |

INIT needs 1 s of standstill (joint velocities ≈ 0 and LO motion < 1 mm per scan) to estimate
gyro bias and the gravity direction and to build the first map. TRACKING refreshes the bias at
every 1 s standstill, so the stack never needs IMU state from the graph. RECOVERING first tries a
wide-gate coarse match against the kept map. After 2 s without a match it resets the map,
increments `chain_id` and returns to INIT, so the backend never links across the break.

## Industrial hardening

The stack assumes every input will fail at some point and defines a fixed response for each
failure. It never stalls, grows a backlog, or crashes silently.

| Fault | Detection | Response |
|---|---|---|
| LiDAR stops | No scan for 200 ms | ERROR diagnostic, no factors, stay ready |
| IMU gap | Sample gap > 20 ms in scan span | Deskew with LO's last velocity, `DESKEW_FALLBACK`, Σ × 4 |
| IMU lost | No IMU for 1 s | Same fallback plus WARN. Gyro gate disabled, so plausibility thresholds tighten by 2× |
| Joint states lost | No message for 100 ms | Conservative static leg exclusion box, flag |
| Clock jump or PTP lost | Non-monotonic stamp, jump > 50 ms, offset > 2 ms | Drop scan, ERROR until stable for 2 s |
| Compute overrun | Scan takes > 100 ms | Queue depth 1, newest wins, never a backlog. The next factor spans the gap |
| Sensor blocked or dusty | > 30% returns within 0.5 m, or valid points < 3 000 | DEGRADED or REJECT, flag |
| Solver blow-up | NaN/Inf, Cholesky fail, step > 0.5 m / 10° | REJECT |
| Repeated rejects | 5 in a row | LOST, then RECOVERING |
| Process crash | Lifecycle supervisor | Restart, new `chain_id`, INIT |

Engineering rules:

- **Bounded everything:** preallocated point and voxel pools, map capacity cap with LRU eviction,
  fixed thread pool, no per-scan heap allocation.
- **Deterministic replay:** message time only (never wall clock), fixed thread count, ordered
  voxel iteration. The same bag gives bit-identical output, which CI checks.
- **Black box:** an in-RAM ring of the last 20 s of raw scans, IMU and joints, dumped to MCAP on
  LOST or ERROR (rate-limited), stamped with the config hash and git SHA.
- **Lifecycle discipline:** `configure` validates every parameter against a schema (ranges,
  orthonormal extrinsics, files present). `activate` happens only after INIT succeeds.
- **Runtime:** worker threads pinned to dedicated cores, CPU governor set to performance,
  optional `SCHED_FIFO`.
- **Code quality:** `-Wall -Werror`, clang-tidy, no exceptions in the hot path, ASan, UBSan and
  TSan builds in CI, 8 h soak test with flat memory.

## Dependencies and code structure

The core is ROS-free C++17 with four dependencies. This keeps it small, unit-testable and
deterministic. ROS, GTSAM and file I/O live in thin outer packages.

| Dependency | Where | Why |
|---|---|---|
| Eigen 3.4 | Core | Linear algebra |
| Sophus (header-only) | Core | SO(3)/SE(3), exp/log, adjoints |
| small_gicp (MIT) | Core | GICP factors, Gaussian voxel map, kNN. Fork it and pin the version; replace pieces only if profiling demands |
| TBB *or* OpenMP (pick one) | Core | Fixed 2-thread pool |
| rclcpp, rclcpp_lifecycle, diagnostic_updater, livox_ros_driver2 msgs, rosbag2 (MCAP) | ROS layer | Node, diagnostics, black box |
| GTSAM | Backend adapter only | `LoFactor` → `BetweenFactor` |

The core does not use PCL, CUDA or GTSAM.

```
lo_core/             # pure C++, no ROS
  types/             # Scan, Point (xyz, t, cov), SE3 aliases, flags
  preprocess/        # validation, self-filter (FK capsules), range, voxel
  deskew/            # IMU ring, CT integration, gyro bias estimator
  registration/      # seed, coarse/fine GICP, adaptive gate, orientation check
  map/               # 2-level Gaussian voxel map, LRU, insert policy
  integrity/         # checks, degeneracy, covariance
  state/             # INIT/TRACKING/DEGRADED/LOST/RECOVERING
  emit/              # keyframe overlap, Between composition
lo_msgs/             # LoFactor.msg
lo_ros/              # lifecycle node, conversions, diagnostics, black box
lo_gtsam_adapter/    # backend-side factor construction + chain_id handling
lo_tools/            # replay CLI, evo/NEES evaluation, fault injector
```

## Default parameters

These are starting values for a Mid-360 on a ~0.5 m tall quadruped. All are validated at
`configure` and logged with the config hash.

| Parameter | Default | Notes |
|---|---|---|
| `range.min / max` | 0.3 m / 60 m | Min includes Mid-360 blind zone plus body margin |
| `voxel.base` | adaptive, 0.1–0.5 m | `0.1 · median range`, smoothed with EMA α = 0.1 |
| `voxel.map_ratio / coarse_ratio` | 0.5 / 4.0 | Fine-map voxel and coarse-map voxel, as multiples of `voxel.base` |
| `knn.k` | 10 | Per-point covariance |
| `self_filter.inflate` | 0.03 m | Around FK leg capsules |
| `imu.min_rate` | 200 Hz | Below this the IMU counts as degraded |
| `gicp.coarse_iters / fine_iters` | 5 / 15 | |
| `gate.initial / min / max` | 1.0 / 0.1 / 2.0 m | KISS-style adaptive τ |
| `kernel` | Geman-McClure, scale τ/3 | |
| `integrity.inlier_min` | 0.5 | |
| `integrity.gyro_rot_max` | 3° | Gate only |
| `integrity.v_max / w_max` | 2.5 m/s / 4 rad/s | Platform limits |
| `degeneracy.lambda_min` | tune on bags | Normalised by point count |
| `cov.kappa` | 20 | Tune to NEES ≈ 6 |
| `cov.floor` | 0.05° / 5 mm | Per axis, 1σ |
| `map.insert_dist / insert_rot` | 0.1 m / 2° | |
| `map.max_pts_voxel / radius / ttl` | 20 / 80 m / 30 s | |
| `emit.overlap_new_kf` | 0.9 | Emit when overlap < 90% |
| `lost.consecutive_rejects` | 5 | 0.5 s at 10 Hz |
| `standstill.min_duration` | 1.0 s | For gyro bias update |
| `blackbox.duration` | 20 s | RAM ring |

## Validation and acceptance criteria

The stack is accepted only when it meets every criterion below on real robot bags, not only in
sim.

| Criterion | Target | How measured |
|---|---|---|
| Relative translation error | ≤ 1.0% over 10 m segments | `evo_rpe` vs ground truth |
| Relative rotation error | ≤ 0.1°/m | `evo_rpe` |
| Covariance consistency | Mean NEES in [4, 9] (ideal 6) | `lo_tools` NEES script |
| False accepts in fault-injection suite | 0 | Injected faults must never yield a tight wrong factor |
| Factor availability, nominal environments | ≥ 99% of graph nodes | Diagnostics counters |
| Recovery from LOST, normal geometry | ≤ 2 s | Fault-injection suite |
| Timing | Median ≤ 25 ms, p99 ≤ 50 ms on Orin | Diagnostics, 1 h run |
| Memory | Flat within ±5% over 8 h | Soak test |
| Determinism | Bit-identical output on replay | CI |

Test data, in order of trust:

1. **Gazebo + ground truth:** pipeline logic, frames, covariance plumbing. Gazebo's `gpu_lidar`
   has no per-point timing or Mid-360 pattern, so deskew cannot be validated here.
2. **Real bags with ground truth:** motion capture indoors, total station or a surveyed closed
   loop outdoors. This is where deskew, κ and λ_min get tuned.
3. **Public legged-robot datasets with a Mid-360** for a second opinion before tuning freezes.
   Confirm each dataset's sensor list first.

The fault-injection suite (`lo_tools`) runs on every commit:

- IMU dropout
- Clock jump
- PTP loss
- 50% point dropout
- Lens blocked
- Leg kept inside the FoV
- Crowd of dynamic objects
- Long featureless corridor
- Open field
- Stair ascent
- Sudden 180° turn in place

## Open decisions and roadmap

Two decisions block hardware and must be made first: LiDAR mount pitch and which IMU feeds
deskew.

| Decision | Options | Recommendation | Decide by |
|---|---|---|---|
| Mid-360 mount pitch | Level, or pitched down 15–30° | Pitch down. The −7° lower FoV at 0.5 m height sees ground only beyond ~4 m, so z, roll and pitch are weakly observed | Before mount design freeze |
| Deskew gyro | Mid-360 internal IMU, or MTi-680G | MTi if the backend has no IMU (better gyro, no correlation concern). Internal if the backend fuses the MTi (independent noise, shared clock) | Before Phase 2 |
| GPU registration | CPU small_gicp, or GPU VGICP (`gtsam_points`) | Stay on CPU; switch only if accuracy misses target | After Phase 3 results |
| Registration factors in the graph | `BetweenFactor` + Σ, or direct GICP cost factors (GLIM-style) | Between first. Direct factors are a v2 option that removes covariance tuning | After Phase 5 |
| Dynamic removal | LRU + robust kernel, or ray-cast free-space | LRU first; add ray-cast only if crowds break the gate | After field tests |

Build phases. Each one must pass its gate before the next starts:

1. **Core LO:** S0, S2, S5, S6, S8 on real Mid-360 bags. *Gate: no divergence, RPE ≤ 2%.*
2. **Deskew and self-filter:** S1, S3, S4, gyro bias estimator. *Gate: RPE ≤ 1% on trotting bags.*
3. **Integrity, covariance, adapter:** S7, S9, `LoFactor`, GTSAM adapter. *Gate: NEES in band,
   0 false accepts on fault bags.*
4. **Hardening:** state machine, watchdogs, black box, lifecycle, CI suites. *Gate: fault suite
   green, 8 h soak flat.*
5. **Orin deployment and tuning:** pinning, profiling, final κ and λ_min. *Gate: the full
   acceptance table.*
