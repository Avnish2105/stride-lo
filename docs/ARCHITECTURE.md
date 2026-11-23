# Architecture

The one page to read first. It answers *what is this, how do the parts fit, where does a change
go*. The spec is [DESIGN.md](DESIGN.md); every interpretation of it is in
[DECISIONS.md](DECISIONS.md) (D*, I1–I40); phase history and numbers are in [PLAN.md](PLAN.md).

## 1. What STRIDE is, in one picture

STRIDE turns LiDAR scans into **relative-pose measurements** ("factors") that a GTSAM backend
adds to its graph. It is a *sensor*, not an estimator: it never fuses IMU information into a
factor, and it would rather emit nothing than emit something tight and wrong.

```
                 ┌──────────────────────── robot ─────────────────────────┐
 Livox Mid-360 ──┤ /livox/lidar (CustomMsg, 10 Hz)                        │
   (+ its IMU) ──┤ /livox/imu   (200 Hz)  ── deskew + seed + reject only  │
 legged robot  ──┤ /joint_states          ── self-filter (legs in FOV)    │
                 └───────────────┬────────────────────────────────────────┘
                                 ▼
                    ┌────────────────────────┐   ~/odometry, ~/pose, ~/path   (viz / debug)
                    │  stride_lo (lo_ros)    │── ~/deskewed, ~/local_map
                    │  lifecycle node        │── /diagnostics, black box MCAP on failure
                    │  └─ lo::Pipeline       │
                    │     (lo_core, no ROS)  │── ~/factor  (lo_msgs/LoFactor) ─────┐
                    └───────────▲────────────┘                                     ▼
                                │ NodeRequest(stamp)  "give me a factor ending here"   backend
                                │ graph odometry      (optional translation seed)    (GTSAM,
                                └──────────────────────────────────────────────────── iSAM2)
```

A `LoFactor` is `T_ij` (base frame, i → j), a 6×6 covariance in `[rx ry rz tx ty tz]` order,
a health (`OK` / `DEGRADED` / `LOW_CONFIDENCE`), flags saying *why*, and a `chain_id`. The
backend wraps it as `BetweenFactor<Pose3>` (`lo_gtsam_adapter` does exactly that).

## 2. The three invariants that shape everything

Most of the odd-looking code exists to protect one of these (full list: CLAUDE.md hard rules).

| Invariant | What it forces in the code |
|---|---|
| **No IMU information in the factor** | Gyro is used to deskew, seed, and *reject*; never in the cost. The map is built from LO poses only. |
| **Never confidently wrong** | Every check can only drop the factor or inflate Σ (in the unobservable directions). Fault suite asserts 0 false accepts. |
| **Real-time and deterministic on a Jetson** | Buffers sized from `limits.*` at construction (0 allocations/scan, tested), message time only, OpenMP team of 2 with ordered reductions → same bag gives a bit-identical hash. |

## 3. Packages and how they depend on each other

```
                      small_gicp (vendored, patched copy in .stride_patched/)
                          │
                       lo_core ◄──────────────── the product. ROS-free C++17.
                     ┌────┴─────┬───────────────┐
                 lo_synth    lo_ros ◄── lo_msgs ─► lo_gtsam_adapter (+GTSAM)
              (simulator,  (ROS node)              (backend glue, demo graph)
               fault suite)    │                         │
                     └────┬────┴──────────┬──────────────┘
                       lo_tools         lo_sim (+Gazebo Harmonic)
               (replay, evaluate,     (live synthetic robot rig)
                suites, launcher)
```

| Package | Kind | Holds | You touch it when… |
|---|---|---|---|
| `lo_core` | plain CMake, no ROS | `lo::Pipeline` and every stage | changing the algorithm, checks, Σ |
| `lo_synth` | plain CMake, no ROS | Mid-360-like ray caster, 13 scenarios, faults, metrics | adding a scenario or a fault test |
| `lo_msgs` | ROS interfaces | `LoFactor`, `NodeRequest` | changing the contract with the backend (rare, breaking) |
| `lo_ros` | ROS 2 | lifecycle node, supervisor, params ↔ `Config`, URDF → kinematics, black box, launch, configs | ROS wiring, parameters, a new robot config |
| `lo_gtsam_adapter` | ROS 2 + GTSAM | `toBetweenFactor`, `ChainTracker`, `lo_graph_demo` | how a backend consumes factors |
| `lo_tools` | ROS 2 | `lo_replay`, `evaluate.py`, `bag_suite.py`, fault injector, soak, launcher GUI | measuring anything on bags |
| `lo_sim` | ROS 2 + Gazebo | sensor rig, world generator, `KinematicRig` plugin | live demo / end-to-end runs |

Hard boundary: **GTSAM never enters `lo_core`**, and `lo_core` never includes ROS. The node
converts messages to plain structs and calls the pipeline.

## 4. Inside `lo::Pipeline`: the life of one scan

Inputs arrive in any order with their message time (`addImu`, `addJoints`, `addGraphOdom`,
`addNodeRequest`, `addPtpOffset`). Then `process(scan)` runs stages S0–S9:

```
process(scan) ─ S0 validate + clock health ─ IMU status ─ reference time (request in span | end)
          ├─ S1 self-filter (body box, URDF capsules at 11 times, blockage)
          ├─ S2 range gate, adaptive voxel v, 1.0·v registration cloud
          ├─ S5 seed: gyro rotation; translation from the graph if it passes the gate (I26, I29),
          │          else LO's own velocity (≥ 50 ms baseline), else identity
          ├─ S3 deskew: gyro rotation (constant ang. accel) + base-origin lever arm (I28)
          │           + base velocity; points to the reference time
          ├─ S4 kNN covariances (k = 10, plane-regularised, sensor-facing normals)
          ├─ S6 coarse (4·v map, wide gate) → reset degenerate axes → projected fine GN
          │     (0.5·v map, adaptive gate, GM kernel, frozen correspondences late, I18/I30)
          ├─ S7 integrity checks → ACCEPT / DEGRADED / REJECT; Σ = κσ̂²H⁻¹ (+ weak directions,
          │     I31) + floor; degenerate axes σ = 1 m / 0.5 rad
          ├─ S8 map insert (accepted, ≥ 0.1 m / 2°, not under deskew fallback)
          └─ S9 FrameRecord ring; factors at requests (also late, I23) or on overlap < 90 %
answerRequests()   requests that arrived after their scan, answered from the records
```

Where each stage lives (`src/lo_core/`):

| Stage | Header dir / file | Source |
|---|---|---|
| S0 validation, clock | `preprocess/validation.hpp` | `validation.cpp` |
| S1 self-filter, kinematics | `preprocess/self_filter.hpp`, `kinematics.hpp` | `self_filter.cpp`, `kinematics.cpp` |
| S2 downsample | `preprocess/downsample.hpp` | `downsample.cpp` |
| S3 deskew, IMU buffer, bias | `deskew/` | `deskew.cpp`, `imu_buffer.cpp`, `bias_estimator.cpp` |
| S4 point covariances | `registration/point_covariance.hpp` | `point_covariance.cpp` |
| S5 seed | `registration/seed.hpp` | `seed.cpp` |
| S6 GICP (small_gicp factor) | `registration/` | `registration.cpp` |
| S7 integrity, degeneracy, Σ | `integrity/` | `degeneracy.cpp` (+ `pipeline.cpp`) |
| S8 local map (voxel hash) | `map/` | `local_map.cpp` |
| S9 emitter, requests | `emit/emitter.hpp` | `emitter.cpp` |
| state machine, orchestration | `state/`, `pipeline.hpp` | `pipeline.cpp` |
| config + validation | `config/config.hpp`, `schema.hpp` | `config.cpp` |
| SE(3) conventions (the only Sophus door) | `types/se3.hpp` | — |

### Health state machine

```
 INIT ──ready──► TRACKING ──degenerate──► DEGRADED ──clear for 1 s──► TRACKING
                    │ 5 rejects (from TRACKING or DEGRADED)
                    ▼
                  LOST ──next scan──► RECOVERING ──match against kept map──► TRACKING
                                          │ no match within 2 s
                                          ▼
                                        INIT  (map reset → new chain_id; the backend must
                                               not link factors across chains)
```
Source of truth: `state/state_machine.hpp` (DESIGN: 5 states, 8 transitions; D3 warm re-INIT;
I13 recovery window; I38 chain break without state change).

Per scan the verdict is `ACCEPT` / `DEGRADED` / `REJECT` / `SKIPPED`; a `RejectReason` records the
first failing check (order = evaluation order, `types/flags.hpp`).

### How a factor gets out

1. The backend picks a keyframe time and publishes `NodeRequest(stamp)`.
2. The scan whose time span contains `stamp` is deskewed *to that time* (reference time).
3. S9 emits `LoFactor(stamp_i = previous emitted node, stamp_j = stamp)`.
4. Requests that arrive after their scan are answered later from the `FrameRecord` ring.
5. Without requests the emitter falls back to overlap keyframes (overlap < 90 %).

## 5. Conventions you must keep in your head

- Tangent order `[rx ry rz tx ty tz]`, right perturbation, base frame (`T_ij` = base_i → base_j).
  Sophus is `[trans, rot]`, so touch it only through `types/se3.hpp` (`expRT`, `logRT`,
  `adjointRT`).
- Internal LO frame = the INIT map frame; published odometry = `T_align` × internal pose
  (gravity aligned, yaw 0). Nothing is published before INIT completes (I24).
- `chain_id` = stamp of the chain's first frame; it changes on re-INIT after LOST.
- Message time drives every decision; wall time is only for timing stats.

## 6. ROS node at runtime (`lo_ros::LoNode`)

```
 executor thread                       worker thread (owns the Pipeline)
 ───────────────                       ─────────────────────────────────
 callbacks: convert msg → struct  ──►  SPSC queues (IMU, joints, odom, requests)
 scan → one-deep slot (newest wins)──► wait briefly for IMU coverage → process()
                                       → publish factor / odometry / clouds / map
```

- **Lifecycle:** `configure` validates every parameter, tf extrinsics and the URDF, then starts
  processing (INIT runs while inactive). `activate` is refused until INIT completes.
  `lo_supervisor` retries and re-drives a respawned process.
- **Watchdogs:** LiDAR stop, IMU lost, joints lost → `/diagnostics` at 1 Hz.
- **Black box:** 20 s rings dumped to MCAP on LOST / ERROR, with config hash and git SHA.
- **Config:** `src/lo_ros/config/*.yaml` (`default`, `anymal_c`, `stride_rig`,
  `robot_template`) → `params.cpp` → `lo::Config` → `validate()`.

## 7. How you check that it works

Fastest first. Section 8 has the commands.

| Level | Tool | Time | Answers |
|---|---|---|---|
| unit | `lo_core` gtest (33) | seconds | conventions, stages, state machine |
| ROS glue | `lo_ros`, `lo_gtsam_adapter` gtest | seconds | params, conversions, GTSAM marginals = Σ |
| synthetic | `lo_synth` tests (fault suite, determinism, 0 allocations) | minutes | 0 false accepts, accuracy targets, bit-identical |
| bags | `lo_replay` + `evaluate.py`, `bag_suite.py` | minutes per bag | RPE, NEES, availability, timing vs truth |
| faults on bags | `fault_inject.py`, `run_fault_suite.sh` | tens of min | 0 false accepts under injected faults |
| live | `live_check.sh`, `ros2 launch lo_sim sim.launch.py` | real time | the node + supervisor + demo graph end to end |

Metrics: **RPE** (accuracy), **NEES** (is Σ honest; ≈ 3–6 is good here, ≫ 40 = false accept),
**availability** (fraction of requests answered), **false accepts** (must be 0).

## 8. Build / run topology

```
 host (Ubuntu, no ROS)                  stride-lo:dev container (ROS 2 Jazzy, GTSAM 4.3, Gazebo)
 ──────────────────────                 ─────────────────────────────────────────────────────
 VS Code ── Reopen in Container ──────► repo bind-mounted at its host path
 scripts/dc run --rm dev <cmd> ───────► scripts/build.sh  → colcon (colcon_defaults.yaml, Ninja, ccache)
                                         scripts/test.sh   → colcon test → ctest -j
 GPS-denied repo rosbags/ ───(ro)────► /data/bags
```

Build outputs: `build/ install/ log/` (Release), `build_<san>/ install_<san>/ log_<san>/`
(sanitizers). small_gicp is built from `.stride_patched/` (pristine vendor + `patches/`).

## 9. "I want to change X": where to go

| Change | Start at | Then verify with |
|---|---|---|
| a registration / integrity threshold | `config/config.hpp` (defaults) + `lo_ros/config/*.yaml` | `test.sh`, then `bag_suite.py` before/after |
| covariance scaling (κ, floor, λ_min) | `integrity/`, [TUNING.md](TUNING.md) | NEES and false accepts on the 25-bag suite |
| a new robot | `lo_ros/config/robot_template.yaml`, [INTEGRATION.md](INTEGRATION.md) | `live_check.sh` on its bags |
| a backend consuming factors | `lo_gtsam_adapter/adapter.hpp`, `examples/minimal_backend` | `scripts/check_consumer.sh` |
| a new fault case | `lo_synth/src/scene.cpp` / `simulator.cpp`, `faultSuiteNames()` | `test.sh --packages-select lo_synth` |
| small_gicp behaviour | a new `patches/small_gicp/*.patch` (never edit vendored code), [SMALL_GICP.md](SMALL_GICP.md) | full `test.sh` + determinism hash |
| a ROS topic / param | `lo_ros/src/lo_node.cpp`, `params.cpp` | `test_params`, `live_check.sh` |

## 10. Document map

| Doc | Read it when |
|---|---|
| this page | always first |
| [DESIGN.md](DESIGN.md) | you need the exact spec of a stage or interface |
| [DECISIONS.md](DECISIONS.md) | code deviates from the spec and you want to know why (search the I-number) |
| [PLAN.md](PLAN.md) | status, remaining work, historical numbers |
| [INTEGRATION.md](INTEGRATION.md) | wiring STRIDE into a robot or backend |
| [TUNING.md](TUNING.md) | calibrating on real bags |
| [SMALL_GICP.md](SMALL_GICP.md) | the vendored library and its patches |
