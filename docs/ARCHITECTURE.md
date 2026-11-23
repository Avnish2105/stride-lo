# Architecture (as built)

The spec is [DESIGN.md](DESIGN.md); every place where the implementation had to interpret or
extend it is in [DECISIONS.md](DECISIONS.md) (I1–I33). This page maps the design onto the code.

## Packages

| Package | Kind | Contents |
|---|---|---|
| `lo_core` | plain CMake, ROS-free C++17 (Eigen, Sophus, small_gicp, OpenMP) | the whole pipeline: `lo::Pipeline` |
| `lo_msgs` | ROS 2 interfaces | `LoFactor`, `NodeRequest` |
| `lo_ros` | ROS 2 | lifecycle node `stride_lo` (component + executable), `lo_supervisor`, conversions, parameters, URDF → kinematics, black box, configs, launch, RViz |
| `lo_gtsam_adapter` | ROS 2 + GTSAM 4.3 | `toBetweenFactor`, `toRelative` (body frame), `ChainTracker`, `lo_graph_demo` |
| `lo_synth` | plain CMake, ROS-free | Mid-360-like simulator: scenes, ray casting, trajectories, IMU/joints, faults, 13 scenarios, metrics, soak |
| `lo_tools` | ROS 2 | `lo_replay`, `lo_synth_bag`, `evaluate.py`, `bag_suite.py`, `fault_inject.py`, `run_fault_suite.sh`, `soak.py`, `live_check.sh`, `time_offset.py` |
| `lo_sim` | ROS 2 + Gazebo Harmonic | `lo_sim_rig` (live sensors from a scenario), `lo_sim_world` (SDF), `KinematicRig` gz-sim plugin, `sim.launch.py` |
| `third_party/small_gicp` | vendored v1.0.1, pristine | patched copy in `.stride_patched/` (4 patches, `patches/small_gicp`) |

GTSAM never enters `lo_core` (CLAUDE.md rule 4); the backend sees only `LoFactor`.

## Data flow in `lo::Pipeline`

```
addImu / addJoints / addGraphOdom / addNodeRequest / addPtpOffset      (any order, message time)
          │
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

Every buffer is sized from `limits.*` in the constructor; `process()` and `answerRequests()`
allocate nothing (test `test_allocations`). Decisions use message time only; wall time feeds
the timing statistics. OpenMP uses a fixed team of 2 with static blocks and ordered sums, so the
same input gives bit-identical output (`test_determinism`, `lo_replay` output hash).

## Frames and conventions

- Tangent order `[rx ry rz tx ty tz]`, right perturbation, base frame (`T_ij` = base_i → base_j);
  Sophus is touched only in `lo_core/types/se3.hpp` (`expRT`, `logRT`, `adjointRT`).
- Internal LO frame = the INIT map frame; the published odometry is `T_align` × internal pose
  (gravity aligned, yaw 0, from INIT). No pose is published before INIT completes (I24).
- `chain_id` = stamp of the chain's first frame; a chain ends on re-INIT after LOST.

## ROS node threads (`lo_ros::LoNode`)

- Executor thread: subscriptions only convert and push into lock-free SPSC queues; scans go to
  a one-deep slot (newest wins).
- Worker thread: owns the pipeline; drains the queues, waits briefly for IMU coverage of the
  scan, processes, publishes factors (also late answers right after requests arrive),
  odometry, deskewed cloud and the local map. Optional CPU pinning / SCHED_FIFO.
- Lifecycle: configure validates every parameter, extrinsics (tf) and the URDF, then starts
  processing (INIT runs while inactive); activate is refused until INIT completes;
  `lo_supervisor` retries and re-drives a respawned process.
- Watchdogs (LiDAR stop, IMU lost, joints lost), `/diagnostics` at 1 Hz, black box (20 s rings
  dumped to MCAP on LOST/ERROR, with config hash and git SHA).

## Test and evaluation tooling

- Unit and synthetic tests: `scripts/test.sh` (62 tests: SE(3), deskew, degeneracy, emitter,
  state machine, seed gate, black box, adapter + GTSAM marginals, the 11-scenario fault suite,
  determinism, zero allocations).
- Bags: `lo_replay` (recorded order, requests from a topic at stamp + latency), `evaluate.py`
  (RPE, NEES, false accepts, availability, FAST-LIO comparison), `bag_suite.py` (table of a
  bag set), `fault_inject.py` / `run_fault_suite.sh`, `live_check.sh` (bag play into the
  live node and the demo graph).
- Live synthetic rig: `ros2 launch lo_sim sim.launch.py scenario:=crowd`.
