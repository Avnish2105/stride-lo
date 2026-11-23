# CLAUDE.md — STRIDE LiDAR odometry (stride-lo)

Integrity-checked LiDAR odometry for quadrupeds: Livox Mid-360 → IMU-free relative-pose
factors (`lo_msgs/LoFactor`) for a GTSAM backend. ROS 2 Jazzy, CPU-only, Jetson AGX Orin target.

**Read first:** [docs/DESIGN.md](docs/DESIGN.md) (the spec, transcribed from
`docs/design/*.pdf`), [docs/DECISIONS.md](docs/DECISIONS.md) (every open point resolved, with
reasons), [docs/PLAN.md](docs/PLAN.md) (phases, gates, status, results).

## Hard rules (from the spec — never trade these away)
1. **No IMU information in the factor.** The gyro only deskews and seeds; the gyro check may only
   reject. No IMU term in the registration cost; the map is built only from LO poses.
2. **Never confidently wrong.** Any failed check → no factor, or Σ inflated in the unseen
   directions. Missing factors are fine; tight wrong factors are not.
3. **Covariance order is `[rx ry rz tx ty tz]`, right (body-frame) perturbation, base frame.**
   Sophus is `[trans, rot]`: only touch it through `lo_core/types/se3.hpp`.
4. **lo_core is ROS-free C++17** with exactly Eigen, Sophus, small_gicp, OpenMP. No PCL, no GTSAM,
   no CUDA in the core. GTSAM lives only in `lo_gtsam_adapter`.
5. **Zero heap allocations per scan after warm-up.** Size buffers from `limits.*` at construction.
   The allocation-counter test must stay at 0.
6. **Deterministic:** message time only in decisions (wall clock only for timing stats), fixed
   OpenMP team of 2 with static schedules and ordered reductions, ordered voxel iteration. Same
   bag → bit-identical output (`lo_replay` hash).
7. **No exceptions in the hot path**; status codes. `-Wall -Werror` on own packages, clang-tidy.
8. **Jetson-portable:** no `-march=native`, no x86-only deps, GPU never required.
9. **Vendored code is not edited in place:** `src/third_party/small_gicp` stays pristine; changes
   are `patches/small_gicp/*.patch`, applied by `scripts/build.sh`.
10. **The GPS-denied repo (`~/GPS-denied_State_estimation_Qudrapad`) is read-only** for this
    project: its bags are test data, its estimator is the future consumer. Don't modify it.

## Environment
- The host (Ubuntu 26.04) has no ROS/compiler. Everything runs in the `stride-lo:dev` image via
  `scripts/dc` (compose wrapper; repo mounted at its own host path, bags read-only at
  `/data/bags`, container user = host uid):
  ```bash
  scripts/dc build dev                        # image (GTSAM 4.3, Livox driver, Sophus, Gazebo)
  scripts/dc run --rm dev scripts/build.sh    # Release build (--sanitizer=address|undefined|thread)
  scripts/dc run --rm dev scripts/test.sh     # all tests
  ```
- Claude runs on the host and drives Docker; it does not reopen the folder in the container.
- Keep `BUILD_JOBS` ≤ 3 on the 14 GB laptop (GTSAM and heavy TUs).

## Layout
`src/lo_core` (pipeline), `src/lo_synth` (simulator lib), `src/lo_msgs`, `src/lo_ros` (lifecycle
node, supervisor, black box, configs), `src/lo_gtsam_adapter` (+ `lo_graph_demo`), `src/lo_tools`
(replay, fault injector, evaluation, suites), `src/lo_sim` (Gazebo rig),
`src/third_party/small_gicp`, `patches/`, `docker/`, `scripts/`, `docs/`.

## Working style
- Every accuracy/covariance change reports before/after numbers (RPE, NEES, availability,
  timing). No numbers, no claimed improvement.
- Update `docs/PLAN.md` status and results at each phase commit; record new interpretations in
  `docs/DECISIONS.md`.
- Commit locally per phase on the feature branch; don't push unless asked.
