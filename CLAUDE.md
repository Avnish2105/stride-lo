# CLAUDE.md — STRIDE LiDAR odometry (stride-lo)

Integrity-checked LiDAR odometry for quadrupeds: Livox Mid-360 → IMU-free relative-pose
factors (`lo_msgs/LoFactor`) for a GTSAM backend. ROS 2 Jazzy, CPU-only, Jetson AGX Orin target.
Status: phases 0–5 + integration done; what is left needs the robot (docs/PLAN.md "Remaining").

## Orientation: read on demand, not up front
Start with [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) (packages, scan data flow, state
machine, "where to change X"). The other docs are long: open only what the task needs and
`grep -n` for the section or I-number instead of reading them whole.
- [docs/DESIGN.md](docs/DESIGN.md): the spec (transcribed from `docs/design/*.pdf`; don't open the PDF).
- [docs/DECISIONS.md](docs/DECISIONS.md): the resolved open points D1–D8 and implementation interpretations I1–I40, one table row each.
- [docs/PLAN.md](docs/PLAN.md): status table, remaining work, results log (the original plan: `docs/history/`).
- [docs/INTEGRATION.md](docs/INTEGRATION.md), [docs/TUNING.md](docs/TUNING.md), [docs/SMALL_GICP.md](docs/SMALL_GICP.md).

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
- Claude usually runs **inside** the `stride-lo:dev` container (VS Code devcontainer:
  `/.dockerenv` exists, `colcon`/`ros2` on PATH). Run `scripts/build.sh`, `scripts/test.sh`
  directly. On the host (no ROS/compiler) prefix them with `scripts/dc run --rm dev`.
- Bags: read-only at `/data/bags`. 14 GB laptop: keep `BUILD_JOBS` ≤ 3 (GTSAM TUs).
- colcon settings live in `colcon_defaults.yaml` (Ninja, ccache, symlink install, quiet output),
  so a bare `colcon build --packages-select X` matches `scripts/build.sh`. Don't pass other
  `--cmake-args`: a changed CMake argument reconfigures and rebuilds the package.

## Fast inner loop (cheapest check that answers the question)
```bash
scripts/build.sh --packages-select lo_core     # only what changed (--packages-above lo_core: + dependents)
scripts/test.sh --quick --packages-select lo_core   # seconds; --quick skips the `slow` fault suite
scripts/test.sh --quick                        # 50 tests: unit + allocation + determinism, ~15 s
scripts/test.sh                                # 63 tests incl. the fault suite, ~2 min: before a commit
```
Failures: `colcon test-result --verbose`, or `ctest --rerun-failed --output-on-failure` in
`build/<pkg>`. Full build/test logs: `log/latest_build/<pkg>/stdout_stderr.log`, `log/latest_test/`.

## Long-running work (sims, bag suites, sanitizers, soak)
These take minutes to hours of real time and have caused most slow sessions. So:
- Run them with `run_in_background` and wrap them in `timeout <s>`. You are notified when they
  exit. **Never** poll with `until grep …; do sleep …; done` or `sleep` loops: a dead job turns
  that into an unbounded wait.
- Every command that can block (`ros2 launch`, `ros2 bag play`, `scripts/screen`, GUI tools) gets a
  `timeout`.
- Read results, not logs: `evaluate.py` / `bag_suite.py` print summaries; `tail`/`grep` logs.
- Screenshots only when the task is about the GUI (each one is a large image in context).
- Outputs go under `data/` (gitignored) or the scratchpad, never the repo root.
- Run one heavy job at a time: parallel sims/test suites contend for the CPU (the fault suite
  ran 3× slower next to a sim).

## Working style
- Every accuracy/covariance change reports before/after numbers (RPE, NEES, availability,
  timing). No numbers, no claimed improvement.
- Update `docs/PLAN.md` status and results at each phase commit; record new interpretations in
  `docs/DECISIONS.md`.
- One task per session; `/clear` between unrelated tasks so context does not carry hours of logs.
- Commit locally per phase on the feature branch; don't push unless asked.
