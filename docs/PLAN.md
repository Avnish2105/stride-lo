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
| 4 | State machine, watchdogs, black box, lifecycle + supervisor, fault injector + suite, sanitizers, soak | Fault suite green, memory flat ±5% | done (2026-11-22): synthetic + bag fault suites 0 false accepts (I35), ASan/UBSan/TSan green on CI, 8 h soak RSS flat (0.00 %) |
| 5 | lo_sim Gazebo rig + scenes, Orin tooling, TUNING.md | Full acceptance table on x86 (Orin timing reported only) | done on x86 (2026-11-22); Orin timing left for the robot |
| I | Integration readiness: INTEGRATION.md, robot template, out-of-tree consumer + CI check | External workspace builds against `install/` and consumes live factors | done (2026-11-23): `check_consumer.sh` PASS (yard 1, 40 s: 262 factors, 0 unmatched / rejected / chain breaks) |

## Remaining (needs the robot / Orin)

1. Orin timing (median ≤ 25 ms, p99 ≤ 50 ms; x86 laptop: 10.3 / 19.0 ms), `orin_setup.sh`.
2. Real Mid-360 bags with ground truth: `time_offset.py` for `imu.time_offset` (the sim value
   −2.25 ms is the emulated driver latency, I34), then κ / floor / λ_min per TUNING.md (I32).
3. Integration into a real backend (e.g. `factor_graph_estimator`) is done in that repo by its
   owner, against [INTEGRATION.md](INTEGRATION.md); STRIDE is not a FAST-LIO drop-in (I40).
4. PR #1 `feat/stride-lo` → `main` opened 2026-11-23 (https://github.com/Avnish2105/stride-lo/pull/1); merge is the owner's call.

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
- **Phases 4–5** (2026-11-22).
  - CI on GitHub Actions (image, Release -Werror, 62 tests, fault suite, clang-tidy), sanitizers
    ASan/UBSan/TSan: green. First run found an uninitialised `degenerate_directions` (fixed).
  - Soak: 8.01 h of synthetic data in one pipeline, 287 804 scans, RSS 238.7 → 238.7 MB (spread
    0.00 %), 0 false accepts, 0 LOSTs, t_med 4.5 ms / p99 6.6 ms.
  - Bag fault suite (rough 1): 0 false accepts on all 7 faults after the gyro-less deskew rotation
    floor (I35), except PTP drift, which needs the PTP monitor / skew check on hardware.
  - Final 25-bag regression (suite v5): RPE 0.125 %, 0.0081 deg/m, availability 0.999,
    0 false accepts, mean NEES 2.22, t_med 10.3 ms / p99 19.0 ms.
  - lo_sim live: headless room RPE 0.108 %, Gazebo crowd 0.076 % (GUI run: 0.083 %), 0 false
    accepts.
- **Demo, Gazebo fix, integrity fix** (2026-11-23, after PR #1).
  - VS Code status-bar buttons (Launch / Build / Test / Screen / Stop, `.vscode/tasks.json`) and a
    shared virtual screen (`scripts/screen`: Xvfb :99 → noVNC on port 6080) that the user and
    Claude both see. The host desktop icon and the GPU compose override are retired.
  - Gazebo LOST: rig release timing + Gazebo clock + software-GL CPU load (I37). Live crowd with
    Gazebo GUI + RViz + recording: availability 0.170 → 0.966, LOSTs 20 → 1 (the scenario's own),
    clock_unstable rejects 1126 → 0, false accepts 3 → 0, RPE 13.4 % → 0.095 %, NEES 1.49.
  - Long-gap recovery no longer bridges the gap with a factor (I38, `recovery.max_bridge` 2 s).
    Crowd + clock_jump faults (5 cases): false accepts 4 → 0, availability −0.001 at most.
    25-bag suite v6: identical to v5 on every bag (RPE 0.125 %, NEES 2.22, 0 false accepts,
    availability 0.999; timing not comparable, run under load). Synthetic + fault suites:
    0 false accepts. 62 tests green.
  - `lo_graph_demo` no longer dies on an ill-posed new chain (I39).
- **Integration readiness** (2026-11-23). `docs/INTEGRATION.md` (inputs, outputs, LoFactor
  semantics, emit modes, GTSAM recipe, FAST-LIO migration, new-robot checklist, validation
  status), `config/robot_template.yaml` (loads from any path with `config:=`; with no TF it stops
  at configure with an actionable extrinsics message), `examples/minimal_backend` built out of tree
  on the `install/` underlay by `scripts/check_consumer.sh` (also a CI step). Export gap fixed:
  `lo_gtsam_adapter` declared `lo_core` only as a test dependency though its CMake needs it.
  Live, yard 1, 40 s, request mode: 262 factors accepted, 0 unmatched, 0 non-SPD, 0 chain breaks.
  New test: all shipped params files (incl. the template) validate with no unknown keys.
  63 tests green. No lo_core change: replay output unchanged.
- **Dev loop** (2026-11-23). No algorithm change. `colcon_defaults.yaml` (Ninja, ccache,
  symlink install, quiet output) shared by the scripts and a bare `colcon build`; ctest runs
  `-j TEST_JOBS` (4); fault suite labelled `slow`, `scripts/test.sh --quick` skips it.
  Laptop, same build: lo_synth tests serial 320 s → `-j4` 101 s (`-j8` 94 s); full `test.sh`
  63 tests ~101 s (was 310–950 s in the logs); `--quick` 50 tests 13 s; Ninja migration
  rebuild 42 s (95 % ccache hits); no-op build < 1 s. Build/test console output 430/1070 →
  ~20/30 lines. CLAUDE.md rewritten (container environment, on-demand docs, long-job rules);
  `docs/ARCHITECTURE.md` expanded into the entry-point mental model.

The plan as approved on 2026-11-21 is kept verbatim in [history/APPROVED_PLAN.md](history/APPROVED_PLAN.md).
