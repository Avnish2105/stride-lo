#!/usr/bin/env bash
# Run all tests of a built workspace (inside the image):
#   scripts/dc run --rm dev scripts/test.sh                       Release build
#   scripts/dc run --rm dev scripts/test.sh --sanitizer=thread    sanitizer build
#   scripts/dc run --rm dev scripts/test.sh --packages-select lo_core
#   scripts/dc run --rm dev scripts/test.sh --quick       skip tests labelled `slow` (the
#                                                         synthetic fault suite): inner loop
# (Inside the dev container drop the `scripts/dc run --rm dev` prefix.)
# Parallelism: TEST_JOBS ctest jobs per package (default 4; each test runs its own OpenMP
# team of 2, results are deterministic and independent of load). VERBOSE=1: full output.
# Re-run only what failed: --packages-select-test-failures (colcon) or, in build/<pkg>,
# `ctest --rerun-failed --output-on-failure`.
set -eo pipefail
ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ws"
sanitizer=""
quick=0
args=()
for a in "$@"; do
  case "$a" in
    --sanitizer=*) sanitizer="${a#*=}" ;;
    --quick) quick=1 ;;
    *) args+=("$a") ;;
  esac
done
suffix=""
[ -n "$sanitizer" ] && suffix="_${sanitizer}"
skip=(small_gicp)
# Sanitizer runs skip lo_gtsam_adapter: the prebuilt GTSAM is not instrumented, and Eigen's
# aligned allocations crossing into it are reported as bad frees (ASan). The adapter's tests
# run in the Release build.
[ -n "$sanitizer" ] && skip+=(lo_gtsam_adapter)
if [ -z "${AMENT_PREFIX_PATH:-}" ] || [ -z "${STRIDE_UNDERLAY:-}" ]; then
  source /usr/local/bin/stride_env.sh
fi
source "install${suffix}/setup.bash"
export OMP_DYNAMIC=false
# CI users may have no writable home: keep ROS logs in the workspace
export ROS_HOME="${ROS_HOME:-${ws}/log${suffix}/ros_home}"
export ASAN_OPTIONS="detect_leaks=1:abort_on_error=1"
export UBSAN_OPTIONS="print_stacktrace=1:halt_on_error=1"
export TSAN_OPTIONS="suppressions=${ws}/scripts/tsan.supp:halt_on_error=1"
ctest_args=(-j "${TEST_JOBS:-4}" --output-on-failure)
[ "$quick" = 1 ] && ctest_args+=(-LE slow)
extra=()
[ "${VERBOSE:-0}" = 1 ] && extra+=(--event-handlers console_cohesion+ status-)
# Base paths, small_gicp skip and quiet output come from colcon_defaults.yaml.
colcon --log-base "log${suffix}" test \
  --packages-skip "${skip[@]}" \
  --build-base "build${suffix}" --install-base "install${suffix}" \
  "${extra[@]}" "${args[@]}" \
  --ctest-args "${ctest_args[@]}"   # last: it swallows every argument after it
colcon test-result --test-result-base "build${suffix}" --verbose
