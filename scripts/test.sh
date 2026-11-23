#!/usr/bin/env bash
# Run all tests of a built workspace (inside the image):
#   scripts/dc run --rm dev scripts/test.sh                       Release build
#   scripts/dc run --rm dev scripts/test.sh --sanitizer=thread    sanitizer build
#   scripts/dc run --rm dev scripts/test.sh --packages-select lo_core
set -eo pipefail
ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ws"
sanitizer=""
args=()
for a in "$@"; do
  case "$a" in
    --sanitizer=*) sanitizer="${a#*=}" ;;
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
colcon --log-base "log${suffix}" test \
  --base-paths src .stride_patched --packages-skip "${skip[@]}" \
  --build-base "build${suffix}" --install-base "install${suffix}" \
  --event-handlers console_cohesion+ --return-code-on-test-failure \
  "${args[@]}"
colcon test-result --test-result-base "build${suffix}" --verbose
