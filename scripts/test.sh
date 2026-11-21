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
if [ -z "${AMENT_PREFIX_PATH:-}" ] || [ -z "${STRIDE_UNDERLAY:-}" ]; then
  source /usr/local/bin/stride_env.sh
fi
source "install${suffix}/setup.bash"
export OMP_DYNAMIC=false
export ASAN_OPTIONS="detect_leaks=1:abort_on_error=1"
export UBSAN_OPTIONS="print_stacktrace=1:halt_on_error=1"
export TSAN_OPTIONS="suppressions=${ws}/scripts/tsan.supp:halt_on_error=1"
colcon --log-base "log${suffix}" test \
  --base-paths src .stride_patched --packages-skip small_gicp \
  --build-base "build${suffix}" --install-base "install${suffix}" \
  --event-handlers console_cohesion+ --return-code-on-test-failure \
  "${args[@]}"
colcon test-result --test-result-base "build${suffix}" --verbose
