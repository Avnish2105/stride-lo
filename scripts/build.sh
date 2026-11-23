#!/usr/bin/env bash
# Build the STRIDE workspace. Run inside the image:
#   scripts/dc run --rm dev scripts/build.sh                         Release, whole workspace
#   scripts/dc run --rm dev scripts/build.sh --sanitizer=address     ASan into build_address/ ...
#   scripts/dc run --rm dev scripts/build.sh --packages-select lo_core   (extra args -> colcon)
# (Inside the dev container drop the `scripts/dc run --rm dev` prefix.)
# Sanitizers: address | undefined | thread. Parallelism: BUILD_JOBS compile jobs (default 3) per
# package x BUILD_WORKERS packages (default 2, colcon_defaults.yaml). VERBOSE=1: full output.
# Useful colcon selections (any colcon build option is passed through):
#   --packages-select lo_core          just this package
#   --packages-above lo_core           this package and everything that depends on it
#   --packages-up-to lo_tools          this package and its dependencies
#   --packages-select-build-failed     retry what failed last time
#
# small_gicp: src/third_party/small_gicp stays pristine (it has a COLCON_IGNORE). This script
# copies it to .stride_patched/small_gicp, applies patches/small_gicp/*.patch there, and colcon
# builds that copy. The copy is refreshed whenever the vendored tree or a patch changes.
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
case "$sanitizer" in
  ""|address|undefined|thread) ;;
  *) echo "build.sh: unknown sanitizer '$sanitizer' (address|undefined|thread)" >&2; exit 2 ;;
esac

# ---- patched small_gicp copy ------------------------------------------------------------
"$ws/scripts/patch_small_gicp.sh"

# ---- environment --------------------------------------------------------------------------
if [ -z "${AMENT_PREFIX_PATH:-}" ] || [ -z "${STRIDE_UNDERLAY:-}" ]; then
  source /usr/local/bin/stride_env.sh
fi

jobs="${BUILD_JOBS:-3}"
# Ninja ignores MAKEFLAGS; `cmake --build` passes CMAKE_BUILD_PARALLEL_LEVEL on as -j. Keep the
# MAKEFLAGS -j too: it stops colcon from adding its own -j<nproc>.
export CMAKE_BUILD_PARALLEL_LEVEL="${jobs}" MAKEFLAGS="-j${jobs}"
suffix=""
[ -n "$sanitizer" ] && suffix="_${sanitizer}"
build_base="build${suffix}"

# Generator migration (Unix Makefiles -> Ninja, colcon_defaults.yaml): CMake refuses to switch
# the generator of an existing build dir, so drop those package build dirs once (cache only).
for cache in "$build_base"/*/CMakeCache.txt; do
  [ -f "$cache" ] || continue
  if ! grep -q '^CMAKE_GENERATOR:INTERNAL=Ninja$' "$cache"; then
    echo "build.sh: $(dirname "$cache") was configured with another generator; re-creating it"
    rm -rf "$(dirname "$cache")"
  fi
done

# Common options (base paths, symlink install, Ninja, ccache, quiet output) come from
# colcon_defaults.yaml. Only the sanitizer build overrides paths and CMake arguments.
# VERBOSE=1 prints every package's full output (console_cohesion) instead of errors only.
extra=()
if [ -n "$sanitizer" ]; then
  extra+=(--build-base "$build_base" --install-base "install${suffix}"
          --cmake-args -GNinja -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
          "-DSTRIDE_SANITIZER=${sanitizer}"
          -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache)
fi
[ "${VERBOSE:-0}" = 1 ] && extra+=(--event-handlers console_cohesion+ status-)
[ -n "${BUILD_WORKERS:-}" ] && extra+=(--parallel-workers "$BUILD_WORKERS")

colcon --log-base "log${suffix}" build "${extra[@]}" "${args[@]}"

# One compile_commands.json at the root for the editor (Release build only).
if [ -z "$sanitizer" ]; then
  python3 - "$ws" <<'PY'
import json, pathlib, sys
ws = pathlib.Path(sys.argv[1])
cmds = []
for f in sorted((ws / "build").glob("*/compile_commands.json")):
    cmds += json.loads(f.read_text())
(ws / "compile_commands.json").write_text(json.dumps(cmds, indent=1))
PY
fi
