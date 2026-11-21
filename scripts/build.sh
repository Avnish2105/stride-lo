#!/usr/bin/env bash
# Build the STRIDE workspace. Run inside the image:
#   scripts/dc run --rm dev scripts/build.sh                         Release, whole workspace
#   scripts/dc run --rm dev scripts/build.sh --sanitizer=address     ASan into build_address/ ...
#   scripts/dc run --rm dev scripts/build.sh --packages-select lo_core   (extra args -> colcon)
# Sanitizers: address | undefined | thread. Parallelism: BUILD_JOBS (default 3).
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
export MAKEFLAGS="-j${jobs}"
suffix=""
[ -n "$sanitizer" ] && suffix="_${sanitizer}"
cmake_args=(-DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
            "-DSTRIDE_SANITIZER=${sanitizer}")
if command -v ccache >/dev/null 2>&1; then
  cmake_args+=(-DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache)
fi

colcon --log-base "log${suffix}" build \
  --base-paths src .stride_patched \
  --build-base "build${suffix}" --install-base "install${suffix}" \
  --symlink-install --parallel-workers "${BUILD_WORKERS:-2}" \
  --event-handlers console_cohesion+ \
  --cmake-args "${cmake_args[@]}" \
  "${args[@]}"

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
