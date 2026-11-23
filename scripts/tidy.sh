#!/usr/bin/env bash
# clang-tidy over STRIDE's own C++ (inside the image, after scripts/build.sh produced
# compile_commands.json). Exit status != 0 on any WarningsAsErrors finding.
#   scripts/dc run --rm dev scripts/tidy.sh [regex]
set -eo pipefail
ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ws"
if [ ! -f compile_commands.json ]; then
  echo "tidy.sh: no compile_commands.json (run scripts/build.sh first)" >&2
  exit 2
fi
filter="${1:-src/lo_(core|ros|synth|tools|gtsam_adapter)/(src|tools)/.*\.cpp$}"
jobs="${BUILD_JOBS:-3}"
# The database comes from GCC builds: let clang ignore GCC-only warning flags and -Werror.
# OpenMP: GCC's omp.h does not parse with clang; analyse the code paths without OpenMP
# (all OpenMP use is behind #ifdef _OPENMP, lo_core/util/parallel.hpp).
run-clang-tidy -p "$ws" -j "$jobs" -quiet \
  -extra-arg=-Wno-unknown-warning-option -extra-arg=-Wno-error -extra-arg=-fno-openmp \
  "$filter"
