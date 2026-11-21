#!/usr/bin/env bash
# Create/refresh .stride_patched/small_gicp = src/third_party/small_gicp + patches/small_gicp/*.
# Idempotent; called by build.sh. Fails loudly if a patch no longer applies (re-vendored code).
set -eo pipefail
ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
src="$ws/src/third_party/small_gicp"
dst="$ws/.stride_patched/small_gicp"
stamp_file="$ws/.stride_patched/.stamp"
stamp="$( (cd "$ws" && find src/third_party/small_gicp patches/small_gicp -type f -print0 \
           | sort -z | xargs -0 sha256sum) | sha256sum | cut -d' ' -f1)"
if [ -f "$stamp_file" ] && [ "$(cat "$stamp_file")" = "$stamp" ] && [ -d "$dst" ]; then
  exit 0
fi
rm -rf "$dst"
mkdir -p "$(dirname "$dst")"
cp -a "$src" "$dst"
rm -f "$dst/COLCON_IGNORE"
for p in "$ws"/patches/small_gicp/*.patch; do
  # Patch paths are relative to the repo root (a/src/third_party/small_gicp/...): strip 4.
  if ! patch -s -p4 -d "$dst" --forward --batch < "$p"; then
    echo "patch_small_gicp.sh: ERROR: $(basename "$p") does not apply (vendored code changed?)" >&2
    rm -rf "$dst"
    exit 1
  fi
done
echo "$stamp" > "$stamp_file"
echo "patch_small_gicp.sh: .stride_patched/small_gicp refreshed ($(ls "$ws"/patches/small_gicp/*.patch | wc -l) patches)"
