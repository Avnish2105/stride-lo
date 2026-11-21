#!/usr/bin/env bash
# Re-vendor small_gicp (pristine upstream) into src/third_party/small_gicp.
#   scripts/vendor_small_gicp.sh [tag]      default tag: v1.0.1
# Afterwards: scripts/build.sh applies patches/small_gicp/*.patch to a copy
# (.stride_patched/small_gicp) and fails if one no longer applies. Provenance is written to src/third_party/small_gicp/STRIDE_VENDOR.txt.
set -euo pipefail
tag="${1:-v1.0.1}"
url="https://github.com/koide3/small_gicp.git"
ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
dst="$ws/src/third_party/small_gicp"
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

git clone -q --depth 1 --branch "$tag" "$url" "$tmp/src"
commit="$(git -C "$tmp/src" rev-parse HEAD)"
# Prune what our build never uses (tests data, benchmarks, python, docs, CI, assets).
rm -rf "$tmp/src/.git" "$tmp/src/.github" "$tmp/src/data" "$tmp/src/docs" \
       "$tmp/src/src/benchmark" "$tmp/src/src/python" "$tmp/src/src/example" \
       "$tmp/src/include/small_gicp/benchmark" "$tmp/src/src/test"
rm -rf "$dst"
mkdir -p "$(dirname "$dst")"
cp -a "$tmp/src" "$dst"
printf '# Built from .stride_patched/small_gicp (pristine copy + patches/small_gicp/*): see scripts/build.sh\n' > "$dst/COLCON_IGNORE"
cat > "$dst/STRIDE_VENDOR.txt" <<INFO
Upstream: $url
Tag:      $tag
Commit:   $commit
Vendored: $(date -u +%Y-%m-%d) by scripts/vendor_small_gicp.sh (pristine; pruned: .git .github
          data docs src/benchmark src/python src/example src/test include/small_gicp/benchmark)
Patches:  patches/small_gicp/*.patch, applied by scripts/build.sh (see patches/README.md)
License:  MIT (LICENSE in this directory)
INFO
echo "vendored small_gicp $tag ($commit) -> $dst"
