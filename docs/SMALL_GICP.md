# small_gicp (vendored)

| | |
|---|---|
| Upstream | https://github.com/koide3/small_gicp |
| Tag / commit | `v1.0.1` / `57c1106daf83c2c79ee0c58a9c7ed0032298ff4e` |
| License | MIT (`src/third_party/small_gicp/LICENSE`); KdTree is nanoflann-inspired (BSD notice in the header) |
| Vendored by | `scripts/vendor_small_gicp.sh` (pruned: `.git .github data docs src/{benchmark,python,example,test} include/small_gicp/benchmark`) |
| Patches | `patches/small_gicp/*.patch`, see [patches/README.md](../patches/README.md) |
| Build flags | `colcon.meta`: helper library on, OpenMP on, TBB/PCL/Python/tests/benchmarks off, no `-march=native` |

## What STRIDE uses
- `UnsafeKdTree` + serial `KdTreeBuilder` (rebuilt in place per scan, patch 0003) for the S4 kNN.
- `FixedIncrementalVoxelMap` (patch 0001) as the container of both local-map levels; the voxel
  contents type (`lo::MapVoxel`: Gaussian, oriented normal, capped count) lives in lo_core.
- The GICP factor math conventions (right perturbation, rotation-first `se3_exp`) and the
  deterministic `ParallelReductionOMP` (patch 0002) driving lo_core's own factor and its
  degeneracy-projected Gauss-Newton optimizer.
- `GemanMcClure` (patch 0004) for the robust weights.

Not used: upstream `IncrementalVoxelMap`, `Registration::align` (allocates its factor vector per
call), the TBB/PCL code paths, the Python bindings.

## Upgrading
1. `scripts/vendor_small_gicp.sh <new-tag>`
2. `scripts/patch_small_gicp.sh` — fix any patch that no longer applies (regenerate with
   `diff -ruN a/src/third_party/small_gicp/... b/...`).
3. Full build + tests + determinism check, then update the tag/commit above.
