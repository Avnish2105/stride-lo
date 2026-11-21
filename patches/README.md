# Patches to vendored code

`src/third_party/small_gicp` is pristine upstream (v1.0.1, see its `STRIDE_VENDOR.txt`). It carries
a `COLCON_IGNORE`; `scripts/build.sh` (via `scripts/patch_small_gicp.sh`) copies it to
`.stride_patched/small_gicp` (gitignored), applies these patches there with `patch -p4`, and colcon
builds the copy. The copy is refreshed whenever the vendored tree or a patch changes; a patch that
no longer applies stops the build. Patch paths are relative to the repo root.

| Patch | Target | Why |
|---|---|---|
| `0001-fixed-incremental-voxelmap.patch` | new `ann/fixed_incremental_voxelmap.hpp` | Upstream `IncrementalVoxelMap` allocates a `shared_ptr` + `unordered_map` node per new voxel and re-finalizes every voxel on each insert. The fixed map preallocates the pool and an open-addressing table, removes by time stamp / radius / LRU explicitly, keeps a dense deterministic order and finalizes only touched voxels. Required by "0 allocations per scan" and "deterministic replay". |
| `0002-deterministic-omp-reduction.patch` | `registration/reduction_omp.hpp` | `schedule(guided)` changes the floating-point summation order between runs and three `std::vector`s were allocated per call. Now fixed contiguous blocks per thread, stack partial sums, combined in thread order. |
| `0003-kdtree-rebuild-in-place.patch` | `ann/kdtree.hpp` | `UnsafeKdTree` could only be built in its constructor (and crashed on searches after an empty build). Adds a deferred-build constructor and `rebuild()`, which reuses `indices`/`nodes` capacity, plus empty-tree guards. |
| `0004-geman-mcclure-kernel.patch` | `factors/robust_kernel.hpp` | Adds the Geman-McClure kernel required by the design (scale τ/3), GTSAM convention. |

Each patch defines a marker macro (`SMALL_GICP_STRIDE_FIXED_VOXELMAP`,
`SMALL_GICP_STRIDE_DETERMINISTIC_OMP`, `SMALL_GICP_STRIDE_KDTREE_REBUILD`,
`SMALL_GICP_STRIDE_GEMAN_MCCLURE`); `lo_core/third_party/small_gicp_check.hpp` refuses to compile
against an unpatched small_gicp.

Re-vendoring: `scripts/vendor_small_gicp.sh <tag>`, then `scripts/patch_small_gicp.sh` (or a
build) tells you which patch needs rework.
