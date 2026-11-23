// Refuse to build lo_core against an unpatched small_gicp (patches/small_gicp, DECISIONS D6).
#pragma once

#if !__has_include(<small_gicp/ann/fixed_incremental_voxelmap.hpp>)
#error "small_gicp is not patched (no fixed_incremental_voxelmap.hpp): build with scripts/build.sh"
#endif

#include <small_gicp/ann/fixed_incremental_voxelmap.hpp>
#include <small_gicp/ann/kdtree.hpp>
#include <small_gicp/factors/robust_kernel.hpp>
#include <small_gicp/registration/reduction_omp.hpp>

#if !defined(SMALL_GICP_STRIDE_FIXED_VOXELMAP) || !defined(SMALL_GICP_STRIDE_DETERMINISTIC_OMP) || \
  !defined(SMALL_GICP_STRIDE_KDTREE_REBUILD) || !defined(SMALL_GICP_STRIDE_GEMAN_MCCLURE)
#error "small_gicp is missing STRIDE patches 0001-0004 (see patches/README.md)"
#endif
