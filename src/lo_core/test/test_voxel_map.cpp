// Patched small_gicp FixedIncrementalVoxelMap (patch 0001) with lo::MapVoxel.
#include <gtest/gtest.h>

#include <set>

#include "lo_core/map/local_map.hpp"
#include "test_common.hpp"

using namespace lo;

namespace {
Cloud makeCloud(const std::vector<Vec3> & pts)
{
  Cloud c(pts.size());
  for (const auto & p : pts) {c.push(p, 0.0);}
  for (std::size_t i = 0; i < c.size(); ++i) {c.normals[i] = Vec3::UnitZ();}
  return c;
}
}  // namespace

TEST(FixedVoxelMap, InsertFindNearestAndCap)
{
  VoxelMap map(0.5, 1000);
  map.voxel_setting.max_points = 3;
  std::vector<Vec3> pts;
  for (int i = 0; i < 10; ++i) {pts.emplace_back(0.1, 0.1, 0.1);}   // same voxel
  pts.emplace_back(2.2, 0.1, 0.1);
  const Cloud c = makeCloud(pts);
  EXPECT_EQ(map.insert(c, Eigen::Isometry3d::Identity(), 5), 0u);
  EXPECT_EQ(map.size(), 2u);
  const auto idx = map.find(Eigen::Vector3i(0, 0, 0));
  ASSERT_GE(idx, 0);
  EXPECT_EQ(map.voxels[idx].count, 3u);   // capped
  EXPECT_EQ(map.infos[idx].last_hit, 5);
  size_t k;
  double d2;
  map.set_search_offsets(27);
  ASSERT_EQ(map.nearest_neighbor_search(Vec4(0.6, 0.1, 0.1, 1.0), &k, &d2), 1u);
  EXPECT_EQ(map.voxel_id(k), static_cast<size_t>(idx));
}

TEST(FixedVoxelMap, StaleRadiusAndLruEvictionKeepTableConsistent)
{
  std::mt19937_64 rng(5);
  std::uniform_real_distribution<double> u(-20.0, 20.0);
  VoxelMap map(0.3, 2000);
  for (int round = 0; round < 30; ++round) {
    std::vector<Vec3> pts;
    for (int i = 0; i < 150; ++i) {pts.emplace_back(u(rng), u(rng), u(rng) * 0.1);}
    map.insert(makeCloud(pts), Eigen::Isometry3d::Identity(), round);
    if (round % 7 == 6) {map.remove_stale(round - 5, Vec3::Zero(), 15.0);}
    ASSERT_LE(map.size(), map.capacity());
    // every voxel is reachable through the hash table at its own index
    std::set<std::tuple<int, int, int>> seen;
    for (size_t i = 0; i < map.size(); ++i) {
      const auto & c = map.infos[i].coord;
      ASSERT_EQ(map.find(c), static_cast<std::int64_t>(i));
      ASSERT_TRUE(seen.insert({c.x(), c.y(), c.z()}).second);
    }
  }
  // explicit LRU eviction removes the oldest
  const std::int64_t oldest_before = [&] {
      std::int64_t o = INT64_MAX;
      for (size_t i = 0; i < map.size(); ++i) {o = std::min(o, map.infos[i].last_hit);}
      return o;
    }();
  map.evict_lru(map.size() / 2);
  std::int64_t oldest_after = INT64_MAX;
  for (size_t i = 0; i < map.size(); ++i) {oldest_after = std::min(oldest_after, map.infos[i].last_hit);}
  EXPECT_GE(oldest_after, oldest_before);
}

TEST(FixedVoxelMap, DeterministicOrder)
{
  auto build = [] {
      std::mt19937_64 rng(6);
      std::uniform_real_distribution<double> u(-10.0, 10.0);
      VoxelMap map(0.25, 5000);
      for (int r = 0; r < 20; ++r) {
        std::vector<Vec3> pts;
        for (int i = 0; i < 300; ++i) {pts.emplace_back(u(rng), u(rng), u(rng));}
        map.insert(makeCloud(pts), Eigen::Isometry3d::Identity(), r);
        if (r % 5 == 4) {map.remove_stale(r - 3, Vec3::Zero(), 9.0);}
      }
      std::vector<Eigen::Vector3i> order;
      for (size_t i = 0; i < map.size(); ++i) {order.push_back(map.infos[i].coord);}
      return order;
    };
  EXPECT_EQ(build(), build());
}

TEST(LocalMap, RebinKeepsContent)
{
  Config cfg;
  cfg.limits.map_fine_capacity = 20000;
  cfg.limits.map_coarse_capacity = 5000;
  LocalMap map(cfg);
  map.reset(0.3);
  std::mt19937_64 rng(7);
  const auto pts = test::roomPoints(8, 6, 3, 0.1, rng);
  const Cloud c = makeCloud(pts);
  map.insert(c, SE3(), 1);
  const size_t before = map.fine().size();
  EXPECT_TRUE(map.needsRebin(0.45));
  map.rebin(0.45);
  EXPECT_NEAR(map.baseVoxel(), 0.45, 1e-12);
  EXPECT_GT(map.fine().size(), before / 4);
  EXPECT_LT(map.fine().size(), before);
}
