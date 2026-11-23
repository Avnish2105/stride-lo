// Set of packed voxel keys with O(1) clear (generation stamps). Fixed capacity, no allocation
// after construction, deterministic. Used for the S2 voxel downsampling and the S9 overlap test.
#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Core>

namespace lo {

/// Packs a voxel coordinate (each axis in [-2^20, 2^20)) into 63 bits.
inline std::uint64_t packVoxel(int x, int y, int z)
{
  constexpr std::int64_t off = 1 << 20;
  constexpr std::uint64_t m = (1u << 21) - 1;
  return (static_cast<std::uint64_t>(x + off) & m) |
         ((static_cast<std::uint64_t>(y + off) & m) << 21) |
         ((static_cast<std::uint64_t>(z + off) & m) << 42);
}

inline Eigen::Vector3i voxelOf(const Eigen::Vector3d & p, double inv_size)
{
  return Eigen::Vector3i(static_cast<int>(std::floor(p.x() * inv_size)),
    static_cast<int>(std::floor(p.y() * inv_size)), static_cast<int>(std::floor(p.z() * inv_size)));
}

class KeySet
{
public:
  explicit KeySet(std::size_t capacity = 16)
  {
    std::size_t n = 16;
    while (n < 2 * capacity) {n <<= 1;}
    keys_.assign(n, 0);
    gens_.assign(n, 0);
    mask_ = n - 1;
  }

  void clear()
  {
    ++gen_;
    size_ = 0;
    if (gen_ == 0) {   // wrapped: really clear once every 2^32 clears
      std::fill(gens_.begin(), gens_.end(), 0);
      gen_ = 1;
    }
  }

  /// true if newly inserted, false if present or the set is full.
  bool insert(std::uint64_t key)
  {
    if (2 * (size_ + 1) > keys_.size()) {return false;}
    std::size_t slot = hash(key) & mask_;
    while (gens_[slot] == gen_) {
      if (keys_[slot] == key) {return false;}
      slot = (slot + 1) & mask_;
    }
    gens_[slot] = gen_;
    keys_[slot] = key;
    ++size_;
    return true;
  }

  bool contains(std::uint64_t key) const
  {
    std::size_t slot = hash(key) & mask_;
    while (gens_[slot] == gen_) {
      if (keys_[slot] == key) {return true;}
      slot = (slot + 1) & mask_;
    }
    return false;
  }

  std::size_t size() const {return size_;}

private:
  static std::size_t hash(std::uint64_t k)
  {
    k ^= k >> 33;
    k *= 0xff51afd7ed558ccdULL;
    k ^= k >> 33;
    k *= 0xc4ceb9fe1a85ec53ULL;
    k ^= k >> 33;
    return static_cast<std::size_t>(k);
  }

  std::vector<std::uint64_t> keys_;
  std::vector<std::uint32_t> gens_;
  std::size_t mask_{0};
  std::size_t size_{0};
  std::uint32_t gen_{1};
};

}  // namespace lo
