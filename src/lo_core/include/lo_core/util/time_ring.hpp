// Fixed-capacity ring of time-stamped samples (oldest overwritten). Single-threaded. Samples
// must arrive with strictly increasing stamps; a non-increasing stamp is rejected (duplicate
// or out-of-order sensor samples are dropped, never re-sorted).
#pragma once

#include <cstddef>
#include <vector>

#include "lo_core/types/time.hpp"

namespace lo {

template<typename T>
class TimeRing
{
public:
  explicit TimeRing(std::size_t capacity = 2) : data_(capacity < 2 ? 2 : capacity) {}

  /// false if t <= newest stamp (sample dropped).
  bool push(const T & v)
  {
    if (size_ > 0 && v.t <= back().t) {
      ++rejected_;
      return false;
    }
    data_[(head_ + size_) % data_.size()] = v;
    if (size_ < data_.size()) {
      ++size_;
    } else {
      head_ = (head_ + 1) % data_.size();
    }
    return true;
  }

  void clear() {head_ = 0; size_ = 0;}
  std::size_t size() const {return size_;}
  std::size_t capacity() const {return data_.size();}
  bool empty() const {return size_ == 0;}
  const T & operator[](std::size_t i) const {return data_[(head_ + i) % data_.size()];}
  const T & front() const {return (*this)[0];}
  const T & back() const {return (*this)[size_ - 1];}
  std::size_t rejected() const {return rejected_;}

  /// Index of the first sample with stamp >= t (size() if none).
  std::size_t lowerBound(TimeNs t) const
  {
    std::size_t lo = 0, hi = size_;
    while (lo < hi) {
      const std::size_t mid = (lo + hi) / 2;
      if ((*this)[mid].t < t) {lo = mid + 1;} else {hi = mid;}
    }
    return lo;
  }

  /// Bracketing samples a <= t <= b; false if t is outside the buffered span.
  bool bracket(TimeNs t, std::size_t & ia, std::size_t & ib) const
  {
    if (size_ == 0 || t < front().t || t > back().t) {return false;}
    const std::size_t i = lowerBound(t);
    if (i < size_ && (*this)[i].t == t) {ia = ib = i; return true;}
    ia = i - 1;
    ib = i;
    return true;
  }

private:
  std::vector<T> data_;
  std::size_t head_{0};
  std::size_t size_{0};
  std::size_t rejected_{0};
};

}  // namespace lo
