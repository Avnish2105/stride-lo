// Fixed-window statistics without allocation after construction.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace lo {

/// Last N values; median / percentiles via nth_element on a preallocated scratch copy.
class RollingWindow
{
public:
  explicit RollingWindow(std::size_t capacity = 1) : values_(capacity < 1 ? 1 : capacity), scratch_(values_.size()) {}

  void push(double v)
  {
    values_[next_] = v;
    next_ = (next_ + 1) % values_.size();
    if (size_ < values_.size()) {++size_;}
  }
  void clear() {size_ = 0; next_ = 0;}
  std::size_t size() const {return size_;}

  /// p in [0, 1]; 0 if empty.
  double percentile(double p)
  {
    if (size_ == 0) {return 0.0;}
    std::copy(values_.begin(), values_.begin() + static_cast<std::ptrdiff_t>(size_), scratch_.begin());
    const std::size_t k = std::min(size_ - 1,
      static_cast<std::size_t>(std::lround(p * static_cast<double>(size_ - 1))));
    std::nth_element(scratch_.begin(), scratch_.begin() + static_cast<std::ptrdiff_t>(k),
      scratch_.begin() + static_cast<std::ptrdiff_t>(size_));
    return scratch_[k];
  }
  double median() {return percentile(0.5);}
  double max() const
  {
    double m = 0.0;
    for (std::size_t i = 0; i < size_; ++i) {m = std::max(m, values_[i]);}
    return m;
  }

private:
  std::vector<double> values_;
  std::vector<double> scratch_;
  std::size_t next_{0};
  std::size_t size_{0};
};

}  // namespace lo
