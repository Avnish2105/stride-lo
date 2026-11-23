// Lock-free single-producer / single-consumer queue with fixed capacity (power of two).
// Producer: the ROS I/O thread. Consumer: the pipeline worker. When full, push() fails and
// the caller counts the drop (a stalled consumer then sees an input gap, which the pipeline
// already handles); nothing ever blocks or allocates after construction.
#pragma once

#include <atomic>
#include <cstddef>
#include <vector>

namespace lo {

template<typename T>
class SpscQueue
{
public:
  explicit SpscQueue(std::size_t capacity)
  {
    std::size_t n = 2;
    while (n < capacity) {n <<= 1;}
    buffer_.resize(n);
    mask_ = n - 1;
  }

  bool push(const T & v)
  {
    const std::size_t head = head_.load(std::memory_order_relaxed);
    const std::size_t tail = tail_.load(std::memory_order_acquire);
    if (head - tail >= buffer_.size()) {
      dropped_.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
    buffer_[head & mask_] = v;
    head_.store(head + 1, std::memory_order_release);
    return true;
  }

  bool pop(T & out)
  {
    const std::size_t tail = tail_.load(std::memory_order_relaxed);
    const std::size_t head = head_.load(std::memory_order_acquire);
    if (tail == head) {return false;}
    out = buffer_[tail & mask_];
    tail_.store(tail + 1, std::memory_order_release);
    return true;
  }

  std::size_t dropped() const {return dropped_.load(std::memory_order_relaxed);}
  std::size_t capacity() const {return buffer_.size();}

private:
  std::vector<T> buffer_;
  std::size_t mask_{0};
  alignas(64) std::atomic<std::size_t> head_{0};
  alignas(64) std::atomic<std::size_t> tail_{0};
  std::atomic<std::size_t> dropped_{0};
};

}  // namespace lo
