// Zero heap allocations per scan after warm-up (docs/DESIGN.md budget table).
#include <gtest/gtest.h>

#include <atomic>
#include <cstdlib>
#include <new>

#include "lo_synth/runner.hpp"

namespace {
std::atomic<bool> g_counting{false};
std::atomic<std::uint64_t> g_allocs{0};
}  // namespace

void * operator new(std::size_t n)
{
  if (g_counting.load(std::memory_order_relaxed)) {g_allocs.fetch_add(1, std::memory_order_relaxed);}
  if (void * p = std::malloc(n ? n : 1)) {return p;}
  throw std::bad_alloc();
}
void * operator new[](std::size_t n) {return operator new(n);}
void * operator new(std::size_t n, std::align_val_t a)
{
  if (g_counting.load(std::memory_order_relaxed)) {g_allocs.fetch_add(1, std::memory_order_relaxed);}
  if (void * p = std::aligned_alloc(static_cast<std::size_t>(a), (n + static_cast<std::size_t>(a) - 1) & ~(static_cast<std::size_t>(a) - 1))) {return p;}
  throw std::bad_alloc();
}
void * operator new[](std::size_t n, std::align_val_t a) {return operator new(n, a);}
void operator delete(void * p) noexcept {std::free(p);}
void operator delete[](void * p) noexcept {std::free(p);}
void operator delete(void * p, std::size_t) noexcept {std::free(p);}
void operator delete[](void * p, std::size_t) noexcept {std::free(p);}
void operator delete(void * p, std::align_val_t) noexcept {std::free(p);}
void operator delete[](void * p, std::align_val_t) noexcept {std::free(p);}
void operator delete(void * p, std::size_t, std::align_val_t) noexcept {std::free(p);}
void operator delete[](void * p, std::size_t, std::align_val_t) noexcept {std::free(p);}

using namespace lo;
using namespace lo::synth;

namespace {
class CountingSink : public Sink
{
public:
  explicit CountingSink(Pipeline & p) : p_(p) {}
  void onImu(const ImuSample & s, TimeNs a) override {p_.addImu(s, a);}
  void onJoints(const JointSample & s, TimeNs) override {p_.addJoints(s);}
  void onRequest(TimeNs t, TimeNs) override {p_.addNodeRequest(t);}
  void onScan(const RawScan & scan, TimeNs arrival, TimeNs) override
  {
    const bool count = ++scans_ > 80;   // warm-up: INIT, first map, OpenMP team creation
    if (count) {g_counting = true;}
    const ScanResult & r = p_.process(scan, arrival);
    if (count) {
      g_counting = false;
      if (r.verdict != Verdict::REJECT) {++measured_;}
    }
  }
  int measured_{0};

private:
  Pipeline & p_;
  int scans_{0};
};
}  // namespace

TEST(Allocations, ProcessDoesNotAllocateAfterWarmUp)
{
  const Scenario sc = makeScenario("crowd", 4);
  const Config cfg = syntheticConfig(sc, true);
  Simulator sim(sc);
  auto kin = std::make_shared<KinematicModel>(sim.robot().model());
  Pipeline pipeline(cfg, extrinsicsFromConfig(cfg), kin);
  CountingSink sink(pipeline);
  sim.run(sink, 25.0);
  EXPECT_GT(sink.measured_, 100);
  EXPECT_EQ(g_allocs.load(), 0u) << "heap allocations inside Pipeline::process after warm-up";
}
