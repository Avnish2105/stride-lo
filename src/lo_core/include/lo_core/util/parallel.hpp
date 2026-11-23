// Deterministic OpenMP helpers: a fixed team, every thread works on one contiguous block of the
// index range, results are combined by the caller in thread order (never by OpenMP reduction).
#pragma once

#include <cstdint>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace lo {

/// Call once per process before the first parallel region: no dynamic team resizing.
inline void initParallel()
{
#ifdef _OPENMP
  omp_set_dynamic(0);
#endif
}

/// f(begin, end, thread_index) for contiguous blocks of [0, n). Returns the team size used.
template<typename F>
int parallelBlocks(std::int64_t n, int threads, F && f)
{
  int team_size = 1;
#ifdef _OPENMP
  if (threads > 1 && n > 256) {
#pragma omp parallel num_threads(threads)
    {
      const int t = omp_get_thread_num();
      const int team = omp_get_num_threads();
      if (t == 0) {team_size = team;}
      f(n * t / team, n * (t + 1) / team, t);
    }
    return team_size;
  }
#endif
  (void)threads;
  f(0, n, 0);
  return team_size;
}

}  // namespace lo
