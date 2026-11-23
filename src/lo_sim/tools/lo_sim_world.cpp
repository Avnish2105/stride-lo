// lo_sim_world <scenario> [--seed N] [--start-delay S] [--loop] > world.sdf
// Gazebo world of an lo_synth scenario (static primitives, walkers, the rig) with the
// KinematicRig plugin, so Gazebo shows exactly what lo_sim_rig's sensors see.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "lo_sim/rig_description.hpp"

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::fprintf(stderr, "usage: lo_sim_world <scenario> [--seed N] [--start-delay S] [--loop]\n");
    return 2;
  }
  const std::string name = argv[1];
  std::uint64_t seed = 1;
  double delay = 3.0;
  bool loop = false;
  for (int i = 2; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--seed") && i + 1 < argc) {seed = std::strtoull(argv[++i], nullptr, 10);}
    else if (!std::strcmp(argv[i], "--start-delay") && i + 1 < argc) {delay = std::atof(argv[++i]);}
    else if (!std::strcmp(argv[i], "--loop")) {loop = true;}
  }
  std::string sdf = lo_sim::worldSdf(name, seed, lo::synth::makeScenario(name, seed));
  const std::string key = "<seed>" + std::to_string(seed) + "</seed>";
  sdf.replace(sdf.find(key), key.size(), key + "<start_delay>" + std::to_string(delay) +
    "</start_delay><loop>" + (loop ? "true" : "false") + "</loop>");
  std::fputs(sdf.c_str(), stdout);
  return 0;
}
