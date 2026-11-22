// Primitive 3D scene with analytic ray casting (lo_synth: deterministic synthetic sensor data).
// Static primitives: oriented boxes, vertical cylinders, bounded planes. Moving primitives:
// vertical cylinders following looping piecewise-linear paths (a crowd).
#pragma once

#include <vector>

#include "lo_core/types/se3.hpp"

namespace lo::synth {

struct Hit
{
  double range{0.0};
  float reflectivity{0.f};
};

struct Box
{
  SE3 pose;                 ///< world <- box (centre)
  Vec3 half{0.5, 0.5, 0.5};
  float reflectivity{50.f};
};

struct Cylinder
{
  double x{0}, y{0}, radius{0.3}, z_min{0}, z_max{2.0};
  float reflectivity{60.f};
};

/// Rectangle in a plane: centre c, unit axes u, v (normal = u x v), half sizes hu, hv.
struct Rect
{
  Vec3 c{Vec3::Zero()};
  Vec3 u{Vec3::UnitX()};
  Vec3 v{Vec3::UnitY()};
  double hu{1.0}, hv{1.0};
  float reflectivity{40.f};
};

/// A cylinder walking along a closed polyline at constant speed (time in seconds).
struct Walker
{
  std::vector<Vec3> path;   ///< xy used, z ignored
  double speed{1.2};
  double radius{0.25};
  double height{1.8};
  double phase{0.0};        ///< seconds
  float reflectivity{35.f};
  Vec3 positionAt(double t) const;
};

class Scene
{
public:
  std::vector<Box> boxes;
  std::vector<Cylinder> cylinders;
  std::vector<Rect> rects;
  std::vector<Walker> walkers;

  /// Nearest hit along origin + s * dir (dir unit) for s in (min_range, max_range), at time t.
  bool raycast(const Vec3 & origin, const Vec3 & dir, double min_range, double max_range, double t,
    Hit & hit) const;

  // builders
  void addFloor(double z, double half_size, float refl = 30.f);
  void addRoom(const Vec3 & centre, const Vec3 & size, bool ceiling = true, float refl = 45.f);
  void addBox(const Vec3 & centre, const Vec3 & size, double yaw = 0.0, float refl = 50.f);
};

}  // namespace lo::synth
