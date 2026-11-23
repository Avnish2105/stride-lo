#include "lo_synth/scene.hpp"

#include <cmath>
#include <limits>

namespace lo::synth {

Vec3 Walker::positionAt(double t) const
{
  if (path.size() < 2) {return path.empty() ? Vec3::Zero() : path[0];}
  double total = 0.0;
  for (std::size_t i = 0; i < path.size(); ++i) {
    total += (path[(i + 1) % path.size()] - path[i]).head<2>().norm();
  }
  if (total <= 0.0) {return path[0];}
  double s = std::fmod((t + phase) * speed, total);
  if (s < 0) {s += total;}
  for (std::size_t i = 0; i < path.size(); ++i) {
    const Vec3 a = path[i];
    const Vec3 b = path[(i + 1) % path.size()];
    const double len = (b - a).head<2>().norm();
    if (s <= len) {return a + (len > 0 ? s / len : 0.0) * (b - a);}
    s -= len;
  }
  return path[0];
}

namespace {

bool rayBox(const Box & b, const Vec3 & o, const Vec3 & d, double lo, double hi, double & s)
{
  const SE3 inv = b.pose.inverse();
  const Vec3 ol = inv * o;
  const Vec3 dl = inv.so3() * d;
  double t0 = lo, t1 = hi;
  for (int k = 0; k < 3; ++k) {
    if (std::abs(dl[k]) < 1e-12) {
      if (ol[k] < -b.half[k] || ol[k] > b.half[k]) {return false;}
      continue;
    }
    double a = (-b.half[k] - ol[k]) / dl[k];
    double c = (b.half[k] - ol[k]) / dl[k];
    if (a > c) {std::swap(a, c);}
    t0 = std::max(t0, a);
    t1 = std::min(t1, c);
    if (t0 > t1) {return false;}
  }
  // ray starting inside the box: report the exit (wall seen from inside)
  s = t0 > lo ? t0 : t1;
  return s > lo && s < hi;
}

bool rayCylinder(double cx, double cy, double r, double z0, double z1, const Vec3 & o, const Vec3 & d,
  double lo, double hi, double & s)
{
  const double ox = o.x() - cx, oy = o.y() - cy;
  const double a = d.x() * d.x() + d.y() * d.y();
  double best = std::numeric_limits<double>::infinity();
  if (a > 1e-12) {
    const double b = 2.0 * (ox * d.x() + oy * d.y());
    const double c = ox * ox + oy * oy - r * r;
    const double disc = b * b - 4 * a * c;
    if (disc >= 0) {
      const double sq = std::sqrt(disc);
      for (double t : {(-b - sq) / (2 * a), (-b + sq) / (2 * a)}) {
        if (t > lo && t < hi) {
          const double z = o.z() + t * d.z();
          if (z >= z0 && z <= z1) {best = std::min(best, t);}
        }
      }
    }
  }
  // caps
  if (std::abs(d.z()) > 1e-12) {
    for (double zc : {z0, z1}) {
      const double t = (zc - o.z()) / d.z();
      if (t > lo && t < hi) {
        const double x = ox + t * d.x(), y = oy + t * d.y();
        if (x * x + y * y <= r * r) {best = std::min(best, t);}
      }
    }
  }
  if (!std::isfinite(best)) {return false;}
  s = best;
  return true;
}

bool rayRect(const Rect & q, const Vec3 & o, const Vec3 & d, double lo, double hi, double & s)
{
  const Vec3 n = q.u.cross(q.v);
  const double den = n.dot(d);
  if (std::abs(den) < 1e-12) {return false;}
  const double t = n.dot(q.c - o) / den;
  if (t <= lo || t >= hi) {return false;}
  const Vec3 p = o + t * d - q.c;
  if (std::abs(p.dot(q.u)) > q.hu || std::abs(p.dot(q.v)) > q.hv) {return false;}
  s = t;
  return true;
}

}  // namespace

bool Scene::raycast(const Vec3 & o, const Vec3 & d, double min_range, double max_range, double t,
  Hit & hit) const
{
  double best = max_range;
  float refl = 0.f;
  bool found = false;
  double s = 0.0;
  for (const auto & b : boxes) {
    if (rayBox(b, o, d, min_range, best, s)) {best = s; refl = b.reflectivity; found = true;}
  }
  for (const auto & c : cylinders) {
    if (rayCylinder(c.x, c.y, c.radius, c.z_min, c.z_max, o, d, min_range, best, s)) {
      best = s; refl = c.reflectivity; found = true;
    }
  }
  for (const auto & q : rects) {
    if (rayRect(q, o, d, min_range, best, s)) {best = s; refl = q.reflectivity; found = true;}
  }
  for (const auto & w : walkers) {
    const Vec3 p = w.positionAt(t);
    if (rayCylinder(p.x(), p.y(), w.radius, 0.0, w.height, o, d, min_range, best, s)) {
      best = s; refl = w.reflectivity; found = true;
    }
  }
  if (!found) {return false;}
  hit.range = best;
  hit.reflectivity = refl;
  return true;
}

void Scene::addFloor(double z, double half_size, float refl)
{
  rects.push_back(Rect{Vec3(0, 0, z), Vec3::UnitX(), Vec3::UnitY(), half_size, half_size, refl});
}

void Scene::addRoom(const Vec3 & c, const Vec3 & size, bool ceiling, float refl)
{
  const double hx = size.x() / 2, hy = size.y() / 2, hz = size.z() / 2;
  const double zc = c.z() + hz;
  rects.push_back(Rect{Vec3(c.x() + hx, c.y(), zc), Vec3::UnitY(), Vec3::UnitZ(), hy, hz, refl});
  rects.push_back(Rect{Vec3(c.x() - hx, c.y(), zc), Vec3::UnitY(), Vec3::UnitZ(), hy, hz, refl});
  rects.push_back(Rect{Vec3(c.x(), c.y() + hy, zc), Vec3::UnitX(), Vec3::UnitZ(), hx, hz, refl});
  rects.push_back(Rect{Vec3(c.x(), c.y() - hy, zc), Vec3::UnitX(), Vec3::UnitZ(), hx, hz, refl});
  rects.push_back(Rect{Vec3(c.x(), c.y(), c.z()), Vec3::UnitX(), Vec3::UnitY(), hx, hy, refl * 0.8f});
  if (ceiling) {
    rects.push_back(Rect{Vec3(c.x(), c.y(), c.z() + size.z()), Vec3::UnitX(), Vec3::UnitY(), hx, hy, refl});
  }
}

void Scene::addBox(const Vec3 & centre, const Vec3 & size, double yaw, float refl)
{
  Box b;
  b.pose = SE3(SO3::rotZ(yaw), centre);
  b.half = size / 2;
  b.reflectivity = refl;
  boxes.push_back(b);
}

}  // namespace lo::synth
