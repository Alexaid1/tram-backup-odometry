#include "tram_odometry/core/route_map.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "tram_odometry/core/json_lite.hpp"
#include "tram_odometry/core/math_util.hpp"

namespace tram_odometry
{
void RouteMap::load_file(const std::string & file)
{
  const json::Value root = json::parse_file(file);
  const auto & pts = root.at("points").arr;
  const auto & paths = root.at("paths").arr;
  for (size_t k = 0; k < paths.size(); ++k) {
    const auto & pj = paths[k];
    Path p;
    p.name = file + "#" + std::to_string(k);
    if (pj.has("closed") && pj.at("closed").type == json::Value::Type::Bool) {
      p.closed = pj.at("closed").b;
    }
    for (const auto & iv : pj.at("point_indices").arr) {
      const auto idx = static_cast<size_t>(iv.number());
      if (idx >= pts.size()) {throw std::runtime_error("route: point index out of range");}
      const auto & q = pts[idx];
      const double x = q.at("x").number(), y = q.at("y").number();
      const double z = q.has("z") ? q.at("z").number() : 0.0;
      if (!p.x.empty() && std::hypot(x - p.x.back(), y - p.y.back()) < 1e-6) {continue;}
      p.x.push_back(x); p.y.push_back(y); p.z.push_back(z);
    }
    if (p.x.size() < 2) {continue;}
    if (p.closed) {  // explicit closing segment
      p.x.push_back(p.x.front()); p.y.push_back(p.y.front()); p.z.push_back(p.z.front());
    }
    p.cum.assign(p.x.size(), 0.0);
    for (size_t i = 1; i < p.x.size(); ++i) {
      p.cum[i] = p.cum[i - 1] + std::hypot(p.x[i] - p.x[i - 1], p.y[i] - p.y[i - 1]);
    }
    paths_.push_back(std::move(p));
  }
  if (paths_.empty()) {throw std::runtime_error("route: no polylines in " + file);}
}

size_t RouteMap::segment_index(const Path & p, double s)
{
  // index i such that cum[i] <= s < cum[i+1]
  auto it = std::upper_bound(p.cum.begin(), p.cum.end(), s);
  size_t i = static_cast<size_t>(std::max<std::ptrdiff_t>(0, (it - p.cum.begin()) - 1));
  return std::min(i, p.cum.size() - 2);
}

RoutePose RouteMap::pose(int path, double s) const
{
  const Path & p = paths_.at(static_cast<size_t>(path));
  const double L = p.cum.back();
  RoutePose r;
  if (p.closed) {
    s = std::fmod(s, L);
    if (s < 0) {s += L;}
  }
  const bool before = s < 0.0, after = s > L;
  const double sc = std::clamp(s, 0.0, L);
  const size_t i = segment_index(p, sc);
  const double seg = std::max(p.cum[i + 1] - p.cum[i], 1e-9);
  const double t = (sc - p.cum[i]) / seg;
  const double dx = p.x[i + 1] - p.x[i], dy = p.y[i + 1] - p.y[i];
  r.yaw = std::atan2(dy, dx);
  r.x = p.x[i] + t * dx;
  r.y = p.y[i] + t * dy;
  r.z = p.z[i] + t * (p.z[i + 1] - p.z[i]);
  // curvature from neighbouring segment headings
  if (i + 2 < p.x.size()) {
    const double yaw2 = std::atan2(p.y[i + 2] - p.y[i + 1], p.x[i + 2] - p.x[i + 1]);
    r.curvature = wrap_pi(yaw2 - r.yaw) / seg;
  }
  if (before || after) {  // straight-line extrapolation beyond open ends
    const double ext = before ? s : s - L;
    r.x += ext * std::cos(r.yaw);
    r.y += ext * std::sin(r.yaw);
    r.on_map = false;
    r.curvature = 0.0;
  }
  return r;
}

Projection RouteMap::project(double x, double y, double heading, double max_heading_diff) const
{
  Projection best;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (size_t k = 0; k < paths_.size(); ++k) {
    const Path & p = paths_[k];
    for (size_t i = 0; i + 1 < p.x.size(); ++i) {
      const double dx = p.x[i + 1] - p.x[i], dy = p.y[i + 1] - p.y[i];
      const double l2 = dx * dx + dy * dy;
      if (l2 < 1e-12) {continue;}
      // open polylines: allow projection beyond the first/last vertex (extrapolated s)
      const double lo = (!p.closed && i == 0) ? -1e6 : 0.0;
      const double hi = (!p.closed && i + 2 == p.x.size()) ? 1e6 : 1.0;
      const double t = std::clamp(((x - p.x[i]) * dx + (y - p.y[i]) * dy) / l2, lo, hi);
      const double px = p.x[i] + t * dx, py = p.y[i] + t * dy;
      const double d2 = (x - px) * (x - px) + (y - py) * (y - py);
      if (d2 >= best_d2) {continue;}
      const double hd = std::isfinite(heading) ? wrap_pi(heading - std::atan2(dy, dx)) : 0.0;
      if (std::fabs(hd) > max_heading_diff) {continue;}
      best_d2 = d2;
      best.path = static_cast<int>(k);
      best.s = p.cum[i] + t * std::sqrt(l2);
      const double cross = (dx * (y - p.y[i]) - dy * (x - p.x[i])) / std::sqrt(l2);
      best.lateral = cross;
      best.heading_diff = hd;
    }
  }
  return best;
}

double RouteMap::arc_diff(int path, double s_a, double s_b) const
{
  double d = s_b - s_a;
  const Path & p = paths_.at(static_cast<size_t>(path));
  if (p.closed) {
    const double L = p.cum.back();
    d = std::fmod(d, L);
    if (d > L / 2) {d -= L;}
    if (d < -L / 2) {d += L;}
  }
  return d;
}

}  // namespace tram_odometry
