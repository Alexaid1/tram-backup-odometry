// Route map built from pathgraph-format JSON files.
//
// File format (as provided by the case organisers):
//   { "points": [ {x, y, z, tang, curv}, ... ],            // map frame, ~1 m step
//     "paths":  [ {ext_id, point_indices: [...], closed?}, ... ] }
// Each "paths" entry becomes one polyline. "closed": true (our extension, see
// tools/build_route.py) makes the arc coordinate periodic.
//
// Position is represented as (path index, arc length s). Outside an open polyline the
// position is extrapolated along the end tangent (dead-reckoning fallback).
#pragma once

#include <limits>
#include <string>
#include <vector>

namespace tram_odometry
{

struct RoutePose
{
  double x{0.0}, y{0.0}, z{0.0};
  double yaw{0.0};         // rad, map frame
  double curvature{0.0};   // 1/m
  bool on_map{true};       // false when extrapolated beyond an open polyline
};

struct Projection
{
  int path{-1};
  double s{0.0};
  double lateral{std::numeric_limits<double>::infinity()};  // signed cross-track, m (left +)
  double heading_diff{0.0};
};

class RouteMap
{
public:
  /// Append all polylines of one pathgraph-format file. Throws on malformed input.
  void load_file(const std::string & path);

  bool empty() const {return paths_.empty();}
  size_t size() const {return paths_.size();}
  double length(int path) const {return paths_.at(static_cast<size_t>(path)).cum.back();}
  bool closed(int path) const {return paths_.at(static_cast<size_t>(path)).closed;}
  const std::string & name(int path) const {return paths_.at(static_cast<size_t>(path)).name;}

  /// Pose at arc length s (s may be outside [0, L]: wraps if closed, extrapolates if open).
  RoutePose pose(int path, double s) const;

  /// Global nearest-segment projection. If heading is finite, candidates whose tangent
  /// differs by more than max_heading_diff are skipped (disambiguates parallel tracks).
  Projection project(double x, double y,
    double heading = std::numeric_limits<double>::quiet_NaN(),
    double max_heading_diff = 1.0) const;

  /// Continuous difference s_b - s_a on one path (shortest way for closed paths).
  double arc_diff(int path, double s_a, double s_b) const;

private:
  struct Path
  {
    std::string name;
    std::vector<double> x, y, z, cum;
    bool closed{false};
  };
  std::vector<Path> paths_;

  static size_t segment_index(const Path & p, double s);
};

}  // namespace tram_odometry
