#pragma once
#include <Eigen/Core>
#include <ros/ros.h>
#include <cmath>
#include <vector>

namespace fast_planner {
inline bool previewPolygonContains(const std::vector<double>& polygon, const Eigen::Vector3d& p) {
  bool inside = false;
  const size_t n = polygon.size() / 2;
  if (n < 3) return false;
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    const double ax = polygon[2*j], ay = polygon[2*j+1];
    const double bx = polygon[2*i], by = polygon[2*i+1];
    const double cross = (p.x()-ax)*(by-ay) - (p.y()-ay)*(bx-ax);
    if (std::abs(cross) < 1e-8 && p.x() >= std::min(ax,bx)-1e-8 &&
        p.x() <= std::max(ax,bx)+1e-8 && p.y() >= std::min(ay,by)-1e-8 &&
        p.y() <= std::max(ay,by)+1e-8) return true;
    if ((ay > p.y()) != (by > p.y()) &&
        p.x() < (bx-ax)*(p.y()-ay)/(by-ay)+ax) inside = !inside;
  }
  return inside;
}
inline bool previewGeofenceAllowed(const Eigen::Vector3d& p) {
  struct Fence {
    std::vector<double> allowed;
    std::vector<std::vector<double>> excluded;
    Fence() {
      ros::param::get("/astar/preview_allowed_xy", allowed);
      int count = 0;
      ros::param::param("/astar/preview_excluded_count", count, 0);
      for (int i=0; i<count; ++i) {
        std::vector<double> polygon;
        ros::param::get("/astar/preview_excluded_xy_"+std::to_string(i), polygon);
        excluded.push_back(polygon);
      }
    }
  };
  static const Fence fence;
  if (fence.allowed.empty()) return true; // No changes to normal runs.
  if (!p.allFinite() || !previewPolygonContains(fence.allowed,p)) return false;
  for (const auto& polygon : fence.excluded)
    if (previewPolygonContains(polygon,p)) return false;
  return true;
}
// Restrict planning targets, never sensor returns or the physical voxel map.
inline bool previewExplorationTargetAllowed(const Eigen::Vector3d& p) {
  struct TargetBand {
    bool enabled;
    double lo = 0.8, hi = 1.5;
    TargetBand() {
      std::vector<double> polygon;
      ros::param::get("/astar/preview_allowed_xy", polygon);
      enabled = !polygon.empty();
      ros::param::param("/frontier_finder/target_min_z", lo, 0.8);
      ros::param::param("/frontier_finder/target_max_z", hi, 1.5);
    }
  };
  static const TargetBand band;
  return !band.enabled || (p.z() >= band.lo && p.z() <= band.hi && previewGeofenceAllowed(p));
}
}
