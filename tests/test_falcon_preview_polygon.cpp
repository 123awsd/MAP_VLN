#include <cassert>
#include "pathfinding/preview_geofence.h"

int main() {
  const std::vector<double> box{0,0, 4,0, 4,4, 0,4};
  assert(fast_planner::previewPolygonContains(box, Eigen::Vector3d(2,2,1)));
  assert(fast_planner::previewPolygonContains(box, Eigen::Vector3d(0,2,1)));
  assert(!fast_planner::previewPolygonContains(box, Eigen::Vector3d(5,2,1)));
  const std::vector<double> concave{0,0, 4,0, 4,1, 1,1, 1,4, 0,4};
  assert(fast_planner::previewPolygonContains(concave, Eigen::Vector3d(.5,3,1)));
  assert(!fast_planner::previewPolygonContains(concave, Eigen::Vector3d(3,3,1)));
  assert(!fast_planner::previewPolygonContains({}, Eigen::Vector3d(0,0,1)));
  return 0;
}
