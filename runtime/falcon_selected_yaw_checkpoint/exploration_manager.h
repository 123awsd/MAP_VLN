#ifndef _EXPLORATION_MANAGER_H_
#define _EXPLORATION_MANAGER_H_

#include <memory>
#include <vector>

#include <Eigen/Eigen>
#include <glog/logging.h>
#include <ros/ros.h>

#include "exploration_preprocessing/frontier_finder.h"
#include "exploration_preprocessing/hierarchical_grid.h"
#include "traj_utils/planning_visualization.h"
#include "voxel_mapping/map_server.h"

using Eigen::Vector3d;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace fast_planner {
class EDTEnvironment;
class FastPlannerManager;
class FrontierFinder;
class HGrid;

struct ExplorationParam;
struct ExplorationData;
struct ExplorationExpData;

enum EXPL_RESULT { FAIL, SUCCEED, NO_GRID, SEARCH_PENDING };

class ExplorationManager {
public:
  typedef shared_ptr<ExplorationManager> Ptr;

  ExplorationManager();
  ~ExplorationManager();

  void initialize(ros::NodeHandle &nh);
  unsigned long long diagnostic_plan_id_=0;
  unsigned long long diagnosticPlanId() const {return diagnostic_plan_id_;}
  ros::Time executedPoseStamp() const { return observed_stamp_; }
  void updateExecutedPose(const Vector3d &position, double yaw, const ros::Time &stamp) {
    observed_position_ = position; observed_yaw_ = yaw; observed_stamp_ = stamp;
    if (pending_observation_.active) {
      const bool arrived=(position-pending_observation_.position).norm()<.45 &&
        fabs(atan2(sin(yaw-pending_observation_.yaw),cos(yaw-pending_observation_.yaw)))<.35;
      if(!arrived) pending_observation_.arrived=ros::Time();
      else if(pending_observation_.arrived.isZero()) pending_observation_.arrived=stamp;
      pending_observation_.last_pose=stamp;
    }
  }

  int updateFrontierStruct(const Eigen::Vector3d &pos);

  int planExploreMotionHGrid(const Vector3d &pos, const Vector3d &vel, const Vector3d &acc,
                             const Vector3d &yaw);
  EXPL_RESULT planTrajToView(const Vector3d &pos, const Vector3d &vel, const Vector3d &acc,
                             const Vector3d &yaw, const Vector3d &next_pos, const double &next_yaw);

  EXPL_RESULT replanRetainedGoal(const Vector3d &pos, const Vector3d &vel,
      const Vector3d &acc, const Vector3d &yaw, const Vector3d &target, double target_yaw);

  bool planSafeAstarPrefix(const Vector3d &pos, const double yaw,
                           const double max_prefix_length);

  void initializeHierarchicalGrid(const Vector3d &pos, const Vector3d &vel);

  void blockViewpoint(const Vector3d &pos, double seconds, const string &reason);

  shared_ptr<ExplorationData> ed_;
  shared_ptr<ExplorationParam> ep_;
  shared_ptr<ExplorationExpData> ee_;

  shared_ptr<FastPlannerManager> planner_manager_;
  shared_ptr<FrontierFinder> frontier_finder_;
  shared_ptr<HierarchicalGrid> hierarchical_grid_;
  shared_ptr<PlanningVisualization> visualization_;

private:
  struct ObservationAttempt {
    Vector3d position; double yaw;
    std::vector<Eigen::Vector3i> predicted;
    ros::Time started, arrived, last_pose;
    bool active=false;
  };
  ObservationAttempt pending_observation_;
  Vector3d observed_position_ = Vector3d::Zero();
  double observed_yaw_ = 0.0;
  ros::Time observed_stamp_;
  struct TSPConfig {
    int dimension_;
    string problem_name_;

    bool skip_first_ = false;
    bool skip_last_ = false;
    int result_id_offset_ = 1;
  };

  shared_ptr<EDTEnvironment> edt_environment_;
  voxel_mapping::MapServer::Ptr map_server_;
  bool voxel_astar_fallback_enabled_;
  double voxel_astar_fallback_max_search_time_;

  vector<int> dijkstra(vector<vector<double>> &graph, int start, int end);

  // Refine local tour for next few frontiers, using more diverse viewpoints
  void refineLocalTourHGrid(const Vector3d &cur_pos, const Vector3d &cur_vel,
                            const Vector3d &cur_yaw, const Vector3d &next_pos,
                            const vector<vector<Vector3d>> &n_points,
                            const vector<vector<double>> &n_yaws, vector<Vector3d> &refined_pts,
                            vector<double> &refined_yaws);

  void solveTSP(const Eigen::MatrixXd &cost_matrix, const TSPConfig &config,
                vector<int> &result_indices, double &total_cost);

  void clearExplorationData();

  bool isTemporarilyBlocked(const Vector3d &pos) const;
  void notePlanningFailure(const Vector3d &pos);
  void updateOnlineFloorModel(const Vector3d &pos);
  int classifyFloorCandidate(const Vector3d &pos) const;
  void applyFloorPriority(const Vector3d &pos, const Vector3d &vel, const Vector3d &yaw,
                          Vector3d &next_pos, double &next_yaw);

  template <typename T>
  void saveCostMatrix(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &mat,
                      const string &filename);

  double latest_frontier_time_;
  struct FrontierBlockState {
    Vector3d position;
    int failures;
    ros::Time blocked_until;
  };
  vector<FrontierBlockState> blocked_frontiers_;
  int dormant_finish_block_threshold_;

  bool floor_priority_enabled_;
  double floor_histogram_resolution_;
  double floor_min_separation_;
  double floor_same_band_tolerance_;
  int floor_min_viewpoint_support_;
  double floor_crossing_penalty_;
  double floor_switch_lock_seconds_;
  int floor_candidate_limit_;
  bool active_floor_initialized_;
  double active_floor_z_;
  bool pending_floor_valid_;
  double pending_floor_z_;
  ros::Time floor_lock_until_;
  vector<double> online_floor_levels_;
  vector<Vector3d> executed_safe_pose_history_;
};

} // namespace fast_planner

#endif
