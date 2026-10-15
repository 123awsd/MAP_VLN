#ifndef _EXPLORATION_MANAGER_H_
#define _EXPLORATION_MANAGER_H_

#include <memory>
#include <algorithm>
#include <map>
#include <set>
#include <tuple>
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

// Retry history is scheduling evidence only. No entry is an unreachable label.
class SearchRetrySchedule {
public:
  struct Attempt {
    Vector3d start, goal; std::string constraints; uint64_t epoch;
    int result; double attempted_at;
  };
  vector<Attempt> attempts;
  void prune(double now) {
    attempts.erase(std::remove_if(attempts.begin(),attempts.end(),
      [&](const Attempt &a){return now-a.attempted_at>300.;}),attempts.end());
    if(attempts.size()>4096)attempts.erase(attempts.begin(),attempts.end()-4096);
  }
  const Attempt* previous(const Vector3d &start,const Vector3d &goal,const std::string &constraints) const {
    for(auto it=attempts.rbegin();it!=attempts.rend();++it)
      if((it->start-start).norm()<.05 && (it->goal-goal).norm()<.05 && it->constraints==constraints)return &*it;
    return nullptr;
  }
  bool deferred(const Vector3d &start,const Vector3d &goal,const std::string &constraints,uint64_t epoch,double now) const {
    const auto p=previous(start,goal,constraints);
    return p && !(epoch&1) && p->epoch==epoch && now-p->attempted_at<1.;
  }
};

struct ObservationTaskState {
  enum Phase { SELECTED, ACCEPTED, EXECUTING, ARRIVED, SETTLED, CANCELLED };
  uint64_t id=0; int trajectory_id=0; Phase phase=SELECTED;
  Vector3d position=Vector3d::Zero(); double yaw=0.;
  ros::Time executing_at, arrived_at, last_pose;
  uint64_t arrival_epoch=0;
  bool active()const{return phase==EXECUTING || phase==ARRIVED;}
  bool sameGoal(const Vector3d &p,double y)const{
    return (position-p).norm()<.2 && std::abs(std::atan2(std::sin(yaw-y),std::cos(yaw-y)))<.2;
  }
  bool observe(const Vector3d &p,double y,ros::Time stamp,ros::Time now,uint64_t epoch){
    if(!active() || stamp<=last_pose || stamp<executing_at || stamp>now || (now-stamp).toSec()>.5)return false;
    last_pose=stamp;
    bool near=(p-position).norm()<.45 && std::abs(std::atan2(std::sin(y-yaw),std::cos(y-yaw)))<.35;
    if(!near){phase=EXECUTING;arrived_at=ros::Time();return false;}
    if(phase!=ARRIVED){phase=ARRIVED;arrived_at=stamp;arrival_epoch=epoch;return true;}
    return false;
  }
  bool ready(ros::Time now,uint64_t epoch,double integrated_stamp)const{
    return phase==ARRIVED && !last_pose.isZero() && now>=last_pose && (now-last_pose).toSec()<=.5 &&
      (last_pose-arrived_at).toSec()>=.5 && !(epoch&1) && epoch>arrival_epoch && integrated_stamp>arrived_at.toSec();
  }
  void cancel(){phase=CANCELLED;}
};

class ExplorationManager {
public:
  typedef shared_ptr<ExplorationManager> Ptr;

  ExplorationManager();
  ~ExplorationManager();

  void initialize(ros::NodeHandle &nh);
  ros::Time executedPoseStamp() const { return observed_stamp_; }
  void updateExecutedPose(const Vector3d &position, double yaw, const ros::Time &stamp);
  void setSearchRoundDeadline(const ros::WallTime &deadline) { search_round_deadline_=deadline; }
  void refreshObservation();
  void finalizeObservationCandidate(int trajectory_id);
  void acceptObservationCandidate(int trajectory_id);
  void executeObservationCandidate(int trajectory_id);
  void cancelObservationCandidate(int trajectory_id,const std::string &reason);
  void interruptObservation(const std::string &reason);


  int updateFrontierStruct(const Eigen::Vector3d &pos);

  int planExploreMotionHGrid(const Vector3d &pos, const Vector3d &vel, const Vector3d &acc,
                             const Vector3d &yaw);
  EXPL_RESULT planTrajToView(const Vector3d &pos, const Vector3d &vel, const Vector3d &acc,
                             const Vector3d &yaw, const Vector3d &next_pos, const double &next_yaw);

  EXPL_RESULT replanRetainedGoal(const Vector3d &pos, const Vector3d &vel,
      const Vector3d &acc, const Vector3d &yaw, const Vector3d &target, double target_yaw);

  bool planSafeAstarPrefix(const Vector3d &pos, const Vector3d &velocity,
                           const Vector3d &acceleration, const Vector3d &yaw_state,
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
  struct ObservationTask {
    ObservationTaskState state;
    std::vector<Eigen::Vector3i> predicted;
  };
  ObservationTask selected_observation_, executing_observation_;
  std::map<int,ObservationTask> candidate_observations_;
  uint64_t next_observation_id_=0;
  using ObservationKey=std::tuple<int,int,int>;
  std::vector<std::set<ObservationKey>> failed_observations_;
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
  ros::WallTime search_round_deadline_;
  SearchRetrySchedule search_schedule_;
  int last_path_result_ = 0;
  bool round_search_active_ = false;
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
