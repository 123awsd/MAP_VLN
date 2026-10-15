#ifndef _EXPLORATION_FSM_H_
#define _EXPLORATION_FSM_H_

#include "fast_planner/planner_manager.h"
#include <algorithm>
#include <deque>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <glog/logging.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/Marker.h>

using Eigen::Vector3d;
using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::vector;

namespace fast_planner {
// Sender is deliberately stricter than the receiver's 20ms lower bound.
struct CandidateAdmission {
 enum Decision { INVALID, WAIT, SUBMIT };
 static Decision decide(double lead){if(!std::isfinite(lead)||lead<.04)return INVALID;return lead>3.?WAIT:SUBMIT;}
};
// Samples include the complete synchronous planner call; the old curve remains
// owned by the server throughout. A budget is a forecast, never a time extension.
class HandoverBudget {
 std::deque<double> solve_,delivery_;
 static double quantile(const std::deque<double>&q,double fallback){if(q.empty())return fallback;std::vector<double>v(q.begin(),q.end());std::sort(v.begin(),v.end());return v[static_cast<size_t>(std::ceil(.95*(v.size()-1)))];}
 static void add(std::deque<double>&q,double x){if(!std::isfinite(x)||x<0)return;q.push_back(x);if(q.size()>32)q.pop_front();}
 public:
 void observeSolve(double x){add(solve_,x);}
 void observeDelivery(double x){add(delivery_,x);}
 double lead()const{return std::max(.3,quantile(solve_,.5)+std::max(.08,quantile(delivery_,.08))+.08);}
 double triggerRemaining()const{return lead()+.10;}
 bool shouldPlan(double remaining)const{return remaining<=triggerRemaining();}
 bool ready(double now,double scheduled)const{return scheduled-now>=.04;}
};
}

namespace fast_planner {
class FastPlannerManager;
class ExplorationManager;
class PlanningVisualization;
struct FSMParam;
struct FSMData;

enum EXPL_STATE { INIT, WAIT_TRIGGER, PLAN_TRAJ, PUB_TRAJ, EXEC_TRAJ, FINISH, RTB};

class ExplorationFSM {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
  ExplorationFSM(/* args */) {}
  ~ExplorationFSM() {}

  void init(ros::NodeHandle &nh);

private:
  /* planning utils */
  shared_ptr<FastPlannerManager> planner_manager_;
  shared_ptr<ExplorationManager> expl_manager_;
  shared_ptr<PlanningVisualization> visualization_;

  shared_ptr<FSMParam> fp_;
  shared_ptr<FSMData> fd_;
  EXPL_STATE state_;

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, safety_timer_, vis_timer_, frontier_timer_;
  ros::Subscriber trigger_sub_, odom_sub_;
  ros::Publisher replan_pub_, bspline_pub_, grid_tour_pub_, uncertainty_pub_;

  bool frontier_ready_;

  ros::Time trajectory_start_time_;

  struct ExecutedGoalRecord {
    Vector3d viewpoint;
    Vector3d actual_end;
    double coverage;
    double trajectory_length;
    ros::Time completed_at;
  };
  std::deque<ExecutedGoalRecord> executed_goal_history_;
  Vector3d executing_viewpoint_;
  double executing_trajectory_length_;
  bool executing_goal_valid_;
  bool timing_goal_pending_ = false;
  Vector3d timing_goal_position_;
  double timing_goal_yaw_ = 0.0;

  LocalTrajData active_data_, pending_data_;
  int planning_base_id_=0,pending_base_id_=0;
  bool active_valid_ = false, pending_sent_ = false;
  bool frozen_waiting_=false, admission_delayed_=false;
  ros::WallTime frozen_wait_started_;
  double intentional_wait_s_=0.;
  Vector3d pending_goal_;
  void submitFrozenCandidate();
  int pending_ack_ = 0, issued_id_ = 0;
  ros::Time last_capture_, stopped_after_;
  bool waiting_actual_ = false;
  double planning_horizon_ = 0.7;
  HandoverBudget handover_budget_;
  ros::WallTime candidate_completed_,candidate_sent_;
  ros::Time candidate_sent_ros_;
  ros::Subscriber acceptance_sub_, switched_sub_;
  int switched_id_ = 0;
  void executionStop(const std::string &reason);
  void resumeActive(const std::string &reason);
  /* helper functions */
  int callExplorationPlanner();
  bool recordExecutedGoalAndDetectOscillation();
  void transitState(EXPL_STATE new_state, string pos_call);

  /* ROS functions */
  void FSMCallback(const ros::TimerEvent &e);
  void safetyCallback(const ros::TimerEvent &e);
  void frontierCallback(const ros::TimerEvent &e);
  void triggerCallback(const geometry_msgs::PoseStampedPtr &msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
  void visualize();
  void clearVisMarker();
};

} // namespace fast_planner

#endif