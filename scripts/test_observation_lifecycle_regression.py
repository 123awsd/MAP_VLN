"""Real manager regression with historical183 boundary and deterministic map epochs."""
from pathlib import Path
import json,shlex,subprocess
root=Path('/workspace/shared');out=root/'outputs/consistency_validation_20261015';wire=json.loads((root/'tests/fixtures/observation183_boundary.json').read_text());goal=wire['goal183'];actual=wire['plan223_start']['actual'];predicted=wire['plan223_start']['predicted_start'];yaw=wire['wire']['183']['end_yaw']
cpp=r'''
#include "exploration_manager/exploration_manager.h"
#include "exploration_manager/exploration_data.h"
#include "fast_planner/planner_manager.h"
#include <cassert>
#include <iostream>
using namespace fast_planner;
ros::Time sec(double x){ros::Time t;t.fromSec(x);return t;}
void clockAt(double x){ros::Time::setNow(sec(x));}
int main(int argc,char**argv){
 ros::init(argc,argv,"observation_lifecycle_regression");ros::NodeHandle nh("~");ExplorationManager m;m.initialize(nh);
 const Vector3d goal(GOAL),actual(ACTUAL),predicted(PREDICTED);const double yaw=YAW;
 auto map=m.planner_manager_->map_server_;auto grid=map->getOccupancyGrid();
 auto stage=[&](int id,const Vector3d &p,uint64_t task_id){m.ed_->next_pos_=p;m.ed_->next_yaw_=yaw;m.selected_observation_=ExplorationManager::ObservationTask();m.selected_observation_.state.id=task_id;m.selected_observation_.state.position=p;m.selected_observation_.state.yaw=yaw;auto idx=grid->positionToIndex(p);m.selected_observation_.predicted={idx};m.finalizeObservationCandidate(id);};
 clockAt(1000);m.active_floor_initialized_=true;m.active_floor_z_=3.97;
 stage(183,goal,1);m.acceptObservationCandidate(183);m.executeObservationCandidate(183);
 assert(m.executing_observation_.state.id==1 && m.pending_floor_valid_);
 clockAt(1000.41);m.updateExecutedPose(actual,yaw,sec(1000.4));
 assert((predicted-goal).norm()<.45 && (actual-goal).norm()>.45);
 m.refreshObservation();assert(m.executing_observation_.state.phase==ObservationTaskState::EXECUTING);assert(m.failed_observations_.empty());
 // Draft selection and acceptance never erase183 or change its executing intent.
 stage(184,Vector3d(8.0486,6.5116,3.10684),2);m.acceptObservationCandidate(184);
 assert(m.executing_observation_.state.id==1 && m.pending_floor_z_==goal.z());
 m.cancelObservationCandidate(184,"server_rejected");assert(m.executing_observation_.state.id==1 && m.failed_observations_.empty());
 // Same-task rolling execution retains identity and arrival baseline.
 stage(185,goal,1);m.acceptObservationCandidate(185);m.executeObservationCandidate(185);assert(m.executing_observation_.state.id==1);
 clockAt(1001);map->integration_epoch_.store(2);map->integrated_depth_stamp_.store(1000.9);m.updateExecutedPose(goal,yaw,sec(1001));assert(m.executing_observation_.state.phase==ObservationTaskState::ARRIVED);
 clockAt(1001.6);m.updateExecutedPose(goal,yaw,sec(1001.6));assert(m.executing_observation_.state.phase==ObservationTaskState::ARRIVED);assert(m.failed_observations_.empty());
 // Odd/in-progress epoch cannot settle even when the input timestamp is new.
 map->integration_epoch_.store(3);map->integrated_depth_stamp_.store(1001.5);m.refreshObservation();assert(m.executing_observation_.state.phase==ObservationTaskState::ARRIVED);
 auto idx=m.executing_observation_.predicted.front();grid->setOccupancy(grid->indexToAddress(idx),voxel_mapping::OccupancyType::FREE);
 map->integration_epoch_.store(4);m.refreshObservation();assert(m.executing_observation_.state.phase==ObservationTaskState::SETTLED);assert(m.failed_observations_.empty());
 m.refreshObservation();assert(m.executing_observation_.state.phase==ObservationTaskState::SETTLED);
 // A continuation already queued before settlement must not reopen this task.
 stage(190,goal,1);m.acceptObservationCandidate(190);m.executeObservationCandidate(190);
 assert(m.executing_observation_.state.phase==ObservationTaskState::SETTLED);m.refreshObservation();assert(m.failed_observations_.empty());
 // A real failure after actual arrival and later integration is recorded once.
 clockAt(1002);stage(186,goal,3);m.acceptObservationCandidate(186);m.executeObservationCandidate(186);m.updateExecutedPose(goal,yaw,sec(1002));grid->setOccupancy(grid->indexToAddress(idx),voxel_mapping::OccupancyType::UNKNOWN);
 clockAt(1002.6);map->integration_epoch_.store(6);map->integrated_depth_stamp_.store(1002.5);m.updateExecutedPose(goal,yaw,sec(1002.6));assert(m.failed_observations_.size()==1);m.refreshObservation();assert(m.failed_observations_.size()==1);
 // An actual switch uses the finalized target, not a pre-scoring reference.
 clockAt(1003);stage(187,goal,4);m.acceptObservationCandidate(187);m.executeObservationCandidate(187);assert(m.pending_floor_valid_ && m.pending_floor_z_==goal.z());
 m.interruptObservation("test_cancel");assert(!m.executing_observation_.state.active() && !m.pending_floor_valid_);assert(m.failed_observations_.size()==1);
 clockAt(1004);stage(188,goal,5);m.cancelObservationCandidate(188,"rejected_before_execute");assert(m.failed_observations_.size()==1 && !m.executing_observation_.state.active());
 // Stale poses and old/duplicate arrival frames cannot advance a task.
 ObservationTaskState st;st.id=100;st.position=goal;st.yaw=yaw;st.phase=ObservationTaskState::EXECUTING;st.executing_at=sec(1005);
 assert(!st.observe(goal,yaw,sec(1004),sec(1005),6));assert(st.phase==ObservationTaskState::EXECUTING);
 assert(st.observe(goal,yaw,sec(1005),sec(1005),6));assert(!st.observe(goal,yaw,sec(1005),sec(1005.2),6));assert(!st.ready(sec(1006),8,1005.9));
 std::cout<<"REGRESSION_RESULT {\"passed\":true,\"historical_predicted_near_actual_far_no_settlement\":true,\"actual_arrival_waits_for_later_completed_integration\":true,\"settles_success_and_failure_once\":true,\"draft_or_rejection_preserves_active_task\":true,\"rolling_continuation_preserves_identity\":true,\"actual_executing_final_goal_owns_floor_intent\":true,\"cancellation_not_observation_failure\":true,\"stale_duplicate_pose_rejected\":true,\"scope\":\"real manager lifecycle using historical positions, deterministic time and controlled occupancy/epochs; not historical map or score replay\"}"<<std::endl;
}
'''.replace('GOAL',','.join(map(str,goal))).replace('ACTUAL',','.join(map(str,actual))).replace('PREDICTED',','.join(map(str,predicted))).replace('YAW',str(yaw))
Path('/tmp/observation_lifecycle_regression.cpp').write_text(cpp);build=Path('/workspace/falcon_ws/build/FALCON/falcon_planner/exploration_manager');flags=(build/'CMakeFiles/exploration_node.dir/flags.make').read_text();inc=next(s.split('=',1)[1]for s in flags.splitlines()if s.startswith('CXX_INCLUDES ='));subprocess.run(['/usr/bin/c++','-std=c++14','-O2','-w','-fno-access-control',*shlex.split(inc),'-c','/tmp/observation_lifecycle_regression.cpp','-o','/tmp/observation_lifecycle_regression.o'],check=True);args=shlex.split((build/'CMakeFiles/exploration_node.dir/link.txt').read_text());args=[x for x in args if not x.endswith('src/exploration_fsm.cpp.o')];args=['/tmp/observation_lifecycle_regression.o'if x.endswith('src/exploration_node.cpp.o')else'/tmp/observation_lifecycle_regression'if x=='/workspace/falcon_ws/devel/lib/exploration_manager/exploration_node'else x for x in args];subprocess.run(args,check=True,cwd=build);print('compiled actual manager lifecycle regression')
