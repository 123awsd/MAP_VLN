"""Actual helper regression with the historical moving boundary and a synthetic FREE crop."""
import json,subprocess,shlex
from pathlib import Path
root=Path('/workspace/shared');out=root/'outputs/consistency_validation_20261015';d=json.loads((out/'first_rejection_audit.json').read_text())[0];v=d['base_from_wire_at_handover']['velocity'];p=d['base_from_wire_at_handover']['position'];cpp='''#include "exploration_manager/exploration_manager.h"
#include "exploration_manager/exploration_data.h"
#include "fast_planner/planner_manager.h"
#include <cassert>
#include <iostream>
using namespace fast_planner;
int main(int argc,char**argv){ros::init(argc,argv,"motion_prefix_regression");ros::NodeHandle nh("~");ExplorationManager manager;manager.initialize(nh);
Eigen::Vector3d pos('''+','.join(map(str,p))+'''),velocity('''+','.join(map(str,v))+''');Eigen::Vector3d acceleration=Eigen::Vector3d::Zero(),yaw_state(0,0,0);
auto map=manager.planner_manager_->map_server_;auto grid=map->getOccupancyGrid();
for(double x=pos.x()-1;x<=pos.x()+1;x+=.05)for(double y=pos.y()-1;y<=pos.y()+1;y+=.05)for(double z=pos.z()-.5;z<=pos.z()+.5;z+=.05)grid->setOccupancy(grid->positionToAddress(Eigen::Vector3d(x,y,z)),voxel_mapping::OccupancyType::FREE);
Eigen::Vector3d direction=velocity.normalized();manager.ed_->raw_path_next_goal_={pos,pos+.25*direction,pos+.5*direction};manager.ed_->next_pos_=pos+direction;
bool produced=manager.planSafeAstarPrefix(pos,velocity,acceleration,yaw_state,.6);assert(produced);
auto &traj=manager.planner_manager_->local_data_;double err=(traj.position_traj_.getDerivative().evaluateDeBoorT(0.)-velocity).norm();assert(err<=.5);
// The historical zero-speed replacement is still not a valid moving handover.
assert(velocity.norm()>.5);
Eigen::Vector3d invalid=velocity;invalid[0]=NAN;assert(!manager.planSafeAstarPrefix(pos,invalid,acceleration,yaw_state,.6));
std::cout<<"REGRESSION_RESULT {\\"passed\\":true,\\"historical_moving_boundary_error\\":"<<err<<",\\"historical_zero_speed_error\\":"<<velocity.norm()<<",\\"invalid_state_rejected\\":true,\\"scope\\":\\"real helper on synthetic FREE crop, not historical map replay\\"}"<<std::endl;return 0;}
''';Path('/tmp/motion_prefix_regression.cpp').write_text(cpp);build=Path('/workspace/falcon_ws/build/FALCON/falcon_planner/exploration_manager');flags=(build/'CMakeFiles/exploration_node.dir/flags.make').read_text();includes=next(s.split('=',1)[1]for s in flags.splitlines()if s.startswith('CXX_INCLUDES ='));subprocess.run(['/usr/bin/c++','-std=c++14','-O2','-w',*shlex.split(includes),'-c','/tmp/motion_prefix_regression.cpp','-o','/tmp/motion_prefix_regression.o'],check=True);args=shlex.split((build/'CMakeFiles/exploration_node.dir/link.txt').read_text());args=[x for x in args if not x.endswith('src/exploration_fsm.cpp.o')];args=['/tmp/motion_prefix_regression.o'if x.endswith('src/exploration_node.cpp.o')else'/tmp/motion_prefix_regression'if x=='/workspace/falcon_ws/devel/lib/exploration_manager/exploration_node'else x for x in args];subprocess.run(args,check=True,cwd=build);print('compiled /tmp/motion_prefix_regression')
