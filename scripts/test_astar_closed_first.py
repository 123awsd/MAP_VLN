"""Actual baseline and transformed A* against the same synthetic map, in one process.

No runtime planner replacement; compares full paths and termination, plus paired time.
"""
from pathlib import Path
import json, re, shlex, subprocess
from prepare_astar_closed_first import transform

root = Path('/workspace/shared')
src = root/'runtime/consistency_validation_20261015/A3_observation_lifecycle'
assert (Path('/workspace/falcon_ws/src/FALCON/falcon_planner/pathfinding/src/astar.cpp').read_bytes()
        == (src/'astar.cpp').read_bytes()), 'Restore baseline runtime before paired comparison'
header = (src/'astar.h').read_text().replace('_ASTAR_H', '_COMPARISON_ASTAR_H').replace('matrix_hash', 'comparison_matrix_hash')
header = re.sub(r'\bAstar\b', 'ComparisonAstar', header)
Path('/tmp/comparison_astar.h').write_text(header)
code = transform((src/'astar.cpp').read_text()).replace('"pathfinding/astar.h"', '"/tmp/comparison_astar.h"')
code = re.sub(r'\bAstar\b', 'ComparisonAstar', code)
Path('/tmp/comparison_astar.cpp').write_text(code)
main = r'''
#include "exploration_manager/exploration_manager.h"
#include "fast_planner/planner_manager.h"
#include "/tmp/comparison_astar.h"
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <iomanip>
#include <sstream>
#include <cstdlib>
using namespace fast_planner;
void check(bool ok){if(!ok)throw std::runtime_error("search equivalence failed");}
int main(int argc,char**argv){
ros::init(argc,argv,"astar_closed_first_regression");ros::NodeHandle nh("~");
ExplorationManager manager;manager.initialize(nh);auto map=manager.planner_manager_->map_server_;
auto grid=map->getOccupancyGrid();Astar old;old.init(nh,map);ComparisonAstar candidate;candidate.init(nh,map);
const bool probe=std::getenv("ASTAR_PRODUCTION_BUDGET_PROBE")!=nullptr;old.setProfile(Astar::PROFILE::COARSE);const double production_budget=old.getMaxSearchTime();
std::ostringstream result;result<<std::setprecision(12)<<(probe?"ASTAR_BUDGET_PROBE [":"ASTAR_RESULT [");bool first=true;
for(int scene=0;scene<4;++scene){
for(double x=-2;x<4;x+=.09)for(double y=-2;y<4;y+=.09)for(double z=.15;z<2.5;z+=.09)
grid->setOccupancy(grid->positionToAddress(Eigen::Vector3d(x,y,z)),voxel_mapping::OccupancyType::FREE);
if(scene)for(double x=.85;x<1.15;x+=.09)for(double y=-1.2;y<.7;y+=.09)for(double z=.15;z<2.5;z+=.09)
grid->setOccupancy(grid->positionToAddress(Eigen::Vector3d(x,y,z)),voxel_mapping::OccupancyType::OCCUPIED);
if(scene>=2)for(double x=1.4;x<1.65;x+=.09)for(double y=.4;y<1.7;y+=.09)for(double z=.15;z<2.5;z+=.09)
grid->setOccupancy(grid->positionToAddress(Eigen::Vector3d(x,y,z)),voxel_mapping::OccupancyType::OCCUPIED);
if(scene==3)for(double x=-.3;x<2.4;x+=.09)for(double y=-.3;y<1.;y+=.09)
grid->setOccupancy(grid->positionToAddress(Eigen::Vector3d(x,y,1.15)),voxel_mapping::OccupancyType::OCCUPIED);
for(double resolution:{.1,.2,.4,.5,1.0})for(int mode=0;mode<2;++mode){
old.setResolution(resolution);candidate.setResolution(resolution);old.setMaxSearchTime(probe?production_budget:5);candidate.setMaxSearchTime(probe?production_budget:5);
double old_time=0,new_time=0;int equal=0,success=0,new_success=0,changed_termination=0;
for(int repeat=0;repeat<3;++repeat)for(int i=0;i<24;++i){
Eigen::Vector3d a(0,0,1),b(2,.015*(i%12),.8+.1*(i/12)),lo(-1.5,-1.5,.2),hi(3,2.5,2.3);
int r1=0,r2=0;
auto call_old=[&](){old.reset();auto t=std::chrono::steady_clock::now();r1=mode?old.searchUnknownBBox(a,b,lo,hi):old.searchUnknown(a,b);old_time+=std::chrono::duration<double>(std::chrono::steady_clock::now()-t).count();};
auto call_new=[&](){candidate.reset();auto t=std::chrono::steady_clock::now();r2=mode?candidate.searchUnknownBBox(a,b,lo,hi):candidate.searchUnknown(a,b);new_time+=std::chrono::duration<double>(std::chrono::steady_clock::now()-t).count();};
if(i%2){call_old();call_new();}else{call_new();call_old();}
auto p=old.getPath(),q=candidate.getPath();bool same=r1==r2&&old.lastSearchResult()==candidate.lastSearchResult()&&p.size()==q.size();
if(same)for(size_t k=0;k<p.size();++k)if((p[k]-q[k]).norm()!=0)same=false;
if(!probe){check(same);check(old.lastSearchResult()!=Astar::SEARCH_TIMEOUT);}
equal+=same;success+=r1==Astar::REACH_END;new_success+=r2==ComparisonAstar::REACH_END;changed_termination+=old.lastSearchResult()!=candidate.lastSearchResult();
}
if(!first)result<<",";first=false;
result<<"{\"scene\":"<<scene<<",\"resolution\":"<<resolution<<",\"bbox\":"<<mode<<",\"equal_paths\":"<<equal<<",\"successes\":"<<success<<",\"candidate_successes\":"<<new_success<<",\"changed_termination\":"<<changed_termination<<",\"budget_s\":"<<(probe?production_budget:5)<<",\"old_s\":"<<old_time<<",\"candidate_s\":"<<new_time<<"}";
}}
// Expired round budget and finite node pool keep their original result semantics.
old.setRoundDeadline(ros::WallTime::now()-ros::WallDuration(1));candidate.setRoundDeadline(ros::WallTime::now()-ros::WallDuration(1));
Eigen::Vector3d a(0,0,1),b(2,0,1);old.reset();candidate.reset();check(old.searchUnknown(a,b)==Astar::NO_PATH);check(candidate.searchUnknown(a,b)==ComparisonAstar::NO_PATH);check(old.lastSearchResult()==Astar::SEARCH_TIMEOUT&&candidate.lastSearchResult()==ComparisonAstar::SEARCH_TIMEOUT);
ros::param::set("/astar/allocate_num",2);Astar tiny_old;tiny_old.init(nh,map);ComparisonAstar tiny_new;tiny_new.init(nh,map);
a=Eigen::Vector3d(-1,-1,1);b=Eigen::Vector3d(3,3,1);
check(tiny_old.searchUnknown(a,b)==Astar::NO_PATH);check(tiny_new.searchUnknown(a,b)==ComparisonAstar::NO_PATH);
check(tiny_old.lastSearchResult()==Astar::RESOURCE_LIMIT&&tiny_new.lastSearchResult()==ComparisonAstar::RESOURCE_LIMIT);
old.clearRoundDeadline();candidate.clearRoundDeadline();old.setResolution(.1);candidate.setResolution(.1);
for(double x=-.4;x<.4;x+=.09)for(double y=-.4;y<.4;y+=.09)for(double z=.5;z<1.5;z+=.09)
grid->setOccupancy(grid->positionToAddress(Eigen::Vector3d(x,y,z)),voxel_mapping::OccupancyType::OCCUPIED);
a=Eigen::Vector3d(0,0,1);b=Eigen::Vector3d(2,2,1);old.reset();candidate.reset();
check(old.searchUnknown(a,b)==Astar::NO_PATH);check(candidate.searchUnknown(a,b)==ComparisonAstar::NO_PATH);
check(old.lastSearchResult()==Astar::NO_PATH&&candidate.lastSearchResult()==ComparisonAstar::NO_PATH);
result<<"]";std::cout<<result.str()<<std::endl;
}
'''
Path('/tmp/astar_closed_first_main.cpp').write_text(main)
build = Path('/workspace/falcon_ws/build/FALCON/falcon_planner/exploration_manager')
flags = (build/'CMakeFiles/exploration_node.dir/flags.make').read_text()
includes = shlex.split(next(s.split('=',1)[1] for s in flags.splitlines() if s.startswith('CXX_INCLUDES =')))
pathfinding_flags = Path('/workspace/falcon_ws/build/FALCON/falcon_planner/pathfinding/CMakeFiles/pathfinding.dir/flags.make').read_text()
compiler_flags = shlex.split(next(s.split('=',1)[1] for s in pathfinding_flags.splitlines() if s.startswith('CXX_FLAGS =')))
for name in ['comparison_astar', 'astar_closed_first_main']:
    subprocess.run(['/usr/bin/c++',*compiler_flags,'-w',*includes,'-c',f'/tmp/{name}.cpp','-o',f'/tmp/{name}.o'],check=True)
args = shlex.split((build/'CMakeFiles/exploration_node.dir/link.txt').read_text())
args = [x for x in args if not x.endswith('src/exploration_fsm.cpp.o')]
args = ['/tmp/astar_closed_first_main.o' if x.endswith('src/exploration_node.cpp.o') else '/tmp/astar_closed_first_regression' if x=='/workspace/falcon_ws/devel/lib/exploration_manager/exploration_node' else x for x in args]
args.insert(1,'/tmp/comparison_astar.o')
subprocess.run(args,check=True,cwd=build)
print('compiled actual A* equivalence and paired benchmark')
