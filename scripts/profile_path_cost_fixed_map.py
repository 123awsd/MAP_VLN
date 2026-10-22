"""Container-only, fixed synthetic occupancy benchmark of actual path-cost code.

Builds a renamed timing copy in /tmp; does not replace planner libraries or run Habitat.
"""
from pathlib import Path
import re, shlex, subprocess

root = Path('/workspace/shared')
source = root/'runtime/consistency_validation_20261015/A3_observation_lifecycle'
header = (source/'path_cost_evaluator.h').read_text().replace('_GRAPH_NODE_H_', '_PROFILE_PATH_COST_H_').replace('PathCostEvaluator', 'ProfilePathCostEvaluator')
Path('/tmp/profile_path_cost.h').write_text(header)
code = (source/'path_cost_evaluator.cpp').read_text().replace('"pathfinding/path_cost_evaluator.h"', '"/tmp/profile_path_cost.h"').replace('PathCostEvaluator', 'ProfilePathCostEvaluator')
timers = '''
#include <chrono>
double phase_seconds[5]={}; unsigned long long phase_calls[5]={};
struct PhaseTimer {int i; std::chrono::steady_clock::time_point t=std::chrono::steady_clock::now();
explicit PhaseTimer(int n):i(n){} ~PhaseTimer(){phase_seconds[i]+=std::chrono::duration<double>(std::chrono::steady_clock::now()-t).count();++phase_calls[i];}};
'''
code = timers + code
# Scope around the original ray traversal, without changing the loop or conditions.
code, count = re.subn(r'(  caster_->input\(p1, p2\);.*?\n  \})', r'  { PhaseTimer timer(0);\n\1\n  }', code, flags=re.S)
assert count == 5, count
code = code.replace('  bool consistent_guard=false;', '  { PhaseTimer timer(1);\n  bool consistent_guard=false;')
code = code.replace('  if (safe) {', '  }\n  if (safe) {', 1)
code = code.replace('  astar_->reset();', '  { PhaseTimer timer(2); astar_->reset(); }')
code, count = re.subn(r'if \((astar_->search\w*\([^;\n]+\)) == Astar::REACH_END\)', r'if ([&](){PhaseTimer timer(3);return \1;}() == Astar::REACH_END)', code)
assert count == 5, count
code = code.replace('    path = astar_->getPath();', '    PhaseTimer timer(4);\n    path = astar_->getPath();')
Path('/tmp/profile_path_cost.cpp').write_text(code)
main = r'''
#include "exploration_manager/exploration_manager.h"
#include "fast_planner/planner_manager.h"
#include "pathfinding/path_cost_evaluator.h"
#include "/tmp/profile_path_cost.h"
#include <chrono>
#include <iostream>
#include <stdexcept>
extern double phase_seconds[5]; extern unsigned long long phase_calls[5];
using namespace fast_planner;
int main(int argc,char**argv){
ros::init(argc,argv,"path_cost_fixed_map_profile");ros::NodeHandle nh("~");
ExplorationManager m;m.initialize(nh);
ProfilePathCostEvaluator::astar_=PathCostEvaluator::astar_;
ProfilePathCostEvaluator::caster_=PathCostEvaluator::caster_;
ProfilePathCostEvaluator::map_server_=PathCostEvaluator::map_server_;
auto grid=m.planner_manager_->map_server_->getOccupancyGrid();
auto astar=PathCostEvaluator::astar_;astar->setMaxSearchTime(.05);astar->clearRoundDeadline();
for(double x=-2;x<5;x+=.09)for(double y=-2;y<5;y+=.09)for(double z=.1;z<2.5;z+=.09)
grid->setOccupancy(grid->positionToAddress(Vector3d(x,y,z)),voxel_mapping::OccupancyType::FREE);
std::cout<<"PROFILE_RESULT [";bool first=true;
for(int obstacle=0;obstacle<2;++obstacle){
if(obstacle)for(double y=-.8;y<1.2;y+=.09)for(double z=.1;z<2.5;z+=.09)
grid->setOccupancy(grid->positionToAddress(Vector3d(1,y,z)),voxel_mapping::OccupancyType::OCCUPIED);
for(int mode=0;mode<3;++mode){
for(int i=0;i<5;++i){phase_seconds[i]=0;phase_calls[i]=0;}
int matched=0,success=0;double checksum=0;auto t=std::chrono::steady_clock::now();
for(int i=0;i<300;++i){Vector3d a(0,0,1),b(2,.001*(i%20),1),lo(-1,-1.5,.2),hi(4,3,2.3);std::vector<Vector3d> p,q;
auto call=[&](bool profile){if(mode==0)return profile?ProfilePathCostEvaluator::searchPath(a,b,p):PathCostEvaluator::searchPath(a,b,q);
if(mode==1)return profile?ProfilePathCostEvaluator::searchPathUnknown(a,b,p):PathCostEvaluator::searchPathUnknown(a,b,q);
return profile?ProfilePathCostEvaluator::searchPathUnknownBBox(a,b,lo,hi,p):PathCostEvaluator::searchPathUnknownBBox(a,b,lo,hi,q);};
double x=call(true),y=call(false);
if(std::abs(x-y)>1e-10 || p.size()!=q.size())throw std::runtime_error("path mismatch");
for(size_t k=0;k<p.size();++k)if((p[k]-q[k]).norm()>1e-10)throw std::runtime_error("point mismatch");
++matched;success+=x<999;checksum+=x;}
if(!first)std::cout<<",";first=false;
std::cout<<"{\"obstacle\":"<<obstacle<<",\"mode\":"<<mode<<",\"equal_paths\":"<<matched<<",\"success\":"<<success<<",\"checksum\":"<<checksum<<",\"paired_wall_s\":"<<std::chrono::duration<double>(std::chrono::steady_clock::now()-t).count()<<",\"phase_s\":[";
for(int j=0;j<5;++j){if(j)std::cout<<",";std::cout<<phase_seconds[j];}std::cout<<"],\"phase_calls\":[";for(int j=0;j<5;++j){if(j)std::cout<<",";std::cout<<phase_calls[j];}std::cout<<"]}";
}}
std::cout<<"]"<<std::endl;
// Release the renamed class's shared owners before ROS/static logger teardown.
ProfilePathCostEvaluator::astar_.reset();ProfilePathCostEvaluator::caster_.reset();ProfilePathCostEvaluator::map_server_.reset();
}
'''
Path('/tmp/path_cost_fixed_map_main.cpp').write_text(main)
build=Path('/workspace/falcon_ws/build/FALCON/falcon_planner/exploration_manager')
flags=(build/'CMakeFiles/exploration_node.dir/flags.make').read_text()
includes=shlex.split(next(s.split('=',1)[1] for s in flags.splitlines() if s.startswith('CXX_INCLUDES =')))
pathfinding_flags=Path('/workspace/falcon_ws/build/FALCON/falcon_planner/pathfinding/CMakeFiles/pathfinding.dir/flags.make').read_text()
compiler_flags=shlex.split(next(s.split('=',1)[1]for s in pathfinding_flags.splitlines()if s.startswith('CXX_FLAGS =')))
for name in ['profile_path_cost','path_cost_fixed_map_main']:
    subprocess.run(['/usr/bin/c++',*compiler_flags,'-w',*includes,'-c',f'/tmp/{name}.cpp','-o',f'/tmp/{name}.o'],check=True)
args=shlex.split((build/'CMakeFiles/exploration_node.dir/link.txt').read_text())
args=[x for x in args if not x.endswith('src/exploration_fsm.cpp.o')]
args=['/tmp/path_cost_fixed_map_main.o' if x.endswith('src/exploration_node.cpp.o') else '/tmp/path_cost_fixed_map_profile' if x=='/workspace/falcon_ws/devel/lib/exploration_manager/exploration_node' else x for x in args]
args.insert(1,'/tmp/profile_path_cost.o')
subprocess.run(args,check=True,cwd=build)
print('compiled /tmp/path_cost_fixed_map_profile; phases: ray, guard/parameter, reset, search, path extraction')
