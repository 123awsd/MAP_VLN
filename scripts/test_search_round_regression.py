"""Compile a regression against the actual manager scheduler and actual A* library."""
from pathlib import Path
import shlex,subprocess
root=Path('/workspace/shared');out=root/'outputs/consistency_validation_20261015'
cpp=r'''
#include "exploration_manager/exploration_manager.h"
#include "exploration_manager/exploration_data.h"
#include "fast_planner/planner_manager.h"
#include <cassert>
#include <iostream>
using namespace fast_planner;
int main(int argc,char**argv){
 ros::init(argc,argv,"search_round_regression");ros::NodeHandle nh("~");
 ExplorationManager manager;manager.initialize(nh);
 auto map=manager.planner_manager_->map_server_; auto grid=map->getOccupancyGrid();
 Eigen::Vector3d start(0,0,1),goal(6,6,1);
 for(double x=-2;x<2;x+=.05)for(double y=-2;y<2;y+=.05)for(double z=0;z<2;z+=.05)
   grid->setOccupancy(grid->positionToAddress(Eigen::Vector3d(x,y,z)),voxel_mapping::OccupancyType::FREE);
 auto search=manager.planner_manager_->path_finder_;
 search->setResolution(.1); search->setMaxSearchTime(5.);
 auto begin=ros::WallTime::now(); search->setRoundDeadline(begin+ros::WallDuration(.03));
 int timeouts=0;
 for(int i=0;i<13;++i){search->reset();assert(search->search(start,goal)!=Astar::REACH_END);if(search->lastSearchResult()==Astar::SEARCH_TIMEOUT)++timeouts;}
 double elapsed=(ros::WallTime::now()-begin).toSec();assert(timeouts==13);assert(elapsed<.15);
 search->clearRoundDeadline();search->reset();assert(search->search(start,start+Eigen::Vector3d(.2,0,0))==Astar::REACH_END);assert(search->lastSearchResult()==Astar::REACH_END);
 // Unknown isolated start: exhaust the graph, without a time limit or a node shortage.
 search->reset();assert(search->search(Eigen::Vector3d(8,8,1),Eigen::Vector3d(10,10,1))==Astar::NO_PATH);assert(search->lastSearchResult()==Astar::NO_PATH);
 ros::param::set("/astar/allocate_num",2);Astar tiny;tiny.init(nh,map);tiny.setResolution(.1);tiny.setMaxSearchTime(5.);assert(tiny.search(start,goal)==Astar::NO_PATH);assert(tiny.lastSearchResult()==Astar::RESOURCE_LIMIT);
 SearchRetrySchedule scheduler;std::vector<Eigen::Vector3d> candidates;
 for(int i=0;i<13;++i)candidates.push_back(Eigen::Vector3d(i+1,0,1));
 std::set<int> chosen;
 for(int round=0;round<13;++round){
   std::stable_sort(candidates.begin(),candidates.end(),[&](const Eigen::Vector3d&a,const Eigen::Vector3d&b){auto pa=scheduler.previous(start,a,"fixed"),pb=scheduler.previous(start,b,"fixed");return (pa?pa->attempted_at:0.)<(pb?pb->attempted_at:0.);});
   auto c=candidates.front();chosen.insert(int(c.x()));scheduler.attempts.push_back({start,c,"fixed",2,Astar::SEARCH_TIMEOUT,100.+round*.1});
 }
 assert(chosen.size()==13);
 auto c=candidates.front();double now=scheduler.previous(start,c,"fixed")->attempted_at+.1;
 assert(scheduler.deferred(start,c,"fixed",2,now));
 assert(!scheduler.deferred(start,c,"fixed",4,now)); // Map changed.
 assert(!scheduler.deferred(start,c,"fixed",3,now)); // Map updating.
 assert(!scheduler.deferred(start+Eigen::Vector3d(.1,0,0),c,"fixed",2,now));
 assert(!scheduler.deferred(start,c,"changed-clearance",2,now));
 assert(!scheduler.deferred(start,c,"fixed",2,now+1.));
 scheduler.prune(now+301.);assert(scheduler.attempts.empty());
 std::cout<<"REGRESSION_RESULT {\"passed\":true,\"shared_deadline_13_calls_s\":"<<elapsed<<",\"timeout_results\":"<<timeouts<<",\"distinct_scheduled_candidates\":"<<chosen.size()<<",\"resource_limit_and_exhausted_distinguished\":true,\"context_changes_invalidate_deferral\":true,\"finite_deferral\":true,\"scope\":\"actual Astar and production scheduler, synthetic occupancy; not historical map replay\"}"<<std::endl;
}
'''
cpp=cpp.replace('#include <cassert>','#include <cassert>\n#include <set>')
Path('/tmp/search_round_regression.cpp').write_text(cpp)
build=Path('/workspace/falcon_ws/build/FALCON/falcon_planner/exploration_manager');flags=(build/'CMakeFiles/exploration_node.dir/flags.make').read_text();includes=next(s.split('=',1)[1]for s in flags.splitlines()if s.startswith('CXX_INCLUDES ='))
subprocess.run(['/usr/bin/c++','-std=c++14','-O2','-w',*shlex.split(includes),'-c','/tmp/search_round_regression.cpp','-o','/tmp/search_round_regression.o'],check=True)
args=shlex.split((build/'CMakeFiles/exploration_node.dir/link.txt').read_text());args=[x for x in args if not x.endswith('src/exploration_fsm.cpp.o')];args=['/tmp/search_round_regression.o'if x.endswith('src/exploration_node.cpp.o')else'/tmp/search_round_regression'if x=='/workspace/falcon_ws/devel/lib/exploration_manager/exploration_node'else x for x in args];subprocess.run(args,check=True,cwd=build)
print('compiled /tmp/search_round_regression')
