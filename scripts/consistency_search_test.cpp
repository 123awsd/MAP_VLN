#include <fstream>
#include <map>
#include "pathfinding/astar.h"
#include "pathfinding/path_cost_evaluator.h"
#include "exploration_manager/exploration_manager.h"

namespace fast_planner {
struct ConsistencySearchProbe {
 static bool blocked(ExplorationManager&m,const Eigen::Vector3d&p){return m.isTemporarilyBlocked(p);}
 static void note(ExplorationManager&m,const Eigen::Vector3d&p){m.notePlanningFailure(p);}
};
}
using namespace fast_planner;
using namespace voxel_mapping;
int main(int argc,char**argv){
 ros::init(argc,argv,"search_contract_integration");ros::NodeHandle nh("~");
 auto map=std::make_shared<MapServer>(nh);auto grid=map->getOccupancyGrid();
 auto set=[&](double x,double y,double z,OccupancyType t){grid->setOccupancy(grid->positionToAddress(Eigen::Vector3d(x,y,z)),t);};
 auto clear=[&](){for(double x=-1.95;x<2;x+=.1)for(double y=-1.95;y<2;y+=.1)for(double z=.05;z<2;z+=.1)set(x,y,z,OccupancyType::UNKNOWN);};
 auto carve=[&](){clear();for(double x=.05;x<1.1;x+=.1)for(double y=-.05;y<.2;y+=.1)for(double z=.95;z<1.2;z+=.1)set(x,y,z,OccupancyType::FREE);for(double x=.95;x<1.2;x+=.1)for(double y=.05;y<1.2;y+=.1)for(double z=.95;z<1.2;z+=.1)set(x,y,z,OccupancyType::FREE);};
 Eigen::Vector3d start(.05,.05,1.05),goal(1.05,1.05,1.05);
 ros::param::set("/fast_planner/observed_trajectory_guard",false);
 ros::param::set("/astar/allocate_num",10000);
 auto search=std::make_shared<Astar>();search->init(nh,map);search->setResolution(.1);search->setMaxSearchTime(5);
 std::map<std::string,bool> checks;
 carve();Astar::beginDecision();search->reset();checks["real_L_route_success"]=search->search(start,goal)==Astar::REACH_END && !search->getPath().empty();
 Astar::beginDecision();search->reset();search->setMaxSearchTime(0.0);
 int timeout=search->search(start,goal);checks["real_budget_timeout"]=timeout==Astar::SEARCH_TIMEOUT;
 ExplorationManager tm;tm.blockViewpoint(goal,20,"no validated local route under current map and search constraints");ConsistencySearchProbe::note(tm,goal);ConsistencySearchProbe::note(tm,goal);
 checks["timeout_no_20s_or_45s_cooldown"]=!ConsistencySearchProbe::blocked(tm,goal);
 Astar::beginDecision();ros::param::set("/astar/allocate_num",2);Astar small;small.init(nh,map);small.setResolution(.1);small.setMaxSearchTime(5);small.reset();
 int limited=small.search(start,goal);checks["real_pool_resource_limit"]=limited==Astar::RESOURCE_LIMIT;
 ExplorationManager rm;rm.blockViewpoint(goal,20,"no validated local route under current map and search constraints");checks["resource_no_cooldown"]=!ConsistencySearchProbe::blocked(rm,goal);
 clear();for(double x=-.05;x<.3;x+=.1)for(double y=-.05;y<.3;y+=.1)for(double z=.95;z<1.2;z+=.1)set(x,y,z,OccupancyType::FREE);set(goal.x(),goal.y(),goal.z(),OccupancyType::FREE);
 Astar::beginDecision();search->reset();search->setMaxSearchTime(5);int exhausted=search->search(start,goal);checks["real_open_set_exhausted"]=exhausted==Astar::NO_PATH;
 ExplorationManager em;em.blockViewpoint(goal,20,"no validated local route under current map and search constraints");checks["exhausted_can_record_constrained_no_route"]=ConsistencySearchProbe::blocked(em,goal);
 Astar::beginDecision();search->reset();checks["invalid_start_reported"]=search->search(Eigen::Vector3d(100,0,1),goal)==Astar::INVALID_START;
 ExplorationManager im;im.blockViewpoint(goal,20,"no validated local route under current map and search constraints");checks["invalid_start_does_not_blame_goal"]=!ConsistencySearchProbe::blocked(im,goal);
 search->reset();checks["invalid_goal_reported"]=search->search(start,Eigen::Vector3d(100,0,1))==Astar::INVALID_GOAL;
 carve();Astar::beginDecision();search->reset();search->setMaxSearchTime(0.0);
 PathCostEvaluator::astar_=search;PathCostEvaluator::map_server_=map;PathCostEvaluator::vm_=2;PathCostEvaluator::am_=2;PathCostEvaluator::yd_=1;PathCostEvaluator::ydd_=1;
 PathCostEvaluator::caster_=std::make_shared<RayCaster>();PathCostEvaluator::caster_->setParams(map->getResolution(),map->getOrigin());
 std::vector<Eigen::Vector3d> route;double cost=PathCostEvaluator::computeCost(start,goal,0,0,Eigen::Vector3d::Zero(),0,route);
 checks["finite_legacy_cost_preserves_timeout_metadata"]=std::isfinite(cost)&&Astar::unknownGoal(goal)&&search->lastSearchResult()==Astar::SEARCH_TIMEOUT;
 bool passed=true;std::ofstream out(argc>1?argv[1]:"/tmp/search_contract.json");out<<"{\"checks\":{";bool first=true;
 for(const auto&c:checks){if(!first)out<<",";first=false;out<<"\""<<c.first<<"\":"<<(c.second?"true":"false");passed=passed&&c.second;}
 out<<"},\"passed\":"<<(passed?"true":"false")<<",\"observed_statuses\":["<<timeout<<","<<limited<<","<<exhausted<<"]}\n";
 return passed?0:1;
}
