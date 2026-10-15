#pragma once
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <atomic>
#include <cstdlib>
namespace fast_planner {
inline unsigned long long &replayPlan(){static unsigned long long n=0;return n;}
class ReplayWriter {
 std::mutex m;std::condition_variable cv;std::deque<std::string> q;bool done=false;std::thread t;std::ofstream out;size_t bytes=0;unsigned long long dropped=0;
 void run(){for(;;){std::string s;{std::unique_lock<std::mutex> l(m);cv.wait(l,[&]{return done||!q.empty();});if(q.empty()&&done)break;s=std::move(q.front());q.pop_front();bytes-=s.size();}out<<s;out.flush();}}
 public: ReplayWriter(){const char*p=getenv("EXPLORATION_DIAGNOSTIC_TRACE");if(p)out.open(p,std::ios::app);t=std::thread([this]{run();});}
 ~ReplayWriter(){{std::lock_guard<std::mutex>l(m);done=true;}cv.notify_one();t.join();}
 void put(std::string s){std::lock_guard<std::mutex>l(m);if(!out)return;if(bytes+s.size()>64*1024*1024){++dropped;ROS_WARN_THROTTLE(5,"Replay queue full; records dropped");return;}if(dropped){q.push_back("{\"event\":\"records_dropped\",\"count\":"+std::to_string(dropped)+"}\n");bytes+=q.back().size();dropped=0;}bytes+=s.size();q.push_back(std::move(s));cv.notify_one();}
};
inline void replayEvent(const std::string &type,const std::string &fields="") {static ReplayWriter w;std::ostringstream s;s<<std::setprecision(16)<<"{\"event\":\""<<type<<"\",\"ros_s\":"<<ros::Time::now().toSec()<<",\"wall_s\":"<<ros::WallTime::now().toSec()<<",\"plan\":"<<replayPlan()<<fields<<"}\n";w.put(s.str());}
inline std::string rp(const Eigen::Vector3d&p){std::ostringstream s;s<<std::setprecision(12)<<"["<<p.x()<<","<<p.y()<<","<<p.z()<<"]";return s.str();}
template<class P> void replayPath(const char *kind,const P&path){std::string s=",\"points\":[";bool first=true;for(const auto&p:path){if(!first)s+=",";first=false;s+=rp(p);}replayEvent(kind,s+"]");}
template<class M> void replayLocalMap(M map,const Eigen::Vector3d&centre){const auto began=ros::WallTime::now();std::ostringstream s;s<<",\"centre\":"<<rp(centre)<<",\"step_m\":0.1,\"radius_m\":1,\"cells\":[";bool first=true;for(int x=-10;x<=10;++x)for(int y=-10;y<=10;++y)for(int z=-10;z<=10;++z){Eigen::Vector3d p=centre+Eigen::Vector3d(x,y,z)*.1;if(!map->isInBox(p))continue;if(!first)s<<",";first=false;s<<"["<<x<<","<<y<<","<<z<<","<<int(map->getOccupancy(p))<<","<<map->getESDF()->getDistance(p)<<"]";}s<<"],\"capture_s\":"<<(ros::WallTime::now()-began).toSec();replayEvent("rejection_local_map",s.str());}
}

namespace fast_planner {
inline std::string viewKey(const Eigen::Vector3d&p,double yaw){std::ostringstream s;s<<std::fixed<<std::setprecision(4)<<p.x()<<":"<<p.y()<<":"<<p.z()<<":"<<yaw;return s.str();}
inline void viewEventImpl(const char*stage,const Eigen::Vector3d&p,double yaw,const std::string&extra=""){
 replayEvent("candidate",",\"stage\":\""+std::string(stage)+"\",\"key\":\""+viewKey(p,yaw)+"\",\"xyz\":"+rp(p)+",\"yaw\":"+std::to_string(yaw)+extra);
}
}

namespace fast_planner { inline bool recordCandidateTrace(){static const bool enabled=[](){const char*p=std::getenv("CONSISTENCY_TRACE_CANDIDATES");return !p || std::string(p)!="0";}();return enabled;} }
#define viewEvent(...) do { if(::fast_planner::recordCandidateTrace()) ::fast_planner::viewEventImpl(__VA_ARGS__); } while(false)
