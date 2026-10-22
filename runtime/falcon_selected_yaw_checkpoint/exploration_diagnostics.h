#pragma once
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <cstdlib>
#include <cmath>
#include <fcntl.h>
#include <unistd.h>
#include <sstream>
#include <iomanip>
namespace fast_planner {
inline std::string diagnosticNumber(double value) {
  if(!std::isfinite(value))return "null";
  std::ostringstream out;out<<std::setprecision(12)<<value;return out.str();
}
inline std::string diagnosticPosition(const Eigen::Vector3d &p) {
  return "["+diagnosticNumber(p.x())+","+diagnosticNumber(p.y())+","+diagnosticNumber(p.z())+"]";
}
inline void diagnosticEvent(const char *event,unsigned long long plan,int candidate,const std::string &fields) {
  const char *path=std::getenv("EXPLORATION_DIAGNOSTIC_TRACE");if(!path)return;
  std::ostringstream out;out<<std::setprecision(15)<<"{\"event\":\""<<event<<"\",\"ros_s\":"<<ros::Time::now().toSec()<<",\"wall_s\":"<<ros::WallTime::now().toSec()<<",\"pid\":"<<getpid()<<",\"plan\":"<<plan<<",\"candidate\":"<<candidate<<fields<<"}\n";
  const std::string line=out.str();const int fd=open(path,O_WRONLY|O_CREAT|O_APPEND,0644);
  if(fd>=0){const auto written=write(fd,line.data(),line.size());if(written!=ssize_t(line.size()))ROS_WARN_THROTTLE(10,"Diagnostic trace write incomplete");close(fd);}
}
}
