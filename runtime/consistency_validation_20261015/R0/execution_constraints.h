#ifndef FALCON_EXECUTION_CONSTRAINTS_H
#define FALCON_EXECUTION_CONSTRAINTS_H
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <voxel_mapping/map_server.h>
#include <vector>

namespace fast_planner {
// One observed-world contract shared by FREE-space A*, shortcut scoring,
// recovery selection and trajectory validation. Never reads the scene mesh.
class ExecutionConstraints {
  voxel_mapping::MapServer *map_;
  Eigen::Vector3d origin_;
  std::vector<Eigen::Vector3d> contacts_;
  double radius_, initial_esdf_;
  bool enabled_;
public:
  ExecutionConstraints(voxel_mapping::MapServer *map,const Eigen::Vector3d &origin):map_(map),origin_(origin) {
    ros::param::param("/fast_planner/observed_trajectory_guard",enabled_,false);
    ros::param::param("/fast_planner/execution_clearance",radius_,.07);
    initial_esdf_=map_->getESDF()->getDistance(origin_);
    XmlRpc::XmlRpcValue values;
    if(ros::param::get("/preview/observed_contacts",values) && values.getType()==XmlRpc::XmlRpcValue::TypeArray)
      for(int i=0;i<values.size();++i) if(values[i].getType()==XmlRpc::XmlRpcValue::TypeArray && values[i].size()==3)
        contacts_.emplace_back(double(values[i][0]),double(values[i][1]),double(values[i][2]));
  }
  double contactDistance(const Eigen::Vector3d &p) const {
    double distance=10.;for(const auto &q:contacts_)distance=std::min(distance,(p-q).norm());return distance;
  }
  double clearance(const Eigen::Vector3d &p) const {
    return std::min(map_->getESDF()->getDistance(p),contactDistance(p));
  }
  bool point(const Eigen::Vector3d &p) const {
    if(!p.allFinite() || !map_->getTSDF()->isInBox(map_->getOccupancyGrid()->positionToIndex(p)))return false;
    const auto occupancy=map_->getOccupancy(p);
    if(occupancy==voxel_mapping::OccupancyType::OCCUPIED)return false;
    const auto voxel=map_->getOccupancyGrid()->positionToIndex(p);
    const auto start_voxel=map_->getOccupancyGrid()->positionToIndex(origin_);
    if(occupancy!=voxel_mapping::OccupancyType::FREE && !(voxel.array()==start_voxel.array()).all())return false;
    if(!enabled_)return true;
    const bool escaping=(p-origin_).norm()<=.35;
    const double distance=map_->getESDF()->getDistance(p);
    if(distance<radius_ && !(escaping && initial_esdf_<radius_ && distance>=initial_esdf_-1e-4))return false;
    for(const auto &q:contacts_) {
      const double d=(p-q).norm(),initial=(origin_-q).norm();
      if(d<radius_ && !(escaping && initial<radius_ && d>=initial-1e-4))return false;
    }
    return true;
  }
  bool segment(const Eigen::Vector3d &a,const Eigen::Vector3d &b) const {
    const double length=(b-a).norm();const int samples=std::max(1,int(std::ceil(length/.025)));
    for(int i=0;i<=samples;++i)if(!point(a+(b-a)*(double(i)/samples)))return false;
    return true;
  }
};
}
#endif
