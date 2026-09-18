#include <chrono>
#include <cmath>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_gicp_st.hpp>
#include <fast_gicp/gicp/fast_vgicp.hpp>

#include <so3_math.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include "IMU_Processing.hpp"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <livox_ros_driver2/CustomMsg.h>
#include "preprocess.h"
#include "Utils/tictoc.hpp"

#include <ros/ros.h>

#include "scancontext/Scancontext.h"
#include "scancontext/surface_desc.h"

#include <fstream>
#include <cstdint>
#include <algorithm>
#include <numeric>

// hard code param
int imu_count_for_align_gravity_ = 50;
double icp_convergence_condition_for_pos_ = 0.01; // meter
double icp_convergence_condition_for_rot_ = 1.0; // deg
//descriptor scancontext
SCManager scManager;
SCViewer scViewer;
int test_PC_NUM_RING = 40;
int test_PC_NUM_SECTOR = 120;
double test_PC_MAX_RADIUS = 20;
// param
int param_accumulated_scan_ = 10;
int param_max_iterations_ = 0;
double param_map_size_ = 50;
int param_function_value_ = 1;
double param_voxelgrid_filter_size_;

string initial_map_pcd_name_ = "initial_map.pcd";
Eigen::Vector3d      Init_Body_pos_;
Eigen::Matrix3d      Init_Body_rot_by_align_gravity_;
Eigen::Matrix3d      Init_Body_rot_gravity_ = Eigen::Matrix3d::Identity(); // roll/pitch only
double init_body_yaw_deg_ = 0.0;
double max_fitness_score_ = 2.0;
Eigen::Vector3d Lidar_wrt_Body_T;
Eigen::Matrix3d Lidar_wrt_Body_R;
// The initial scan and the gravity source do not necessarily use the same
// sensor frame.  In the MAVROS configuration the IMU is already in FCU body
// coordinates, while the MID-360 points still need mapping/extrinsic_{R,T}.
// Keep these transforms separate so correcting the scan does not rotate the
// MAVROS acceleration a second time.
Eigen::Vector3d Initial_Scan_Lidar_wrt_Body_T;
Eigen::Matrix3d Initial_Scan_Lidar_wrt_Body_R;

bool receive_source_ = false;
bool imu_init_ready_ = false;
int scan_count_ = 0;
int imu_count_ = 0;
Eigen::Vector3d accumulated_acc_{0, 0, 0};

Eigen::Matrix4f incremental_pose_ = Eigen::Matrix4f::Identity();
Eigen::Matrix4f result_pose_ = Eigen::Matrix4f::Identity();

double init_zone_width = 10.0;
double init_zone_height = 10.0;
double init_resolution = 1.0;
bool print_detail_score_ = false;
bool advanced_by_sc_ = true;
int step_id_ = 1;

// ---- full-map scdb auto initialization ----
std::string init_mode_ = "pose";           // pose | auto
std::string scdb_path_ = "";               // empty => <map_pcd>.scdb
int         scdb_top_k_ = 5;               // GICP candidates from scdb retrieval
double      scdb_accept_fitness_ = 0.5;     // strict SC-stage acceptance (truths <=0.15, lookalikes >=1.15 observed)
bool        scdb_pose_fallback_ = true;    // fall back to configured pose on auto failure
std::string init_source_ = "pose";         // pose | scdb | none (result file)
double      init_fitness_ = -1.0;          // last accepted GICP fitness
pcl::PointCloud<pcl::PointXYZ>::Ptr full_map_cloud_(new pcl::PointCloud<pcl::PointXYZ>());

struct SCDBEntry { double x = 0.0, y = 0.0; Eigen::MatrixXd desc, key; };
std::vector<SCDBEntry> scdb_entries_;

struct AlignResult { double fitness; Eigen::Matrix4d T; };

// target 是地图，source 是点云
pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_raw_(new pcl::PointCloud<pcl::PointXYZ>());
// Matching input uses the same local radius as the cropped map target.  Keep
// source_cloud_raw_ complete for diagnostics and descriptor generation.
pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_match_(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_sc_(new pcl::PointCloud<pcl::PointXYZ>());

pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_raw_(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_with_g_(new pcl::PointCloud<pcl::PointXYZ>());

shared_ptr<Preprocess> p_pre_(new Preprocess());

//------------------------------------------------------------------------------------------------------

void Eigen_to_PCL(const Eigen::Vector3d &Eigen_tp, pcl::PointXYZ &pcl_tp)
{
    pcl_tp.x = Eigen_tp[0];
    pcl_tp.y = Eigen_tp[1];
    pcl_tp.z = Eigen_tp[2];
}

void PCL_to_Eigen(const pcl::PointXYZ &pcl_tp, Eigen::Vector3d &Eigen_tp)
{
    Eigen_tp[0] = pcl_tp.x;
    Eigen_tp[1] = pcl_tp.y;
    Eigen_tp[2] = pcl_tp.z;
}

void M4_to_RT(Eigen::Matrix4f pose, Eigen::Vector3d &pos, Eigen::Matrix3d &rot)
{
    pos = pose.block<3,1>(0,3).cast<double>();
    rot = pose.block<3,3>(0,0).cast<double>();
}

void RT_to_M4(Eigen::Vector3d pos, Eigen::Matrix3d rot, Eigen::Matrix4f &pose)
{
    pose.block<3,1>(0,3) = pos.cast<float>();
    pose.block<3,3>(0,0) = rot.cast<float>();
}

void multiM4matrix(Eigen::Matrix4f &source,Eigen::Matrix4f &poses_pub)
{
    // poses_pub.block<3,3>(0,0) = source.block<3,3>(0,0)*poses_pub.block<3,3>(0,0);
    // poses_pub.block<3,1>(0,3)(0) = poses_pub.block<3,1>(0,3)(0) + source.block<3,1>(0,3)(0);
    // poses_pub.block<3,1>(0,3)(1) = poses_pub.block<3,1>(0,3)(1) + source.block<3,1>(0,3)(1);
    // poses_pub.block<3,1>(0,3)(2) = poses_pub.block<3,1>(0,3)(2) + source.block<3,1>(0,3)(2);   

    poses_pub = source * poses_pub;
}

void translate_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Vector3d pos, Eigen::Matrix3d rot)
{
    for( int i = 0; i < cloud->points.size(); i++ )
    {
        Eigen::Vector3d tp;
        PCL_to_Eigen(cloud->points[i], tp);
        tp = rot * tp + pos;
        Eigen_to_PCL(tp, cloud->points[i]);
    }
}

void translate_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Matrix4f pose)
{
    Eigen::Vector3d pos;
    Eigen::Matrix3d rot;
    M4_to_RT(pose, pos, rot);
    translate_cloud(cloud, pos, rot);
}

void publish_frame(const ros::Publisher & publisher, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*cloud, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time::now();
    laserCloudmsg.header.frame_id = "world";
    publisher.publish(laserCloudmsg);
}

void publish_frame(const ros::Publisher & publisher, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Vector3d pos, Eigen::Matrix3d rot)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr copy_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    *copy_cloud = *cloud;

    translate_cloud(copy_cloud, pos, rot);
    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*copy_cloud, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time::now();
    laserCloudmsg.header.frame_id = "world";
    publisher.publish(laserCloudmsg);
}

void publish_frame_world(const ros::Publisher & publisher, Eigen::Vector3d pos, Eigen::Matrix3d rot)
{
    static bool translate_flag = false;
    if( !translate_flag )
    {
        translate_cloud(source_cloud_raw_, pos, rot);
    }
    translate_flag = true;

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*source_cloud_raw_, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time::now();
    laserCloudmsg.header.frame_id = "world";
    publisher.publish(laserCloudmsg);
}

// benchmark for fast_gicp registration methods
template <typename Registration>
bool test(Registration& reg, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& target, pcl::PointCloud<pcl::PointXYZ>::Ptr& source,int iteration_num)
{
    std::cout <<"=======================================" << std::endl;
    std::cout << "\033[1;33m[wxx][initial align] Step " << step_id_++ << ":  Fine Align by GICP " << "\033[0m" << std::endl;
    Eigen::Matrix4f relative_pose = Eigen::Matrix4f::Identity();
    const double conv_rot_rad = icp_convergence_condition_for_rot_ * M_PI / 180.0;

    TicToc timer;
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>);
    if (target->empty() || source->empty() || iteration_num <= 0)
    {
        ROS_ERROR("[wxx][initial align] Registration requires non-empty source/target clouds and positive iteration count.");
        return false;
    }
    for (int i = 0; i < iteration_num; i++) {
        reg.setInputTarget(target);
        reg.setInputSource(source);
        reg.align(*aligned);
        incremental_pose_ = reg.getFinalTransformation();
        const double fitness_score = reg.getFitnessScore();
        if (!reg.hasConverged() || !std::isfinite(fitness_score) ||
            fitness_score > max_fitness_score_ || !incremental_pose_.allFinite())
        {
            ROS_ERROR_STREAM("[wxx][initial align] Registration failed at iteration " << i + 1
                             << ": converged=" << reg.hasConverged()
                             << ", fitness=" << fitness_score
                             << ", max_fitness=" << max_fitness_score_);
            return false;
        }

        translate_cloud(source, incremental_pose_);

        multiM4matrix(incremental_pose_, result_pose_);
        multiM4matrix(incremental_pose_, relative_pose);
        std::cout <<"------" << std::endl;
        std::cout << "\033[0;32miteration: " << "\033[0m" << i+1 << std::endl;
        std::cout << "\033[0;32mincremental pos in single iteration: " << "\033[0m" << std::endl;
        std::cout << incremental_pose_.block<3,1>(0,3).transpose() << std::endl;
        std::cout << "\033[0;32mincremental rot in single iteration: " << "\033[0m" << std::endl;
        std::cout << incremental_pose_.block<3,3>(0,0).cast<double>()<<std::endl;

        if( incremental_pose_.block<3,1>(0,3).norm() < icp_convergence_condition_for_pos_ )
        {
            Eigen::Matrix3d R = incremental_pose_.block<3,3>(0,0).cast<double>();
            double sy = std::sqrt(R(0,0) * R(0,0) + R(1,0) * R(1,0));
            double x, y, z;
            x = std::atan2(R(2,1), R(2,2));
            y = std::atan2(-R(2,0), sy);
            z = std::atan2(R(1,0), R(0,0));
            if( x < conv_rot_rad &&
                y < conv_rot_rad &&
                z < conv_rot_rad )
            {
                std::cout << "\033[1;34mICP convergence" << "\033[0m" << std::endl;
                break;
            }
        }

    }
    double time = timer.toc();
    std::cout <<"-------------------------------------" << std::endl;
    std::cout << "\033[1;34mICP time: " << "\033[0m" << time << " ms" << std::endl;
    std::cout << "\033[0;32mrelative pos in ICP: " << "\033[0m" << std::endl;
    std::cout << relative_pose.block<3,1>(0,3).transpose() << std::endl;
    std::cout << "\033[0;32mrelative rot in ICP: " << "\033[0m" << std::endl;
    std::cout << relative_pose.block<3,3>(0,0).cast<double>()<<std::endl;
    init_fitness_ = reg.getFitnessScore();
    init_source_ = "pose";
    return true;
}

void livox_pcl_cbk(const livox_ros_driver2::CustomMsg::ConstPtr &msg) 
{
    if(receive_source_)
        return;

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre_->process(msg, ptr);
    
    scan_count_++;
    if(scan_count_ == param_accumulated_scan_)
    {
        receive_source_ = true;
    }

    for (const auto& pt : ptr->points) 
    {
        pcl::PointXYZ xyz_point;
        xyz_point.x = pt.x;
        xyz_point.y = pt.y;
        xyz_point.z = pt.z;
        source_cloud_->points.push_back(xyz_point);
    }
}

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg) 
{
    if(receive_source_)
        return;

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre_->process(msg, ptr);
    
    scan_count_++;
    if(scan_count_ == param_accumulated_scan_)
    {
        receive_source_ = true;
    }

    for (const auto& pt : ptr->points) 
    {
        pcl::PointXYZ xyz_point;
        xyz_point.x = pt.x;
        xyz_point.y = pt.y;
        xyz_point.z = pt.z;
        source_cloud_->points.push_back(xyz_point);
    }
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in) 
{
    if(imu_init_ready_)
        return;

    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));
    Eigen::Vector3d acc{msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z};
    acc = Lidar_wrt_Body_R * acc;
    accumulated_acc_ += acc;

    imu_count_ ++;
    if(imu_count_ == imu_count_for_align_gravity_)
    {
        imu_init_ready_ = true;
        accumulated_acc_.normalize();
        std::cout <<"=======================================" << std::endl;
        std::cout << "\033[1;33m[wxx][initial align] Step " << step_id_++ << ":  Gravity Align " << "\033[0m" << std::endl;
        std::cout << "\033[0;32maccumulated acceleration norm: " << "\033[0m" << std::endl;
        std::cout << accumulated_acc_.transpose() << std::endl;
        const Eigen::Matrix3d gravity_rotation =
            Eigen::Quaterniond::FromTwoVectors(accumulated_acc_, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
        const Eigen::Matrix3d yaw_rotation = Eigen::AngleAxisd(
            init_body_yaw_deg_ * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Init_Body_rot_by_align_gravity_ = yaw_rotation * gravity_rotation;
        Init_Body_rot_gravity_ = gravity_rotation; // roll/pitch only (no yaw)
    }
}


void scancontext_cal_yaw(pcl::PointCloud<pcl::PointXYZ>::Ptr& target, pcl::PointCloud<pcl::PointXYZ>::Ptr& source)
{
    scManager.PC_NUM_RING = test_PC_NUM_RING;
    scManager.PC_NUM_SECTOR = test_PC_NUM_SECTOR;
    scManager.PC_MAX_RADIUS = test_PC_MAX_RADIUS;
    scManager.PC_UNIT_SECTORANGLE = 360.0 / double(scManager.PC_NUM_SECTOR);
    scManager.PC_UNIT_RINGGAP = scManager.PC_MAX_RADIUS / double(scManager.PC_NUM_RING);

    Eigen::MatrixXd target_sc = scManager.makeScancontext(*target);
    scManager.target_scancontext = target_sc;
    // Eigen::MatrixXd target_ringkey = scManager.makeRingkeyFromScancontext(target_sc);
    // Eigen::MatrixXd target_sectorkey = scManager.makeSectorkeyFromScancontext(target_sc);
    // std::vector<float> target_polarcontext_invkey_vec = eig2stdvec(target_ringkey);

    Eigen::MatrixXd source_sc = scManager.makeScancontext(*source);
    scManager.source_scancontext = source_sc;
    // Eigen::MatrixXd source_ringkey = scManager.makeRingkeyFromScancontext(source_sc);
    // Eigen::MatrixXd source_sectorkey = scManager.makeSectorkeyFromScancontext(source_sc);
    // std::vector<float> source_polarcontext_invkey_vec = eig2stdvec(source_ringkey);
    std::pair<double, int> sc_dist_result = scManager.distanceBtnScanContext(target_sc, source_sc); 
    double dist = sc_dist_result.first;
    int align = sc_dist_result.second;
    float yaw_diff_rad = deg2rad(align * scManager.PC_UNIT_SECTORANGLE);
    std::cout << "\033[1;34myaw_diff_deg: "<< yaw_diff_rad * 180 / M_PI << "\033[0m" << std::endl;
    Eigen::Matrix4f sc_pose_ = Eigen::Matrix4f::Identity();
    sc_pose_.block<3,1>(0,3) << 0, 0, 0;
    sc_pose_.block<3,3>(0,0) = Eigen::AngleAxisf(yaw_diff_rad, Eigen::Vector3f::UnitZ()).matrix();
    translate_cloud(source_cloud_, sc_pose_);
    multiM4matrix(sc_pose_, result_pose_);
    *source_cloud_sc_ = *source_cloud_;
}


void coarse_init_pose(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& target, pcl::PointCloud<pcl::PointXYZ>::Ptr& source) 
{
    TicToc timer;
    double time1, time2;
    std::cout <<"=======================================" << std::endl;
    std::cout << "\033[1;33m[wxx][initial align] Step " << step_id_++ << ":  Coarse Align by Scan Context " << "\033[0m" << std::endl;

    Eigen::Matrix3d custom_R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d custom_T;

    custom_T = Eigen::Vector3d(0, 0, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>());

    for(custom_T[0] = -init_zone_height / 2; custom_T[0] < init_zone_height / 2; custom_T[0] += init_resolution)
    {
        for(custom_T[1] = -init_zone_width / 2; custom_T[1] < init_zone_width / 2; custom_T[1] += init_resolution)
        {
            *temp_cloud = *target;
            translate_cloud(temp_cloud,custom_T, custom_R);
            
            Eigen::MatrixXd target_sc = scManager.makeScancontext(*temp_cloud); 
            scManager.polarcontexts_.push_back( target_sc ); 
            scManager.custom_R_candidate.push_back( custom_R );
            scManager.custom_T_candidate.push_back( custom_T );
            scManager.target_cloud_candidate_.push_back( *temp_cloud );
            // std::cout << "add new candidate" << std::endl;
        }
    }
    time1 = timer.toc();

    int best_index = -1;
    Eigen::MatrixXd source_sc = scManager.makeScancontext(*source);
    scManager.source_scancontext = source_sc;
    auto detectResult = scManager.detect_init_pose(print_detail_score_); // first: nn index, second: yaw diff 
    best_index = detectResult.first;
    if( best_index != -1 )
    {
        std::cout << "\033[0;32m" << "candidates number: " << "\033[0m" << scManager.polarcontexts_.size() << std::endl;
        std::cout << "\033[0;32m" << "closest history frame ID: " << "\033[0m" << best_index << std::endl;
        float yaw_diff_rad = detectResult.second;
        // std::cout << "\033[0;32m" << "yaw diff: " << "\033[0m" << yaw_diff_rad * 180 / M_PI << "[deg]" << std::endl;
        Eigen::Matrix4f sc_pose_ = Eigen::Matrix4f::Identity();
        sc_pose_.block<3,1>(0,3) = -scManager.custom_T_candidate[best_index].cast<float>();
        sc_pose_.block<3,3>(0,0) = Eigen::AngleAxisf(yaw_diff_rad, Eigen::Vector3f::UnitZ()).matrix();

        std::cout << "\033[0;32mrelative pos in Scan Context: " << "\033[0m" << std::endl;
        std::cout << sc_pose_.block<3,1>(0,3).transpose() << std::endl;
        std::cout << "\033[0;32mrelative yaw in Scan Context: " << "\033[0m" << yaw_diff_rad * 180 / M_PI << "[deg]" << std::endl;

        translate_cloud(source, sc_pose_);
        multiM4matrix(sc_pose_, result_pose_);
        *source_cloud_sc_ = *source;
    } 
    else
    {
        std::cout << "Error." << std::endl;
        ROS_ERROR("[wxx][initial align] Scan Context Error !!!");
        exit(0);
    }
    
    time2 = timer.toc();
    std::cout << "\033[1;34m" << "add candidates time: " << "\033[0m" << time1 << " ms" << std::endl;
    std::cout << "\033[1;34m" << "select the best candidate time: " << "\033[0m" << time2 - time1 << " ms" << std::endl;

    return;
}


// ---------------------------------------------------------------------------
// Full-map descriptor DB (scdb) support: built once per venue with
// tests/build_map_scdb.cpp, loaded here for auto initialization.
// ---------------------------------------------------------------------------

bool load_scdb(const std::string& path)
{
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        ROS_WARN_STREAM("[wxx][initial align] scdb not found: " << path);
        return false;
    }
    char magic[5] = {0, 0, 0, 0, 0};
    uint32_t version = 0, count = 0;
    float grid_step = 0.0f;
    f.read(magic, 5);
    f.read(reinterpret_cast<char*>(&version), sizeof(version));
    f.read(reinterpret_cast<char*>(&grid_step), sizeof(grid_step));
    f.read(reinterpret_cast<char*>(&count), sizeof(count));
    if (std::string(magic, 5) != "SCDB1" || version != 3) {
        ROS_ERROR_STREAM("[wxx][initial align] invalid scdb version in " << path
                         << " (expected 3); regenerate with ./build_map_scdb.sh");
        return false;
    }
    scdb_entries_.clear();
    scdb_entries_.reserve(count);
    for (uint32_t i = 0; i < count; ++i) {
        SCDBEntry e;
        float x = 0.0f, y = 0.0f;
        f.read(reinterpret_cast<char*>(&x), sizeof(x));
        f.read(reinterpret_cast<char*>(&y), sizeof(y));
        e.x = x;
        e.y = y;
        e.desc.resize(test_PC_NUM_RING, test_PC_NUM_SECTOR);
        for (int r = 0; r < test_PC_NUM_RING; ++r)
            for (int s = 0; s < test_PC_NUM_SECTOR; ++s) {
                float v = 0.0f;
                f.read(reinterpret_cast<char*>(&v), sizeof(v));
                e.desc(r, s) = v;
            }
        e.key.resize(test_PC_NUM_RING, 1);
        for (int r = 0; r < test_PC_NUM_RING; ++r) {
            float v = 0.0f;
            f.read(reinterpret_cast<char*>(&v), sizeof(v));
            e.key(r, 0) = v;
        }
        scdb_entries_.push_back(std::move(e));
    }
    ROS_INFO_STREAM("[wxx][initial align] scdb loaded: " << scdb_entries_.size()
                    << " entries from " << path);
    return true;
}

// Retrieve the drone pose anywhere in the map: Scan Context ring-key KNN over
// the full-map descriptor DB, then fine-align the top-K candidates with
// fast_gicp and accept only below max_fitness_score_. On success result_pose_
// becomes the final world pose. source_cloud_ must already be translated into
// the gravity-aligned init frame (result_pose_).
void auto_align_by_scdb(std::vector<AlignResult>& pool)
{
    if (scdb_entries_.empty())
        return;

    SCManager sc;
    sc.PC_NUM_RING = test_PC_NUM_RING;
    sc.PC_NUM_SECTOR = test_PC_NUM_SECTOR;
    sc.PC_MAX_RADIUS = test_PC_MAX_RADIUS;
    sc.PC_UNIT_SECTORANGLE = 360.0 / double(sc.PC_NUM_SECTOR);
    sc.PC_UNIT_RINGGAP = sc.PC_MAX_RADIUS / double(sc.PC_NUM_RING);

    // query descriptor from the raw lidar-frame scan (pre-gravity
    // translation). Surface-band encoding must match the DB builder.
    Eigen::MatrixXd src_desc = make_surface_desc(*source_cloud_raw_);

    // Exhaustive Scan Context distance over the whole DB. The ring key is a
    // weak 40-dim summary; on real scans the true cell can easily miss the
    // ring-key top-5. A one-shot init can afford the full sweep (~0.3s for a
    // few thousand cells), and it removes that failure mode entirely.
    struct Cand { int idx; double sc_dist; double yaw_rad; };
    std::vector<Cand> cands;
    cands.reserve(scdb_entries_.size());
    for (size_t i = 0; i < scdb_entries_.size(); ++i) {
        const auto res = sc.distanceBtnScanContext(src_desc, scdb_entries_[i].desc);
        // validated convention (tests/sc_init_validate.cpp): returned yaw is
        // the negation of the world heading.
        cands.push_back({static_cast<int>(i), res.first,
                         -res.second * sc.PC_UNIT_SECTORANGLE * M_PI / 180.0});
    }
    std::sort(cands.begin(), cands.end(),
              [](const Cand& a, const Cand& b) { return a.sc_dist < b.sc_dist; });

    // Spatial dedupe: adjacent cells share nearly identical descriptors and
    // would waste GICP slots on the same hypothesis.
    std::vector<Cand> diverse;
    diverse.reserve(8);
    for (const Cand& c : cands) {
        bool too_close = false;
        for (const Cand& d : diverse) {
            const double dx = scdb_entries_[c.idx].x - scdb_entries_[d.idx].x;
            const double dy = scdb_entries_[c.idx].y - scdb_entries_[d.idx].y;
            if (dx * dx + dy * dy < 1.5 * 1.5) { too_close = true; break; }
        }
        if (!too_close)
            diverse.push_back(c);
        if (diverse.size() >= 8)
            break;
    }
    cands = diverse;
    ROS_INFO_STREAM("[wxx][initial align] scdb retrieval done, top candidates:");
    for (size_t k = 0; k < cands.size(); ++k) {
        ROS_INFO_STREAM("[wxx][initial align]   #" << k
                        << " dist=" << cands[k].sc_dist
                        << " pos=(" << scdb_entries_[cands[k].idx].x << ", "
                        << scdb_entries_[cands[k].idx].y << ")"
                        << " yaw=" << cands[k].yaw_rad * 180.0 / M_PI << "deg");
    }

    const int top_gicp = std::min<int>(scdb_top_k_, cands.size());
    for (int k = 0; k < top_gicp; ++k) {
        const Cand& c = cands[k];

        // world-frame target crop around the candidate (from the voxeled map)
        pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>());
        const double crop_r = test_PC_MAX_RADIUS + 2.0;
        for (const auto& p : target_cloud_->points) {
            const double dx = p.x - scdb_entries_[c.idx].x;
            const double dy = p.y - scdb_entries_[c.idx].y;
            if (dx * dx + dy * dy > crop_r * crop_r)
                continue;
            target->points.push_back(p);
        }
        target->width = target->points.size();
        target->height = 1;
        target->is_dense = true;
        if (target->points.size() < 500)
            continue;

        // world pose guess: candidate position + scdb yaw + gravity attitude
        Eigen::Matrix4d T_guess_world = Eigen::Matrix4d::Identity();
        T_guess_world.block<3,3>(0,0) =
            Eigen::AngleAxisd(c.yaw_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix()
            * Init_Body_rot_by_align_gravity_;
        T_guess_world(0,3) = scdb_entries_[c.idx].x;
        T_guess_world(1,3) = scdb_entries_[c.idx].y;
        T_guess_world(2,3) = Init_Body_pos_[2];

        // source_cloud_ lives in the gravity-aligned init frame
        Eigen::Matrix4d guess = T_guess_world * result_pose_.cast<double>().inverse();

        fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ> reg;
        reg.setInputTarget(target);
        reg.setInputSource(source_cloud_match_);
        reg.setMaximumIterations(64);
        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());
        reg.align(*aligned, guess.cast<float>());
        const double fitness = reg.hasConverged() ? reg.getFitnessScore() : -1.0;
        const Eigen::Matrix4d T_world =
            reg.getFinalTransformation().cast<double>() * result_pose_.cast<double>();
        ROS_INFO_STREAM("[wxx][initial align]   GICP #" << k
                        << " cand=(" << scdb_entries_[c.idx].x << ", "
                        << scdb_entries_[c.idx].y << ")"
                        << " yaw=" << c.yaw_rad * 180.0 / M_PI << "deg"
                        << " converged=" << reg.hasConverged()
                        << " fitness=" << fitness);
        if (fitness < 0.0 || !std::isfinite(fitness))
            continue;
        pool.push_back({fitness, T_world});
    }
    ROS_INFO_STREAM("[wxx][initial align] scdb proposals: " << pool.size()
                    << " converged candidates");
}

// Fallback: cheap GICP sweep on a 3x3 grid (+-8m) around the configured
// INIT pose with two yaw seeds, then fine GICP on the top-2 clusters.
// Validated on NX field data: finds the true pose from a coarse INIT hint
// (within ~8m) when the SC retrieval fails (lookalike regions).
void local_sweep_align(std::vector<AlignResult>& pool)
{
    const double off[3] = {-8.0, 0.0, 8.0};
    const double yaw0 = init_body_yaw_deg_ * M_PI / 180.0;
    const double seeds[2] = {yaw0, yaw0 + M_PI / 2};

    // cheap source (voxel 0.3) from the raw lidar-frame scan
    pcl::PointCloud<pcl::PointXYZ>::Ptr src_cheap(new pcl::PointCloud<pcl::PointXYZ>());
    {
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setLeafSize(0.3, 0.3, 0.3);
        vg.setInputCloud(source_cloud_match_);
        vg.filter(*src_cheap);
    }

    struct SweepRes { double f, wx, wy, wz, yaw; };
    std::vector<SweepRes> all;
    for (double ox : off)
        for (double oy : off) {
            const double cx = Init_Body_pos_[0] + ox;
            const double cy = Init_Body_pos_[1] + oy;
            pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>());
            for (const auto& p : target_cloud_->points) {
                const double dx = p.x - cx, dy = p.y - cy;
                if (dx * dx + dy * dy > 14.0 * 14.0)
                    continue;
                target->points.push_back(p);
            }
            target->width = target->points.size();
            target->height = 1;
            target->is_dense = true;
            if (target->points.size() < 300)
                continue;
            pcl::PointCloud<pcl::PointXYZ>::Ptr target_cheap(new pcl::PointCloud<pcl::PointXYZ>());
            {
                pcl::VoxelGrid<pcl::PointXYZ> vg;
                vg.setLeafSize(0.6, 0.6, 0.6);
                vg.setInputCloud(target);
                vg.filter(*target_cheap);
            }
            if (target_cheap->points.size() < 200)
                continue;
            for (double yw : seeds) {
                fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ> reg;
                reg.setInputTarget(target_cheap);
                reg.setInputSource(src_cheap);
                reg.setMaximumIterations(20);
                Eigen::Matrix4d guess = Eigen::Matrix4d::Identity();
                guess.block<3,3>(0,0) =
                    Eigen::AngleAxisd(yw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
                pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());
                reg.align(*aligned, guess.cast<float>());
                if (!reg.hasConverged())
                    continue;
                const double f = reg.getFitnessScore();
                if (!std::isfinite(f))
                    continue;
                const Eigen::Matrix4d T = reg.getFinalTransformation().cast<double>();
                const double wyaw = std::atan2(T(1,0), T(0,0));
                // The target crop retains world-frame coordinates, so GICP's
                // final translation is already a world position. Adding the
                // crop center here creates duplicate solutions separated by
                // the sweep offsets.
                all.push_back({f, T(0,3), T(1,3), T(2,3), wyaw});
            }
        }
    if (all.empty())
        return;
    std::sort(all.begin(), all.end(),
              [](const SweepRes& a, const SweepRes& b) { return a.f < b.f; });

    // cluster converged results by final pose (3m), keep cluster best
    std::vector<SweepRes> clusters;
    for (const auto& r : all) {
        bool merged = false;
        for (const auto& c : clusters) {
            if (std::hypot(r.wx - c.wx, r.wy - c.wy) < 3.0) { merged = true; break; }
        }
        if (!merged)
            clusters.push_back(r);
    }

    // fine GICP on the top-2 clusters (full-res scan, 22m crops)
    struct FineRes { double f; Eigen::Matrix4d T; };
    std::vector<FineRes> fines;
    for (size_t i = 0; i < std::min<size_t>(2, clusters.size()); ++i) {
        const auto& c = clusters[i];
        pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>());
        for (const auto& p : target_cloud_->points) {
            const double dx = p.x - c.wx, dy = p.y - c.wy;
            if (dx * dx + dy * dy > 22.0 * 22.0)
                continue;
            target->points.push_back(p);
        }
        target->width = target->points.size();
        target->height = 1;
        target->is_dense = true;
        fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ> reg;
        reg.setInputTarget(target);
        reg.setInputSource(source_cloud_match_);
        reg.setMaximumIterations(64);
        Eigen::Matrix4d guess = Eigen::Matrix4d::Identity();
        guess.block<3,3>(0,0) =
            Eigen::AngleAxisd(c.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());
        reg.align(*aligned, guess.cast<float>());
        if (!reg.hasConverged())
            continue;
        const double f = reg.getFitnessScore();
        if (!std::isfinite(f))
            continue;
        Eigen::Matrix4d T_world = reg.getFinalTransformation().cast<double>();
        // As above, the target points are not recentered around c, therefore
        // this transform is already expressed in the map world frame.
        ROS_INFO_STREAM("[wxx][initial align] sweep cluster #" << i
                        << " from=(" << c.wx << ", " << c.wy << ") fitness=" << f
                        << " final=(" << T_world(0,3) << ", " << T_world(1,3)
                        << ", " << T_world(2,3) << ")");
        fines.push_back({f, T_world});
    }
    for (const auto& f : fines)
        pool.push_back({f.f, f.T});
    ROS_INFO_STREAM("[wxx][initial align] sweep proposals: " << fines.size()
                    << " fine clusters");
}

// Unified verdict over proposals from both independent mechanisms
// (SC retrieval and the local GICP sweep). The best cluster must be clearly
// better than the runner-up; otherwise reject (fail safe).
bool finalize_alignment(const std::vector<AlignResult>& pool)
{
    if (pool.empty()) {
        ROS_WARN("[wxx][initial align] no converged proposals");
        return false;
    }
    std::vector<AlignResult> sorted = pool;
    std::sort(sorted.begin(), sorted.end(),
              [](const AlignResult& a, const AlignResult& b) { return a.fitness < b.fitness; });
    std::vector<AlignResult> clusters;
    for (const auto& r : sorted) {
        bool merged = false;
        for (const auto& c : clusters) {
            const double d = std::hypot(r.T(0,3) - c.T(0,3), r.T(1,3) - c.T(1,3));
            if (d < 3.0) { merged = true; break; }
        }
        if (!merged)
            clusters.push_back(r);
    }
    if (clusters[0].fitness > scdb_accept_fitness_) {
        ROS_WARN_STREAM("[wxx][initial align] rejected: best cluster fitness "
                        << clusters[0].fitness << " > accept=" << scdb_accept_fitness_);
        return false;
    }
    if (clusters.size() >= 2 && clusters[1].fitness < 1.5 * clusters[0].fitness) {
        ROS_WARN_STREAM("[wxx][initial align] ambiguous: second cluster fitness "
                        << clusters[1].fitness << " too close to best " << clusters[0].fitness);
        return false;
    }
    result_pose_ = clusters[0].T.cast<float>();
    init_fitness_ = clusters[0].fitness;
    init_source_ = "auto";
    ROS_INFO_STREAM("[wxx][initial align] accepted: fitness=" << clusters[0].fitness
                    << " pos=(" << clusters[0].T(0,3) << ", " << clusters[0].T(1,3)
                    << ", " << clusters[0].T(2,3) << ")");
    return true;
}

// The pre-existing GICP fine alignment from the configured initial pose.
bool align_from_pose()
{
    switch(param_function_value_)
    {
        case 1:{
            pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> pcl_gicp;
            return test(pcl_gicp, target_cloud_, source_cloud_, param_max_iterations_);
        }
        case 2:{
            pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> pcl_ndt;
            pcl_ndt.setResolution(1.0);
            return test(pcl_ndt, target_cloud_, source_cloud_, param_max_iterations_);
        }
        case 3:{
            fast_gicp::FastGICPSingleThread<pcl::PointXYZ, pcl::PointXYZ> fgicp_st;
            return test(fgicp_st, target_cloud_, source_cloud_, param_max_iterations_);
        }
        case 4:{
            fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ> fgicp_mt;
            return test(fgicp_mt, target_cloud_, source_cloud_, param_max_iterations_);
        }
        case 5:{
            fast_gicp::FastVGICP<pcl::PointXYZ, pcl::PointXYZ> vgicp;
            vgicp.setResolution(1.0);
            vgicp.setNumThreads(1);
            return test(vgicp, target_cloud_, source_cloud_, param_max_iterations_);
        }
        default:
            return false;
    }
}

// Machine-readable result for the GCS agent: it reports the ACTUAL converged
// pose (and its source) so the ground station can verify and remember it.
void write_init_result(bool ok, const std::string& error)
{
    const char* path = "/tmp/lio_init_result.yaml";
    std::ofstream f(path);
    if (!f) {
        ROS_WARN_STREAM("[wxx][initial align] cannot write " << path);
        return;
    }
    f << "success: " << (ok ? "true" : "false") << "\n";
    f << "source: " << (ok ? init_source_ : "none") << "\n";
    f << "error: \"" << error << "\"\n";
    f << "map: \"" << initial_map_pcd_name_ << "\"\n";
    if (ok) {
        const Eigen::Vector3d pos = result_pose_.block<3,1>(0,3).cast<double>();
        const Eigen::Matrix3d rot = result_pose_.block<3,3>(0,0).cast<double>();
        const double yaw = std::atan2(rot(1,0), rot(0,0));
        f << "x: " << pos[0] << "\n";
        f << "y: " << pos[1] << "\n";
        f << "z: " << pos[2] << "\n";
        f << "yaw_deg: " << yaw * 180.0 / M_PI << "\n";
    }
    f << "fitness: " << init_fitness_ << "\n";
    f << "timestamp: " << ros::Time::now().toSec() << "\n";
    f.close();

    // On failure keep the accumulated scan for offline replay/debugging.
    if (!ok && source_cloud_raw_ && !source_cloud_raw_->empty()) {
        const std::string dump_path = "/tmp/lio_scan_dump.pcd";
        if (pcl::io::savePCDFileBinary(dump_path, *source_cloud_raw_) == 0)
            ROS_INFO_STREAM("[wxx][initial align] scan dump saved: " << dump_path);
    }
}


int main(int argc, char** argv) 
{
    ros::init(argc, argv, "initial_align");
    ros::NodeHandle nh;
    string  lid_topic, imu_topic;
    nh.param<string>("common/lid_topic",lid_topic,"/livox/lidar");
    nh.param<string>("common/imu_topic", imu_topic,"/livox/imu");

    nh.param<double>("preprocess/blind", p_pre_->blind, 0.01);
    nh.param<int>("preprocess/lidar_type", p_pre_->lidar_type, AVIA);
    nh.param<int>("preprocess/scan_line", p_pre_->N_SCANS, 16);
    nh.param<int>("preprocess/timestamp_unit", p_pre_->time_unit, US);
    nh.param<int>("preprocess/scan_rate", p_pre_->SCAN_RATE, 10);
    nh.param<int>("point_filter_num", p_pre_->point_filter_num, 2);
    nh.param<bool>("feature_extract_enable", p_pre_->feature_enabled, false);
    cout<<"p_pre_->lidar_type "<<p_pre_->lidar_type<<endl;

    nh.param<double>("wxx/initial_align/Init_Body_pos_x", Init_Body_pos_[0], 0.0);
    nh.param<double>("wxx/initial_align/Init_Body_pos_y", Init_Body_pos_[1], 0.0);
    nh.param<double>("wxx/initial_align/Init_Body_pos_z", Init_Body_pos_[2], 0.0);
    nh.param<double>("wxx/initial_align/Init_Body_yaw_deg", init_body_yaw_deg_, 0.0);
    nh.param<string>("wxx/initial_map_pcd_name", initial_map_pcd_name_, "initial_map.pcd");
    vector<double> Lidar_wrt_Body_T_vec(3, 0.0);
    vector<double> Lidar_wrt_Body_R_vec(9, 0.0);
    nh.param<vector<double>>("wxx/Lidar_wrt_Body_T", Lidar_wrt_Body_T_vec, vector<double>());
    nh.param<vector<double>>("wxx/Lidar_wrt_Body_R", Lidar_wrt_Body_R_vec, vector<double>());
    Lidar_wrt_Body_T << Lidar_wrt_Body_T_vec[0], Lidar_wrt_Body_T_vec[1], Lidar_wrt_Body_T_vec[2];
    Lidar_wrt_Body_R << Lidar_wrt_Body_R_vec[0], Lidar_wrt_Body_R_vec[1], Lidar_wrt_Body_R_vec[2],
                        Lidar_wrt_Body_R_vec[3], Lidar_wrt_Body_R_vec[4], Lidar_wrt_Body_R_vec[5],
                        Lidar_wrt_Body_R_vec[6], Lidar_wrt_Body_R_vec[7], Lidar_wrt_Body_R_vec[8];

    // Backward compatible default: the original Livox-IMU setup uses the
    // wxx transform for both scan preprocessing and gravity alignment.  The
    // MAVROS setup opts in to mapping/extrinsic_{R,T} for scan preprocessing
    // only, because /mavros/imu/data is already expressed in body axes.
    bool scan_extrinsic_from_mapping = false;
    nh.param<bool>("wxx/initial_align/scan_extrinsic_from_mapping",
                   scan_extrinsic_from_mapping, false);
    Initial_Scan_Lidar_wrt_Body_T = Lidar_wrt_Body_T;
    Initial_Scan_Lidar_wrt_Body_R = Lidar_wrt_Body_R;
    if (scan_extrinsic_from_mapping)
    {
        vector<double> mapping_extrinsic_T_vec(3, 0.0);
        vector<double> mapping_extrinsic_R_vec(9, 0.0);
        nh.param<vector<double>>("mapping/extrinsic_T", mapping_extrinsic_T_vec,
                                 vector<double>());
        nh.param<vector<double>>("mapping/extrinsic_R", mapping_extrinsic_R_vec,
                                 vector<double>());
        if (mapping_extrinsic_T_vec.size() != 3 || mapping_extrinsic_R_vec.size() != 9)
        {
            ROS_ERROR("[wxx][initial align] mapping extrinsic must contain 3 translation and 9 rotation values");
            return 2;
        }
        Initial_Scan_Lidar_wrt_Body_T <<
            mapping_extrinsic_T_vec[0], mapping_extrinsic_T_vec[1], mapping_extrinsic_T_vec[2];
        Initial_Scan_Lidar_wrt_Body_R <<
            mapping_extrinsic_R_vec[0], mapping_extrinsic_R_vec[1], mapping_extrinsic_R_vec[2],
            mapping_extrinsic_R_vec[3], mapping_extrinsic_R_vec[4], mapping_extrinsic_R_vec[5],
            mapping_extrinsic_R_vec[6], mapping_extrinsic_R_vec[7], mapping_extrinsic_R_vec[8];
    }
    p_pre_->Lidar_wrt_Body_RT(Initial_Scan_Lidar_wrt_Body_T,
                              Initial_Scan_Lidar_wrt_Body_R);
    ROS_INFO_STREAM("[wxx][initial align] scan extrinsic source="
                    << (scan_extrinsic_from_mapping ? "mapping" : "wxx"));

    nh.param<double>("wxx/initial_align/voxelgrid_filter_size", param_voxelgrid_filter_size_, 0.5);
    nh.param<int>("wxx/initial_align/max_iteration", param_max_iterations_, 4);
    nh.param<int>("wxx/initial_align/icp_mode", param_function_value_, 1);
    nh.param<double>("wxx/initial_align/initial_map_size", param_map_size_, 50);
    nh.param<int>("wxx/initial_align/param_accumulated_scan", param_accumulated_scan_, 10);
    nh.param<double>("wxx/initial_align/max_fitness_score", max_fitness_score_, 2.0);

    nh.param<bool>("zty/advanced_by_scan_context", advanced_by_sc_, true);
    nh.param<int>("zty/scancontext/test_PC_NUM_RING", test_PC_NUM_RING, 20);
    nh.param<int>("zty/scancontext/test_PC_NUM_SECTOR", test_PC_NUM_SECTOR, 60);
    nh.param<double>("zty/scancontext/test_PC_MAX_RADIUS", test_PC_MAX_RADIUS, 20);
    nh.param<bool>("zty/scancontext/print_detail_score", print_detail_score_, false);

    nh.param<double>("zty/init_zone_width", init_zone_width, 10.0);
    nh.param<double>("zty/init_zone_height", init_zone_height, 10.0);
    nh.param<double>("zty/init_resolution", init_resolution, 1.0);

    nh.param<string>("wxx/initial_align/init_mode", init_mode_, "pose");
    nh.param<string>("wxx/initial_align/scdb_path", scdb_path_, "");
    nh.param<int>("wxx/initial_align/scdb_top_k", scdb_top_k_, 5);
    nh.param<double>("wxx/initial_align/scdb_accept_fitness", scdb_accept_fitness_, 0.5);
    nh.param<bool>("wxx/initial_align/scdb_pose_fallback", scdb_pose_fallback_, true);

    std::cout << "\033[1;32m[wxx][initial align] initial_map_pcd_name: " << initial_map_pcd_name_ << "\033[0m" << std::endl;
    std::cout << "\033[1;32m[wxx][initial align] Init_Body_pos: " << Init_Body_pos_.transpose() << "\033[0m" << std::endl;
    std::cout << "\033[1;32m[wxx][initial align] Init_Body_yaw_deg: " << init_body_yaw_deg_ << "\033[0m" << std::endl;

    if( advanced_by_sc_ )
    {
        std::cout << "\033[1;32m[wxx][initial align] Scan Context: \033[0m" << std::endl;
        std::cout << "[wxx][initial align] PC_NUM_RING: " << test_PC_NUM_RING << std::endl;
        std::cout << "[wxx][initial align] PC_NUM_SECTOR: " << test_PC_NUM_SECTOR << std::endl;
        std::cout << "[wxx][initial align] PC_MAX_RADIUS: " << test_PC_MAX_RADIUS << std::endl;
        std::cout << "[wxx][initial align] init_zone_width: " << init_zone_width << std::endl;
        std::cout << "[wxx][initial align] init_zone_height: " << init_zone_height << std::endl;
        std::cout << "[wxx][initial align] init_resolution: " << init_resolution << std::endl;
    }

    std::cout << "\033[1;32m[wxx][initial align] GICP: \033[0m" << std::endl;
    std::cout << "[wxx][initial align] max icp iterations: " << param_max_iterations_ << std::endl;
    switch(param_function_value_)
    {
        case 1:{
            // std::cout << "\033[1;32m[wxx][initial align] icp mode: " << "--- pcl_gicp ---" << "\033[0m" << std::endl;
            std::cout << "[wxx][initial align] icp mode: " << "--- pcl_gicp ---" << std::endl;
            break;
        }
        case 2:{
            // std::cout << "\033[1;32m[wxx][initial align] icp mode: " << "--- pcl_ndt ---" << "\033[0m" << std::endl;
            std::cout << "[wxx][initial align] icp mode: " << "--- pcl_ndt ---" << std::endl;
            break;
        }
        case 3:{
            // std::cout << "\033[1;32m[wxx][initial align] icp mode: " << "--- fgicp_st ---" << "\033[0m" << std::endl;
            std::cout << "[wxx][initial align] icp mode: " << "--- fgicp_st ---" << std::endl;
            break;
        }
        case 4:{
            // std::cout << "\033[1;32m[wxx][initial align] icp mode: " << "--- fgicp_mt ---" << "\033[0m" << std::endl;
            std::cout << "[wxx][initial align] icp mode: " << "--- fgicp_mt ---" << std::endl;
            break;
            }
        case 5:{
            // std::cout << "\033[1;32m[wxx][initial align] icp mode: " << "--- vgicp_st ---" << "\033[0m" << std::endl;
            std::cout << "[wxx][initial align] icp mode: " << "--- vgicp_st ---" << std::endl;
            break;
        }
    
        default:{
            std::cout << "\033[1;32m[wxx][initial align] icp mode: " << "--- invalid function ---" << "\033[0m" << std::endl;
            ROS_ERROR("[wxx][initial align] icp mode INVALID !!!");
            ROS_ERROR("[wxx][initial align] icp mode INVALID !!!");
            ROS_ERROR("[wxx][initial align] icp mode INVALID !!!");
            exit(0);
            break;
        }
    }

    pcl::PCDReader reader;
    // string file_name_map_kdtree = initial_map_pcd_name_;
    PointCloudXYZI::Ptr cloud_map(new PointCloudXYZI());
    const string all_points_dir_map_kdtree = initial_map_pcd_name_.size() > 0 && initial_map_pcd_name_[0] == '/' ?
        initial_map_pcd_name_ : string(string(ROOT_DIR) + "PCD/") + initial_map_pcd_name_;
    if( reader.read(all_points_dir_map_kdtree, *cloud_map) == -1 )
    {
        std::cout << "\033[1;31m[wxx][initial align] The initial pcd file [" << initial_map_pcd_name_ << "] does not exist!!!" << "\033[0m" << std::endl;
        std::cout << "\033[1;31m[wxx][initial align] Align Initialization Fail!!!" << "\033[0m" << std::endl;

        ROS_ERROR("[wxx][initial align] initial pcd file  does not exist!!!");
        ROS_ERROR("[wxx][initial align] initial pcd file  does not exist!!!");
        ROS_ERROR("[wxx][initial align] initial pcd file  does not exist!!!");
        return 1;
    }
    for (const auto& pt : cloud_map->points) {
        pcl::PointXYZ xyz_point;
        Eigen::Vector3d tp;
        xyz_point.x = pt.x;
        xyz_point.y = pt.y;
        xyz_point.z = pt.z;
        PCL_to_Eigen(xyz_point, tp);
        full_map_cloud_->points.push_back(xyz_point);
        // auto mode searches the whole map; pose mode keeps the legacy
        // crop around the configured initial pose.
        if( init_mode_ == "auto" || (tp - Init_Body_pos_).norm() < param_map_size_ )
            target_cloud_->points.push_back(xyz_point);
    }
    if (target_cloud_->empty())
    {
        ROS_ERROR_STREAM("[wxx][initial align] No map points remain within " << param_map_size_
                         << " m of Init_Body_pos " << Init_Body_pos_.transpose());
        write_init_result(false, "empty map crop");
        return 1;
    }

    /*** ROS subscribe initialization ***/
    ros::Subscriber sub_pcl = p_pre_->lidar_type == AVIA ? \
        nh.subscribe(lid_topic, 200000, livox_pcl_cbk) : \
        nh.subscribe(lid_topic, 200000, standard_pcl_cbk);
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);
    ros::Publisher pubOdomAftInit = nh.advertise<nav_msgs::Odometry> 
        ("/initial_odom_for_lio", 100000);
    // 两种输出用来debug，确认从icp里拿出来的odom是正确的
    ros::Publisher pubLaserCloudOdom = nh.advertise<sensor_msgs::PointCloud2>
            ("/initial_cloud_from_odom", 100000);
    ros::Publisher pubLaserCloudICP  = nh.advertise<sensor_msgs::PointCloud2>
            ("/initial_cloud_from_icp", 100000);
    ros::Publisher pubLaserCloudSC  = nh.advertise<sensor_msgs::PointCloud2>
            ("/initial_cloud_from_sc", 100000);    
    ros::Publisher pubLaserCloudTarget  = nh.advertise<sensor_msgs::PointCloud2>
            ("/target_cloud", 100000);
    ros::Publisher pubLaserCloudRaw  = nh.advertise<sensor_msgs::PointCloud2>
            ("/initial_cloud_raw", 100000);     
    ros::Publisher pubLaserCloudWithg  = nh.advertise<sensor_msgs::PointCloud2>
            ("/initial_cloud_with_g", 100000); 
//------------------------------------------------------------------------------------------------------
    ros::Rate rate(2000);
    std::cout << "\033[1;33m[wxx][initial align] Start!!! " << "\033[0m" << std::endl;

    if (init_mode_ == "auto") {
        if (scdb_path_.empty())
            scdb_path_ = initial_map_pcd_name_ + ".scdb";
        if (!load_scdb(scdb_path_)) {
            ROS_ERROR_STREAM("[wxx][initial align] auto mode requires a valid scdb: " << scdb_path_);
            if (!scdb_pose_fallback_) {
                write_init_result(false, "scdb load failed");
                return 1;
            }
            ROS_WARN("[wxx][initial align] no scdb; only configured-pose fallback available");
        }
    }

    bool alignment_succeeded = false;
    while (ros::ok())
    {
        ros::spinOnce();

        if(receive_source_ && imu_init_ready_)
        {
            // downsampling
            pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
            voxelgrid.setLeafSize(param_voxelgrid_filter_size_, param_voxelgrid_filter_size_, param_voxelgrid_filter_size_);

            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
            voxelgrid.setInputCloud(target_cloud_);
            voxelgrid.filter(*filtered);
            target_cloud_ = filtered;

            filtered.reset(new pcl::PointCloud<pcl::PointXYZ>());
            voxelgrid.setInputCloud(source_cloud_);
            voxelgrid.filter(*filtered);
            source_cloud_ = filtered;
            *source_cloud_raw_ = *source_cloud_;
            source_cloud_match_->clear();
            const double match_radius = test_PC_MAX_RADIUS + 2.0;
            const double match_radius_sq = match_radius * match_radius;
            for (const auto& p : source_cloud_->points) {
                if (p.x * p.x + p.y * p.y <= match_radius_sq)
                    source_cloud_match_->points.push_back(p);
            }
            source_cloud_match_->width = source_cloud_match_->points.size();
            source_cloud_match_->height = 1;
            source_cloud_match_->is_dense = true;
            std::cout << "target: " << target_cloud_->size() << "[pts]" << std::endl;
            std::cout << "source: " << source_cloud_->size() << "[pts]" << std::endl; 
            std::cout << "source_match: " << source_cloud_match_->size()
                      << "[pts] radius=" << match_radius << "m" << std::endl;

            // 1. init pose with gravity
            RT_to_M4(Init_Body_pos_, Init_Body_rot_by_align_gravity_, result_pose_);
            translate_cloud(source_cloud_, result_pose_);
            translate_cloud(source_cloud_match_, result_pose_);
            *source_cloud_with_g_ = *source_cloud_;

            if (init_mode_ == "auto") {
                // two independent proposal mechanisms, one unified verdict
                std::vector<AlignResult> pool;
                auto_align_by_scdb(pool);
                if (scdb_pose_fallback_)
                    local_sweep_align(pool);
                alignment_succeeded = finalize_alignment(pool);
            } else {
                // 2. coarse align by scan context (legacy zone search)
                if(advanced_by_sc_)
                    coarse_init_pose(target_cloud_, source_cloud_);

                // 3. fine align by ICP
                alignment_succeeded = align_from_pose();
            }
            break;
        }
    }

    // Keep GICP's position and yaw, but restore gravity roll/pitch for the
    // initial attitude. The GICP solution can tilt away from gravity (map
    // tilt / local minima); FAST-LIO is IMU-centric and must start
    // attitude-consistent with gravity, otherwise the filter fights between
    // IMU gravity and scan matching and the state diverges (NX field issue).
    {
        const Eigen::Matrix3d R = result_pose_.block<3,3>(0,0).cast<double>();
        const double yaw = std::atan2(R(1,0), R(0,0));
        const Eigen::Matrix3d R_g =
            Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix()
            * Init_Body_rot_gravity_;
        result_pose_.block<3,3>(0,0) = R_g.cast<float>();
        ROS_INFO_STREAM("[wxx][initial align] attitude gravity-restored (yaw kept): z="
                        << result_pose_(2,3));
    }

    if (!alignment_succeeded)
    {
        ROS_ERROR("[wxx][initial align] Alignment failed; no initial odometry will be published.");
        write_init_result(false, "alignment rejected");
        return 1;
    }
    write_init_result(true, "");

    // pub result
    nav_msgs::Odometry odomAftInit;
    odomAftInit.header.stamp = ros::Time::now();
    odomAftInit.header.frame_id = "world";
    odomAftInit.child_frame_id = "body";
    Eigen::Quaterniond quaternion(result_pose_.block<3,3>(0,0).cast<double>());
    odomAftInit.pose.pose.orientation.x = quaternion.x();
    odomAftInit.pose.pose.orientation.y = quaternion.y();
    odomAftInit.pose.pose.orientation.z = quaternion.z();
    odomAftInit.pose.pose.orientation.w = quaternion.w();
    odomAftInit.pose.pose.position.x = result_pose_.block<3,1>(0,3)(0);
    odomAftInit.pose.pose.position.y = result_pose_.block<3,1>(0,3)(1);
    odomAftInit.pose.pose.position.z = result_pose_.block<3,1>(0,3)(2);
    Eigen::Vector3d result_pos = result_pose_.block<3,1>(0,3).cast<double>();
    Eigen::Matrix3d result_rot = result_pose_.block<3,3>(0,0).cast<double>();
    ros::Rate rate_pub_odom(1);
    for(int i=0;i<100;i++)
    {
        odomAftInit.header.stamp = ros::Time::now();
        pubOdomAftInit.publish(odomAftInit);
        
        publish_frame(pubLaserCloudTarget, target_cloud_);
        publish_frame(pubLaserCloudRaw, source_cloud_raw_);

        publish_frame(pubLaserCloudWithg, source_cloud_with_g_);
        if(advanced_by_sc_)
            publish_frame(pubLaserCloudSC, source_cloud_sc_);
        publish_frame(pubLaserCloudICP, source_cloud_);

        publish_frame(pubLaserCloudOdom, source_cloud_raw_, result_pos, result_rot);

        rate_pub_odom.sleep();
    }

    return 0;
}
