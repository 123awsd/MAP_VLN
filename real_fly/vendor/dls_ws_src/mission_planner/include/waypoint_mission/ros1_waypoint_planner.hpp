#ifndef MISSION_PLANNER_WAYPOINT_PLANNER
#define MISSION_PLANNER_WAYPOINT_PLANNER

#include "ros/ros.h"
#include "vector"
#include "string"
#include "config.hpp"
#include "nav_msgs/Path.h"
#include "visualization_msgs/MarkerArray.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "mavros_msgs/RCIn.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "quadrotor_msgs/TakeoffLand.h"
#include "std_msgs/Bool.h"
#include "Eigen/Core"
#include <mutex>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <iomanip>
#include <sstream>
#include "utils/eigen_alias.hpp"
#include "utils/color_msg_utils.hpp"
#include "waypoint_mission/quintic_gate_trajectory.hpp"

namespace mission_planner {
    using namespace std;
    using namespace super_utils;

    class WaypointPlanner {

    private:
        MissionConfig cfg_;

        ros::NodeHandle nh_;

        Eigen::Vector3d cur_position;
        Eigen::Vector3d cur_velocity{Eigen::Vector3d::Zero()};
        int waypoint_counter{0};
        bool had_odom{false};
        bool triggered{false};
        bool new_goal{true};
        double odom_rcv_time{0};
        double cur_yaw{0};
        double close_since{-1};
        ros::Publisher goal_pub_, path_pub_, mkr_pub_, land_pub_, gate_cmd_pub_, gate_select_pub_,
                       super_pause_pub_, gate_path_pub_, gate_marker_pub_;
        ros::Subscriber click_sub_, mavros_sub_, odom_sub_, control_trigger_sub_, position_cmd_sub_,
                        super_position_cmd_sub_, gate_collision_cloud_sub_;
        ros::Timer goal_pub_timer_;
        double system_start_time{0};
        bool trigger_once{false};
        bool landing_pending{false};
        double last_position_cmd_time{0};
        double last_goal_pub_time{0};
        quadrotor_msgs::PositionCommand last_super_cmd_;
        double last_super_cmd_time_{0};
        bool had_super_cmd_{false};
        QuinticGateTrajectory gate_trajectory_;
        QuinticGateTrajectory gate_preview_trajectory_;
        enum class GateState { IDLE, ACTIVE, HANDOFF, FINAL_DWELL };
        GateState gate_state_{GateState::IDLE};
        double gate_start_time_{0};
        double gate_handoff_start_time_{0};
        double gate_handoff_goal_time_{0};
        double gate_start_ready_since_{-1};
        double last_gate_start_attempt_time_{-1};
        bool gate_handoff_timeout_reported_{false};
        pcl::PointCloud<pcl::PointXYZ>::Ptr gate_collision_cloud_{new pcl::PointCloud<pcl::PointXYZ>()};
        double gate_collision_cloud_time_{0};
        double last_gate_preview_time_{0};
        std::mutex gate_cloud_mutex_;
        enum class GateCollisionStatus { UNKNOWN, SAFE, COLLISION };


        void OdomCallback(const nav_msgs::OdometryConstPtr &msg) {
            had_odom = true;
            odom_rcv_time = ros::Time::now().toSec();
            cur_position = Eigen::Vector3d(msg->pose.pose.position.x,
                                           msg->pose.pose.position.y,
                                           msg->pose.pose.position.z);
            cur_velocity = Eigen::Vector3d(msg->twist.twist.linear.x,
                                           msg->twist.twist.linear.y,
                                           msg->twist.twist.linear.z);
            const auto &q = msg->pose.pose.orientation;
            cur_yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        }

        bool CloseToPoint(Vec3f &position) {
            return (position - cur_position).norm() < cfg_.switch_dis;
        }

        bool CloseToPoint(Vec3f &position, int id) {
            return (position - cur_position).norm() < cfg_.switch_dis_vec[id];
        }

        bool ReadyToSwitchWaypoint(const int id, const double cur_t) {
            if (!CloseToPoint(cfg_.waypoints[id], id)) {
                close_since = -1;
                return false;
            }

            const double target_yaw = cfg_.waypoint_yaws[id];
            if (!std::isnan(target_yaw)) {
                const double yaw_error = std::abs(std::atan2(std::sin(cur_yaw - target_yaw),
                                                             std::cos(cur_yaw - target_yaw)));
                if (yaw_error > cfg_.yaw_tolerances[id]) {
                    close_since = -1;
                    return false;
                }
            }

            if (cfg_.dwell_times[id] <= 0.0) {
                return true;
            }
            if (close_since < 0.0) {
                close_since = cur_t;
                return false;
            }
            return cur_t - close_since >= cfg_.dwell_times[id];
        }

        void RvizClickCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
            if (!had_odom) {
                return;
            }
            triggered = true;
            new_goal = true;
            waypoint_counter = 0;
            cout << YELLOW <<
                 " -- [MISSION] Rviz triggered." << RESET << endl;
        }

        void MavrosRcCallback(const mavros_msgs::RCInConstPtr &msg) {
            static int last_ch_10 = 1000;
            if (!had_odom) {
                return;
            }
            int ch_10 = msg->channels[9];
            bool pitch_up = msg->channels[1] < 1200;
            if (last_ch_10 > 1500 && ch_10 < 1500) {
                triggered = true;
                new_goal = true;
                waypoint_counter = 0;
                cout << YELLOW << " -- [MISSION] Mavros triggered." << RESET << endl;
            }
            last_ch_10 = ch_10;
        }

        void ControllerReadyCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
            (void) msg;
            if (triggered || landing_pending) {
                return;
            }
            triggered = true;
            new_goal = true;
            waypoint_counter = 0;
            close_since = -1;
            cout << GREEN << " -- [MISSION] Controller ready, start preset mission." << RESET << endl;
        }

        void PositionCommandCallback(const quadrotor_msgs::PositionCommandConstPtr &msg) {
            (void) msg;
            last_position_cmd_time = ros::Time::now().toSec();
        }

        void SuperPositionCommandCallback(const quadrotor_msgs::PositionCommandConstPtr &msg) {
            last_super_cmd_ = *msg;
            last_super_cmd_time_ = ros::Time::now().toSec();
            had_super_cmd_ = true;
        }

        void GateCollisionCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
            if (msg->header.frame_id != "world") {
                ROS_ERROR_THROTTLE(1.0, "[MISSION] Ignore D-gate collision cloud in frame '%s'; expected 'world'",
                                   msg->header.frame_id.c_str());
                return;
            }
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromROSMsg(*msg, *cloud);
            {
                std::lock_guard<std::mutex> lock(gate_cloud_mutex_);
                gate_collision_cloud_ = cloud;
                gate_collision_cloud_time_ = ros::Time::now().toSec();
            }

            const double now = ros::Time::now().toSec();
            if (cfg_.d_gate_mode == "smooth" && gate_state_ == GateState::IDLE &&
                now - last_gate_preview_time_ > 1.0) {
                double minimum_distance = std::numeric_limits<double>::infinity();
                const auto status = CheckGateCollision(gate_preview_trajectory_, now, minimum_distance);
                PublishGateVisualization(gate_preview_trajectory_, status, minimum_distance);
                last_gate_preview_time_ = now;
            }
        }

        void TryPublishLanding(const double cur_t) {
            if (!landing_pending) {
                return;
            }
            // Never request LAND unless the controller actually received at least one
            // trajectory command during this mission.
            if (last_position_cmd_time <= 0) {
                return;
            }
            if (cur_t - last_position_cmd_time <= cfg_.land_after_cmd_silence) {
                return;
            }

            quadrotor_msgs::TakeoffLand land;
            land.takeoff_land_cmd = quadrotor_msgs::TakeoffLand::LAND;
            land_pub_.publish(land);
            landing_pending = false;
            cout << GREEN << " -- [MISSION] SUPER command stream is quiet; request PX4Ctrl landing."
                 << RESET << endl;
        }

        void PublishGateSelection(const bool active) {
            std_msgs::Bool selection;
            selection.data = active;
            gate_select_pub_.publish(selection);
        }

        void PublishSuperPause(const bool paused) {
            std_msgs::Bool pause;
            pause.data = paused;
            super_pause_pub_.publish(pause);
        }

        void PublishWaypointGoal(const int index, const double cur_t) {
            geometry_msgs::PoseStamped goal;
            goal.pose.position.x = cfg_.waypoints[index].x();
            goal.pose.position.y = cfg_.waypoints[index].y();
            goal.pose.position.z = cfg_.waypoints[index].z();
            const double target_yaw = cfg_.waypoint_yaws[index];
            if (std::isnan(target_yaw)) {
                goal.pose.orientation.x = std::numeric_limits<double>::quiet_NaN();
                goal.pose.orientation.y = std::numeric_limits<double>::quiet_NaN();
                goal.pose.orientation.z = std::numeric_limits<double>::quiet_NaN();
                goal.pose.orientation.w = std::numeric_limits<double>::quiet_NaN();
            } else {
                goal.pose.orientation.z = std::sin(target_yaw / 2.0);
                goal.pose.orientation.w = std::cos(target_yaw / 2.0);
            }
            goal.header.frame_id = "world";
            goal.header.stamp = ros::Time::now();
            last_goal_pub_time = cur_t;
            goal_pub_.publish(goal);

            std::ostringstream goal_log;
            goal_log << std::fixed << std::setprecision(2)
                     << " -- [MISSION] Publish waypoint " << index + 1 << "/"
                     << cfg_.waypoints.size() << ": position=["
                     << cfg_.waypoints[index].x() << " "
                     << cfg_.waypoints[index].y() << " "
                     << cfg_.waypoints[index].z() << "], yaw=";
            if (std::isnan(target_yaw)) {
                goal_log << "auto";
            } else {
                goal_log << std::setprecision(1) << target_yaw * 180.0 / M_PI << " deg";
            }
            goal_log << std::setprecision(2) << ", distance="
                     << (cfg_.waypoints[index] - cur_position).norm() << " m.";
            cout << YELLOW << goal_log.str() << RESET << endl;
        }

        bool ReadyToStartSmoothGate(const double cur_t) {
            const Eigen::Vector3d target = cfg_.waypoints[cfg_.d_gate_start_waypoint];
            const double yaw_error = std::abs(std::atan2(std::sin(cur_yaw - cfg_.d_gate_yaw),
                                                         std::cos(cur_yaw - cfg_.d_gate_yaw)));
            const bool ready = (target - cur_position).norm() <= cfg_.d_gate_start_tolerance &&
                               cur_velocity.norm() <= cfg_.d_gate_start_max_speed &&
                               (!cfg_.d_gate_start_require_yaw ||
                                yaw_error <= cfg_.yaw_tolerances[cfg_.d_gate_start_waypoint]);
            if (!ready) {
                gate_start_ready_since_ = -1.0;
                return false;
            }
            if (gate_start_ready_since_ < 0.0) {
                gate_start_ready_since_ = cur_t;
                return cfg_.d_gate_start_dwell <= 0.0;
            }
            return cur_t - gate_start_ready_since_ >= cfg_.d_gate_start_dwell;
        }

        quadrotor_msgs::PositionCommand MakeGateCommand(const QuinticGateTrajectory::Sample& sample,
                                                        const double cur_t) const {
            quadrotor_msgs::PositionCommand cmd;
            cmd.header.stamp = ros::Time(cur_t);
            cmd.header.frame_id = "world";
            cmd.position.x = sample.position.x();
            cmd.position.y = sample.position.y();
            cmd.position.z = sample.position.z();
            cmd.velocity.x = sample.velocity.x();
            cmd.velocity.y = sample.velocity.y();
            cmd.velocity.z = sample.velocity.z();
            cmd.acceleration.x = sample.acceleration.x();
            cmd.acceleration.y = sample.acceleration.y();
            cmd.acceleration.z = sample.acceleration.z();
            cmd.jerk.x = sample.jerk.x();
            cmd.jerk.y = sample.jerk.y();
            cmd.jerk.z = sample.jerk.z();
            cmd.yaw = cfg_.d_gate_yaw;
            cmd.yaw_dot = 0.0;
            cmd.vel_norm = sample.velocity.norm();
            cmd.acc_norm = sample.acceleration.norm();
            cmd.trajectory_id = 900001;
            cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
            return cmd;
        }

        GateCollisionStatus CheckGateCollision(const QuinticGateTrajectory& trajectory,
                                               const double cur_t,
                                               double& minimum_distance) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
            double cloud_time = 0.0;
            {
                std::lock_guard<std::mutex> lock(gate_cloud_mutex_);
                cloud = gate_collision_cloud_;
                cloud_time = gate_collision_cloud_time_;
            }
            minimum_distance = std::numeric_limits<double>::infinity();
            if (!cloud || static_cast<int>(cloud->size()) < cfg_.d_gate_collision_min_points ||
                cur_t - cloud_time > cfg_.d_gate_collision_cloud_timeout) {
                return GateCollisionStatus::UNKNOWN;
            }

            pcl::KdTreeFLANN<pcl::PointXYZ> tree;
            tree.setInputCloud(cloud);
            std::vector<int> indices(1);
            std::vector<float> squared_distances(1);
            const double clearance_squared = cfg_.d_gate_collision_clearance *
                                             cfg_.d_gate_collision_clearance;
            const int sample_count = std::max(
                1, static_cast<int>(std::ceil(trajectory.duration() / cfg_.d_gate_sample_dt)));
            for (int i = 0; i <= sample_count; ++i) {
                const double t = trajectory.duration() * static_cast<double>(i) / sample_count;
                const auto sample = trajectory.sample(t);
                pcl::PointXYZ query;
                query.x = static_cast<float>(sample.position.x());
                query.y = static_cast<float>(sample.position.y());
                query.z = static_cast<float>(sample.position.z());
                if (tree.nearestKSearch(query, 1, indices, squared_distances) > 0) {
                    minimum_distance = std::min(minimum_distance,
                                                std::sqrt(static_cast<double>(squared_distances[0])));
                    if (squared_distances[0] < clearance_squared) {
                        return GateCollisionStatus::COLLISION;
                    }
                }
            }
            if (!std::isfinite(minimum_distance) ||
                minimum_distance > cfg_.d_gate_collision_coverage_max_distance) {
                // A distant cloud does not prove that the gate itself was observed.
                return GateCollisionStatus::UNKNOWN;
            }
            return GateCollisionStatus::SAFE;
        }

        void PublishGateVisualization(const QuinticGateTrajectory& trajectory,
                                      const GateCollisionStatus status,
                                      const double minimum_distance) {
            nav_msgs::Path path;
            path.header.frame_id = "world";
            path.header.stamp = ros::Time::now();

            visualization_msgs::Marker line;
            line.header = path.header;
            line.ns = "d_gate_smooth_centerline";
            line.id = 0;
            line.type = visualization_msgs::Marker::LINE_STRIP;
            line.action = visualization_msgs::Marker::ADD;
            line.pose.orientation.w = 1.0;
            line.scale.x = 0.04;
            line.color.a = 1.0;

            visualization_msgs::Marker envelope;
            envelope.header = path.header;
            envelope.ns = "d_gate_safety_envelope";
            envelope.id = 1;
            envelope.type = visualization_msgs::Marker::SPHERE_LIST;
            envelope.action = visualization_msgs::Marker::ADD;
            envelope.pose.orientation.w = 1.0;
            envelope.scale.x = 2.0 * cfg_.d_gate_collision_clearance;
            envelope.scale.y = envelope.scale.x;
            envelope.scale.z = envelope.scale.x;
            envelope.color.a = 0.16;

            if (status == GateCollisionStatus::SAFE) {
                line.color.g = 1.0;
                envelope.color.g = 1.0;
            } else if (status == GateCollisionStatus::COLLISION) {
                line.color.r = 1.0;
                envelope.color.r = 1.0;
            } else {
                line.color.r = 1.0;
                line.color.g = 0.75;
                envelope.color.r = 1.0;
                envelope.color.g = 0.75;
            }

            const double envelope_dt = std::max(0.12, cfg_.d_gate_sample_dt);
            double next_envelope_t = 0.0;
            const int sample_count = std::max(
                1, static_cast<int>(std::ceil(trajectory.duration() / cfg_.d_gate_sample_dt)));
            for (int i = 0; i <= sample_count; ++i) {
                const double t = trajectory.duration() * static_cast<double>(i) / sample_count;
                const auto sample = trajectory.sample(t);
                geometry_msgs::PoseStamped pose;
                pose.header = path.header;
                pose.pose.position.x = sample.position.x();
                pose.pose.position.y = sample.position.y();
                pose.pose.position.z = sample.position.z();
                pose.pose.orientation.z = std::sin(cfg_.d_gate_yaw / 2.0);
                pose.pose.orientation.w = std::cos(cfg_.d_gate_yaw / 2.0);
                path.poses.push_back(pose);
                line.points.push_back(pose.pose.position);
                if (t + 1.0e-6 >= next_envelope_t) {
                    envelope.points.push_back(pose.pose.position);
                    next_envelope_t += envelope_dt;
                }
            }

            visualization_msgs::Marker status_text;
            status_text.header = path.header;
            status_text.ns = "d_gate_smooth_status";
            status_text.id = 2;
            status_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            status_text.action = visualization_msgs::Marker::ADD;
            status_text.pose.orientation.w = 1.0;
            status_text.scale.z = 0.25;
            status_text.color = line.color;
            if (!path.poses.empty()) {
                status_text.pose.position = path.poses.front().pose.position;
                status_text.pose.position.z += 0.45;
            }
            std::ostringstream text;
            if (status == GateCollisionStatus::SAFE) {
                text << "D smooth: SAFE, min=" << std::fixed << std::setprecision(2)
                     << minimum_distance << " m";
            } else if (status == GateCollisionStatus::COLLISION) {
                text << "D smooth: COLLISION, min<" << std::fixed << std::setprecision(2)
                     << cfg_.d_gate_collision_clearance << " m";
            } else {
                text << "D smooth: WAITING FOR GATE CLOUD";
            }
            status_text.text = text.str();

            visualization_msgs::MarkerArray markers;
            markers.markers = {line, envelope, status_text};
            gate_path_pub_.publish(path);
            gate_marker_pub_.publish(markers);
        }

        void BuildNominalGatePreview() {
            std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> guides;
            guides.reserve(cfg_.d_gate_guide_x.size());
            for (std::size_t i = 0; i < cfg_.d_gate_guide_x.size(); ++i) {
                guides.emplace_back(cfg_.d_gate_guide_x[i], cfg_.d_gate_guide_y[i],
                                    cfg_.d_gate_guide_z[i]);
            }
            gate_preview_trajectory_.build(cfg_.waypoints[cfg_.d_gate_start_waypoint],
                                           Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), guides,
                                           cfg_.d_gate_guide_speed, cfg_.d_gate_min_segment_time);
            PublishGateVisualization(gate_preview_trajectory_, GateCollisionStatus::UNKNOWN,
                                     std::numeric_limits<double>::infinity());
        }

        bool StartSmoothGate(const double cur_t) {
            Eigen::Vector3d start_position = cur_position;
            Eigen::Vector3d start_velocity = cur_velocity;
            Eigen::Vector3d start_acceleration = Eigen::Vector3d::Zero();
            if (had_super_cmd_ && cur_t - last_super_cmd_time_ < 0.2) {
                const Eigen::Vector3d command_position(last_super_cmd_.position.x,
                                                       last_super_cmd_.position.y,
                                                       last_super_cmd_.position.z);
                if ((command_position - cur_position).norm() < 0.3) {
                    start_position = command_position;
                    start_velocity = Eigen::Vector3d(last_super_cmd_.velocity.x,
                                                     last_super_cmd_.velocity.y,
                                                     last_super_cmd_.velocity.z);
                    start_acceleration = Eigen::Vector3d(last_super_cmd_.acceleration.x,
                                                         last_super_cmd_.acceleration.y,
                                                         last_super_cmd_.acceleration.z);
                }
            }

            std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> guides;
            guides.reserve(cfg_.d_gate_guide_x.size());
            for (std::size_t i = 0; i < cfg_.d_gate_guide_x.size(); ++i) {
                guides.emplace_back(cfg_.d_gate_guide_x[i], cfg_.d_gate_guide_y[i],
                                    cfg_.d_gate_guide_z[i]);
            }
            gate_trajectory_.build(start_position, start_velocity, start_acceleration, guides,
                                   cfg_.d_gate_guide_speed, cfg_.d_gate_min_segment_time);
            double minimum_distance = std::numeric_limits<double>::infinity();
            const auto collision_status = CheckGateCollision(gate_trajectory_, cur_t, minimum_distance);
            PublishGateVisualization(gate_trajectory_, collision_status, minimum_distance);
            if (collision_status == GateCollisionStatus::COLLISION ||
                (collision_status == GateCollisionStatus::UNKNOWN &&
                 cfg_.d_gate_collision_check_required)) {
                if (collision_status == GateCollisionStatus::COLLISION) {
                    ROS_ERROR_THROTTLE(1.0,
                                       "[MISSION] Block smooth D-gate start: swept clearance is below %.2f m",
                                       cfg_.d_gate_collision_clearance);
                } else {
                    ROS_ERROR_THROTTLE(1.0,
                                       "[MISSION] Block smooth D-gate start: gate cloud is missing, stale, or not observed near the route");
                }
                return false;
            }
            gate_start_time_ = cur_t;
            gate_state_ = GateState::ACTIVE;
            gate_handoff_timeout_reported_ = false;
            PublishSuperPause(true);
            PublishGateSelection(true);
            gate_cmd_pub_.publish(MakeGateCommand(gate_trajectory_.sample(0.0), cur_t));
            cout << GREEN << " -- [MISSION] Start smooth D-gate trajectory: duration="
                 << std::fixed << std::setprecision(2) << gate_trajectory_.duration()
                 << " s, speed=" << cfg_.d_gate_speed << " m/s, checked clearance="
                 << minimum_distance << " m." << RESET << endl;
            return true;
        }

        void UpdateSmoothGate(const double cur_t) {
            if (gate_state_ == GateState::ACTIVE) {
                const double elapsed = cur_t - gate_start_time_;
                gate_cmd_pub_.publish(MakeGateCommand(gate_trajectory_.sample(elapsed), cur_t));
                if (elapsed < gate_trajectory_.duration()) {
                    return;
                }

                if (cfg_.d_gate_finishes_mission) {
                    gate_state_ = GateState::FINAL_DWELL;
                    gate_handoff_start_time_ = cur_t;
                    cout << GREEN << " -- [MISSION] Smooth C-to-D trajectory reached the landing area; hold before mission completion."
                         << RESET << endl;
                    return;
                }

                gate_state_ = GateState::HANDOFF;
                gate_handoff_start_time_ = cur_t;
                waypoint_counter = cfg_.d_gate_end_waypoint + 1;
                close_since = -1.0;
                new_goal = false;
                PublishWaypointGoal(waypoint_counter, cur_t);
                gate_handoff_goal_time_ = cur_t;
                cout << GREEN << " -- [MISSION] Smooth D-gate trajectory completed; hold exit and prepare SUPER handoff."
                     << RESET << endl;
                return;
            }

            const auto endpoint = gate_trajectory_.sample(gate_trajectory_.duration());
            gate_cmd_pub_.publish(MakeGateCommand(endpoint, cur_t));
            if (gate_state_ == GateState::FINAL_DWELL) {
                if (cur_t - gate_handoff_start_time_ < cfg_.d_gate_final_dwell) {
                    return;
                }
                gate_state_ = GateState::IDLE;
                triggered = false;
                new_goal = false;
                PublishGateSelection(false);
                cout << GREEN << " -- [MISSION] Preset mission completed by smooth C-to-D trajectory."
                     << RESET << endl;
                if (cfg_.auto_land) {
                    landing_pending = true;
                    cout << YELLOW << " -- [MISSION] Smooth command stream stopped; prepare PX4Ctrl landing."
                         << RESET << endl;
                }
                return;
            }
            const Eigen::Vector3d super_position(last_super_cmd_.position.x,
                                                 last_super_cmd_.position.y,
                                                 last_super_cmd_.position.z);
            const bool fresh_super_command = had_super_cmd_ &&
                                             last_super_cmd_time_ > gate_handoff_goal_time_ &&
                                             cur_t - last_super_cmd_time_ < 0.15;
            const bool handoff_ready = fresh_super_command &&
                                       (super_position - cur_position).norm() < 0.4 &&
                                       cur_t - gate_handoff_start_time_ >= cfg_.d_gate_handoff_min_hold;
            if (handoff_ready) {
                PublishSuperPause(false);
                PublishGateSelection(false);
                gate_state_ = GateState::IDLE;
                gate_start_ready_since_ = -1.0;
                cout << GREEN << " -- [MISSION] Smooth D-gate handoff to SUPER succeeded at waypoint "
                     << waypoint_counter + 1 << "/" << cfg_.waypoints.size() << "."
                     << RESET << endl;
                return;
            }
            if (!gate_handoff_timeout_reported_ &&
                cur_t - gate_handoff_start_time_ > cfg_.d_gate_handoff_timeout) {
                gate_handoff_timeout_reported_ = true;
                ROS_ERROR("[MISSION] SUPER handoff timed out; keep holding the D-gate exit with the smooth command source");
            }
        }

        void GoalPubTimerCallback(const ros::TimerEvent &e) {
            static int last_mkr_sub_num = mkr_pub_.getNumSubscribers();
            int cur_mkr_sub_num = mkr_pub_.getNumSubscribers();
            if (cur_mkr_sub_num != last_mkr_sub_num && cur_mkr_sub_num > 0) {
                visualizeMission();
            }
            last_mkr_sub_num = cur_mkr_sub_num;
            const double cur_t = ros::Time::now().toSec();
            if (landing_pending) {
                TryPublishLanding(cur_t);
                return;
            }
            if (cfg_.start_trigger_type == 2 && ! trigger_once) {
                if (cur_t - system_start_time < cfg_.start_program_delay) {
                    return;
                } else {
                    triggered = true;
                    trigger_once = true;
                }
            }
            if (!triggered) {
                return;
            }

            if (cur_t - odom_rcv_time > cfg_.odom_timeout) {
                static double last_print_t = ros::Time::now().toSec();
                if (cur_t - last_print_t > 1.0) {
                    last_print_t = cur_t;
                    cout << YELLOW << " -- [MISSION] Odom Timeout!" << RESET << endl;
                }
                return;
            }

            if (gate_state_ != GateState::IDLE) {
                UpdateSmoothGate(cur_t);
                return;
            }

            const bool at_smooth_gate_start = cfg_.d_gate_mode == "smooth" &&
                                              waypoint_counter == cfg_.d_gate_start_waypoint;
            const bool waypoint_ready = at_smooth_gate_start
                                            ? ReadyToStartSmoothGate(cur_t)
                                            : ReadyToSwitchWaypoint(waypoint_counter, cur_t);
            if (waypoint_ready) {
                if (at_smooth_gate_start) {
                    if (last_gate_start_attempt_time_ >= 0.0 &&
                        cur_t - last_gate_start_attempt_time_ < 0.2) {
                        return;
                    }
                    last_gate_start_attempt_time_ = cur_t;
                    if (StartSmoothGate(cur_t)) {
                        cout << GREEN << " -- [MISSION] Reached D-gate staging waypoint "
                             << waypoint_counter + 1 << "/" << cfg_.waypoints.size() << "."
                             << RESET << endl;
                    }
                    return;
                }
                const int reached_waypoint = waypoint_counter;
                const int waypoint_count = static_cast<int>(cfg_.waypoints.size());
                if (reached_waypoint + 1 < waypoint_count) {
                    cout << GREEN << " -- [MISSION] Reached waypoint " << reached_waypoint + 1 << "/"
                         << waypoint_count << "; switch to " << reached_waypoint + 2 << "/"
                         << waypoint_count << "." << RESET << endl;
                } else {
                    cout << GREEN << " -- [MISSION] Reached final waypoint " << waypoint_count << "/"
                         << waypoint_count << "." << RESET << endl;
                }
                waypoint_counter++;
                close_since = -1;
                new_goal = true;
                if (waypoint_counter >= cfg_.waypoints.size()) {
                    // 结束，停止发布。
                    waypoint_counter = cfg_.waypoints.size() - 1;
                    triggered = false;
                    new_goal = false;
                    cout << GREEN << " -- [MISSION] Preset mission completed." << RESET << endl;
                    if (cfg_.auto_land) {
                        landing_pending = true;
                        cout << YELLOW << " -- [MISSION] Wait for SUPER command stream to stop before landing."
                             << RESET << endl;
                    }
                }
            }

            if (new_goal || cur_t - last_goal_pub_time > cfg_.publish_dt) {
                new_goal = false;
                PublishWaypointGoal(waypoint_counter, cur_t);
            }


        }

    public:
        WaypointPlanner() {};

        WaypointPlanner(const ros::NodeHandle &nh) {
            nh_ = nh;
#define CONFIG_FILE_DIR(name) (string(string(ROOT_DIR) + "config/"+name))
            std::string dft_cfg_path = CONFIG_FILE_DIR("waypoint.yaml");
            std::string cfg_path, cfg_name;
            if (nh.param("config_path", cfg_path, dft_cfg_path)) {
                cout << " -- [Fsm-Test] Load config from: " << cfg_path << endl;
            } else if (nh.param("config_name", cfg_name, dft_cfg_path)) {
                cfg_path = CONFIG_FILE_DIR(cfg_name);
                cout << " -- [Fsm-Test] Load config by file name: " << cfg_name << endl;
            }
#define DATA_FILE_DIR(name) (string(string(ROOT_DIR) + "data/"+name))
            std::string dft_data_path = DATA_FILE_DIR("benchmark.yaml");
            std::string data_path, data_name;
            if (nh.param("data_path", data_path, dft_data_path)) {
                cout << " -- [MissionPlanner] Load data from: " << data_path << endl;
            } else if (nh.param("data_name", data_name, dft_data_path)) {
                data_path = DATA_FILE_DIR(data_name);
                cout << " -- [MissionPlanner] Load data by file name: " << data_path << endl;
            }
            cfg_ = MissionConfig(cfg_path);

            cfg_.LoadWaypoint(data_path);
            nh_.param("d_gate_mode", cfg_.d_gate_mode, cfg_.d_gate_mode);
            cfg_.ValidateGateConfig();
            cout << GREEN << " -- [MISSION] D-gate mode: " << cfg_.d_gate_mode << RESET << endl;

            odom_sub_ = nh_.subscribe(cfg_.odom_topic, 10, &WaypointPlanner::OdomCallback, this);
            goal_pub_timer_ = nh_.createTimer(ros::Duration(0.01), &WaypointPlanner::GoalPubTimerCallback, this);
            goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(cfg_.goal_pub_topic, 10);
            path_pub_ = nh_.advertise<nav_msgs::Path>(cfg_.path_pub_topic, 10);
            gate_cmd_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>(cfg_.d_gate_cmd_topic, 10);
            gate_select_pub_ = nh_.advertise<std_msgs::Bool>(cfg_.d_gate_select_topic, 1, true);
            super_pause_pub_ = nh_.advertise<std_msgs::Bool>(cfg_.d_gate_super_pause_topic, 1, true);
            gate_path_pub_ = nh_.advertise<nav_msgs::Path>(cfg_.d_gate_path_topic, 1, true);
            gate_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(cfg_.d_gate_marker_topic, 1, true);
            super_position_cmd_sub_ = nh_.subscribe(cfg_.super_position_cmd_topic, 20,
                                                    &WaypointPlanner::SuperPositionCommandCallback, this);
            PublishGateSelection(false);
            PublishSuperPause(false);
            if (cfg_.d_gate_mode == "smooth") {
                BuildNominalGatePreview();
                gate_collision_cloud_sub_ = nh_.subscribe(cfg_.d_gate_collision_cloud_topic, 1,
                                                          &WaypointPlanner::GateCollisionCloudCallback, this);
            }
            if (cfg_.start_trigger_type == 0) {
                click_sub_ = nh_.subscribe("/goal", 10, &WaypointPlanner::RvizClickCallback, this);
            } else if (cfg_.start_trigger_type == 1) {
                mavros_sub_ = nh_.subscribe("/mavros/rc/in", 10, &WaypointPlanner::MavrosRcCallback, this);
            } else if (cfg_.start_trigger_type == 3) {
                control_trigger_sub_ = nh_.subscribe(cfg_.start_trigger_topic, 1,
                                                     &WaypointPlanner::ControllerReadyCallback, this);
            }
            position_cmd_sub_ = nh_.subscribe(cfg_.position_cmd_topic, 10,
                                              &WaypointPlanner::PositionCommandCallback, this);
            if (cfg_.auto_land) {
                land_pub_ = nh_.advertise<quadrotor_msgs::TakeoffLand>(cfg_.land_topic, 1);
            }
            mkr_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("mkr", 1);
            system_start_time = ros::Time::now().toSec();
        }

        void visualizeMission() {
            visualization_msgs::MarkerArray mkr_arr;
            addPathToMarkerArray(mkr_arr, cfg_.waypoints, Color::SteelBlue(), "waypoints", 0.5, 0.3);

            for (int i = 0; i < cfg_.switch_dis_vec.size(); i++) {
                Color c = Color::Orange();
                c.a = 0.2;
                visualizePoint(mkr_arr, cfg_.waypoints[i], c,
                               "switch_dis", cfg_.switch_dis_vec[i] * 2, i);
                visualizeText(mkr_arr, "id", to_string(1 + i), cfg_.waypoints[i],
                              Color::Black(), 3, i);
            }
            mkr_pub_.publish(mkr_arr);
        }

        static void visualizePoint(visualization_msgs::MarkerArray &mkr_arr,
                                   const Vec3f &pt,
                                   Color color = Color::Pink(),
                                   std::string ns = "pt",
                                   double size = 0.1, int id = -1,
                                   const bool &print_ns = true) {
            visualization_msgs::Marker marker_ball;
            static int cnt = 0;
            Vec3f cur_pos = pt;
            if (isnan(pt.x()) || isnan(pt.y()) || isnan(pt.z())) {
                return;
            }
            marker_ball.header.frame_id = "world";
            marker_ball.header.stamp = ros::Time::now();
            marker_ball.ns = ns.c_str();
            marker_ball.id = id >= 0 ? id : cnt++;
            marker_ball.action = visualization_msgs::Marker::ADD;
            marker_ball.pose.orientation.w = 1.0;
            marker_ball.type = visualization_msgs::Marker::SPHERE;
            marker_ball.scale.x = size;
            marker_ball.scale.y = size;
            marker_ball.scale.z = size;
            marker_ball.color = color;

            geometry_msgs::Point p;
            p.x = cur_pos.x();
            p.y = cur_pos.y();
            p.z = cur_pos.z();

            marker_ball.pose.position = p;
            mkr_arr.markers.push_back(marker_ball);

            // add test
            if (print_ns) {
                visualization_msgs::Marker marker;
                marker.header.frame_id = "world";
                marker.header.stamp = ros::Time::now();
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.orientation.w = 1.0;
                marker.ns = ns + "_text";
                if (id >= 0) {
                    marker.id = id;
                } else {
                    static int id = 0;
                    marker.id = id++;
                }
                marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                marker.scale.z = 0.6;
                marker.color = color;
                marker.text = ns;
                marker.pose.position.x = cur_pos.x();
                marker.pose.position.y = cur_pos.y();
                marker.pose.position.z = cur_pos.z() + 0.5;
                marker.pose.orientation.w = 1.0;
                mkr_arr.markers.push_back(marker);
            }
        }

        static void visualizeText(visualization_msgs::MarkerArray &mkr_arr,
                                  const std::string &ns,
                                  const std::string &text,
                                  const Vec3f &position,
                                  const Color &c = Color::White(),
                                  const double &size = 0.6,
                                  const int &id = -1) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time::now();
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.ns = ns.c_str();
            if (id >= 0) {
                marker.id = id;
            } else {
                static int id = 0;
                marker.id = id++;
            }
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.scale.z = size;
            marker.color = c;
            marker.text = text;
            marker.pose.position.x = position.x();
            marker.pose.position.y = position.y();
            marker.pose.position.z = position.z();
            marker.pose.orientation.w = 1.0;
            mkr_arr.markers.push_back(marker);
        };

        static void addPathToMarkerArray(visualization_msgs::MarkerArray &mkr_ary,
                                         const vec_E<Vec3f> &path,
                                         Color color,
                                         string ns,
                                         double pt_size,
                                         double line_size) {
            visualization_msgs::Marker line_list;
            if (path.size() <= 0) {
                std::cout << YELLOW << "Try to publish empty path, return.\n" << RESET << std::endl;
                return;
            }
            Vec3f cur_pt = path[0], last_pt;
            static int point_id = 0;
            static int line_cnt = 0;
            for (size_t i = 0; i < path.size(); i++) {
                last_pt = cur_pt;
                cur_pt = path[i];

                /* Publish point */
                visualization_msgs::Marker point;
                point.header.frame_id = "world";
                point.header.stamp = ros::Time::now();
                point.ns = ns.c_str();
                point.id = point_id++;
                point.action = visualization_msgs::Marker::ADD;
                point.pose.orientation.w = 1.0;
                point.type = visualization_msgs::Marker::SPHERE;
                // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
                point.scale.x = pt_size;
                point.scale.y = pt_size;
                point.scale.z = pt_size;
                // Line list is blue
                point.color = color;
                point.color.a = 1.0;
                // Create the vertices for the points and lines
                geometry_msgs::Point p;
                p.x = cur_pt.x();
                p.y = cur_pt.y();
                p.z = cur_pt.z();
                point.pose.position = p;
                mkr_ary.markers.push_back(point);
                /* publish lines */
                if (i > 0) {
                    geometry_msgs::Point p;
                    // publish lines
                    visualization_msgs::Marker line_list;
                    line_list.header.frame_id = "world";
                    line_list.header.stamp = ros::Time::now();
                    line_list.ns = ns + "_line";
                    line_list.id = line_cnt++;
                    line_list.action = visualization_msgs::Marker::ADD;
                    line_list.pose.orientation.w = 1.0;
                    line_list.type = visualization_msgs::Marker::LINE_LIST;
                    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
                    line_list.scale.x = line_size;
                    // Line list is blue
                    line_list.color = color;
                    // Create the vertices for the points and lines

                    p.x = last_pt.x();
                    p.y = last_pt.y();
                    p.z = last_pt.z();
                    // The line list needs two points for each line
                    line_list.points.push_back(p);
                    p.x = cur_pt.x();
                    p.y = cur_pt.y();
                    p.z = cur_pt.z();
                    // The line list needs
                    line_list.points.push_back(p);

                    mkr_ary.markers.push_back(line_list);
                }
            }
        }

    };
}
#endif //MISSION_PLANNER_WAYPOINT_PLANNER
