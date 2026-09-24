#include <algorithm>
#include <cstdio>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/MarkerArray.h>

#include <path_search/astar.h>
#include <ros_interface/ros1/ros1_interface.hpp>
#include <super_core/config.hpp>
#include <super_core/corridor_generator.h>
#include <traj_opt/exp_traj_optimizer_s4.h>
#include "waypoint_mission/minco_trajectory.hpp"
#include "waypoint_mission/smooth_route.hpp"

namespace mission_planner {

class FullSmoothMission {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FullSmoothMission() : private_nh_("~") {
        private_nh_.param("route_path", route_path_, std::string());
        private_nh_.param("known_map_pcd", known_map_pcd_, std::string());
        private_nh_.param("odom_topic", odom_topic_, std::string("/lidar_slam/odom"));
        private_nh_.param("command_topic", command_topic_, std::string("/planning/gate_pos_cmd"));
        private_nh_.param("select_topic", select_topic_, std::string("/planning/gate_active"));
        private_nh_.param("path_topic", path_topic_, std::string("/planning/full_smooth_path"));
        private_nh_.param("marker_topic", marker_topic_,
                          std::string("/planning/full_smooth_markers"));
        private_nh_.param("trigger_topic", trigger_topic_, std::string("/traj_start_trigger"));
        private_nh_.param("land_topic", land_topic_, std::string("/px4ctrl/takeoff_land"));
        private_nh_.param("start_trigger_type", start_trigger_type_, 2);
        private_nh_.param("start_delay", start_delay_, 5.0);
        private_nh_.param("auto_land", auto_land_, false);
        private_nh_.param("min_segment_time", min_segment_time_, 0.6);
        private_nh_.param("collision_clearance", collision_clearance_, 0.21);
        private_nh_.param("sample_dt", sample_dt_, 0.03);
        private_nh_.param("collision_validation_step", collision_validation_step_, 0.01);
        // max_velocity is intentionally sourced from super_config_path below.
        // Keep this read only to emit a clear warning for legacy launch files
        // that still try to override it locally.
        const bool has_legacy_max_velocity = private_nh_.hasParam("max_velocity");
        private_nh_.param("max_velocity", max_velocity_, 2.45);
        const double legacy_max_velocity = max_velocity_;
        private_nh_.param("max_acceleration", max_acceleration_, 6.0);
        private_nh_.param("max_jerk", max_jerk_, 120.0);
        private_nh_.param("max_snap", max_snap_, 500.0);
        private_nh_.param("max_jerk_discontinuity", max_jerk_discontinuity_, 1.0e-4);
        private_nh_.param("yaw_start_blend_duration", yaw_start_blend_duration_, 1.0);
        private_nh_.param("yaw_terminal_blend_duration", yaw_terminal_blend_duration_, 1.5);
        private_nh_.param("yaw_velocity_threshold", yaw_velocity_threshold_, 0.08);
        private_nh_.param("yaw_lookahead_time", yaw_lookahead_time_, 0.30);
        private_nh_.param("max_yaw_rate", max_yaw_rate_, 2.5);
        private_nh_.param("execution_max_velocity", execution_max_velocity_, 0.30);
        private_nh_.param("execution_max_yaw_rate", execution_max_yaw_rate_, 0.60);
        private_nh_.param("max_yaw_lock_variation", max_yaw_lock_variation_, 1.0);
        private_nh_.param("wait_for_reload", wait_for_reload_, false);
        private_nh_.param("preview_only", preview_only_, false);
        private_nh_.param("save_generated_trajectory", save_generated_trajectory_, false);
        private_nh_.param("execute_saved_trajectory", execute_saved_trajectory_, false);
        private_nh_.param("load_saved_trajectory_for_preview",
                          load_saved_trajectory_for_preview_, false);
        private_nh_.param("saved_trajectory_path", saved_trajectory_path_, std::string());
        if (saved_trajectory_path_.empty()) {
            saved_trajectory_path_ = workspaceOutputFile("recorded_full_smooth_trajectory.txt");
        }
        private_nh_.param("start_position_tolerance", start_position_tolerance_, 0.20);
        private_nh_.param("start_yaw_tolerance", start_yaw_tolerance_, M_PI / 4.0);
        private_nh_.param("preview_start_at_first_route_point",
                          preview_start_at_first_route_point_, false);
        private_nh_.param("recorded_waypoint_speed", recorded_waypoint_speed_, 1.2);
        private_nh_.param("use_super_safe_corridor", use_super_safe_corridor_, false);
        private_nh_.param("super_use_recorded_guide_without_astar",
                          super_use_recorded_guide_without_astar_, false);
        private_nh_.param("super_static_map_only", super_static_map_only_, true);
        private_nh_.param("super_config_path", super_config_path_,
                          std::string(ROOT_DIR) + "../super_planner/config/competition_2026.yaml");
        // A saved full-smooth polynomial is generated and executed at different
        // times, but both stages must use exactly one velocity authority.  The
        // real SUPER configuration already owns the vehicle velocity bound, so
        // do not duplicate it in either launch file.
        const super_planner::Config super_config(super_config_path_);
        const double configured_max_velocity = super_config.exp_traj_cfg.max_vel;
        if (!std::isfinite(configured_max_velocity) || configured_max_velocity <= 0.0) {
            throw std::runtime_error("traj_opt.boundary.max_vel must be positive in " +
                                     super_config_path_);
        }
        max_velocity_ = configured_max_velocity;
        if (has_legacy_max_velocity &&
            std::abs(legacy_max_velocity - max_velocity_) > 1.0e-6) {
            ROS_WARN("[FULL_SMOOTH] ignoring legacy ~max_velocity=%.3f; using %.3f m/s from %s traj_opt.boundary.max_vel",
                     legacy_max_velocity, max_velocity_, super_config_path_.c_str());
        }
        ROS_INFO("[FULL_SMOOTH] velocity limit %.3f m/s from %s traj_opt.boundary.max_vel",
                 max_velocity_, super_config_path_.c_str());
        private_nh_.param("super_astar_timeout", super_astar_timeout_, 5.0);
        private_nh_.param("super_corridor_extra_margin", super_corridor_extra_margin_, 0.0);
        private_nh_.param("narrow_corridor/enabled", narrow_corridor_enabled_, true);
        private_nh_.param("narrow_corridor/clearance_threshold",
                          narrow_corridor_clearance_threshold_, 0.40);
        private_nh_.param("narrow_corridor/max_width", narrow_corridor_max_width_, 1.00);
        private_nh_.param("narrow_corridor/transition_length",
                          narrow_corridor_transition_length_, 0.45);
        private_nh_.param("narrow_corridor/max_half_width",
                          narrow_corridor_max_half_width_, 0.18);
        private_nh_.param("narrow_corridor/min_half_width",
                          narrow_corridor_min_half_width_, 0.10);
        private_nh_.param("narrow_corridor/margin", narrow_corridor_margin_, 0.03);
        private_nh_.param("narrow_corridor/side_vertical_window",
                          narrow_corridor_side_vertical_window_, 0.35);
        private_nh_.param("narrow_corridor/side_longitudinal_window",
                          narrow_corridor_side_longitudinal_window_, 0.35);
        private_nh_.param("narrow_corridor/max_plane_violation",
                          narrow_corridor_max_plane_violation_, 0.005);
        private_nh_.param("vertical_guide_floor/enabled", vertical_guide_floor_enabled_, true);
        private_nh_.param("vertical_guide_floor/max_violation",
                          vertical_guide_floor_max_violation_, 0.002);
        private_nh_.param("guide_tracking/enabled", guide_tracking_enabled_, true);
        private_nh_.param("guide_tracking/horizontal_half_width",
                          guide_tracking_horizontal_half_width_, 0.12);
        private_nh_.param("guide_tracking/monotonic_vertical_band",
                          guide_tracking_monotonic_vertical_band_, 0.01);
        private_nh_.param("guide_tracking/max_plane_violation",
                          guide_tracking_max_plane_violation_, 0.005);
        private_nh_.param("local_collision_replan_enabled", local_collision_replan_enabled_, true);
        private_nh_.param("local_collision_replan_max_attempts",
                          local_collision_replan_max_attempts_, 1);
        private_nh_.param("local_collision_replan_inflation_radius",
                          local_collision_replan_inflation_radius_, 0.15);
        private_nh_.param("regional_ceiling/enabled", regional_ceiling_enabled_, false);
        private_nh_.param("regional_ceiling/ceiling_z", regional_ceiling_z_, 2.0);
        private_nh_.param("regional_ceiling/grid_resolution",
                          regional_ceiling_grid_resolution_, 0.2);
        private_nh_.getParam("regional_ceiling/region_x_min", regional_ceiling_x_min_);
        private_nh_.getParam("regional_ceiling/region_x_max", regional_ceiling_x_max_);
        private_nh_.getParam("regional_ceiling/region_y_min", regional_ceiling_y_min_);
        private_nh_.getParam("regional_ceiling/region_y_max", regional_ceiling_y_max_);
        if (super_corridor_extra_margin_ < 0.0) {
            throw std::invalid_argument("super_corridor_extra_margin must be non-negative");
        }
        if (narrow_corridor_clearance_threshold_ <= collision_clearance_ ||
            narrow_corridor_max_width_ <= 2.0 * narrow_corridor_min_half_width_ ||
            narrow_corridor_transition_length_ <= 0.0 ||
            narrow_corridor_max_half_width_ < narrow_corridor_min_half_width_ ||
            narrow_corridor_margin_ < 0.0 ||
            narrow_corridor_side_vertical_window_ <= 0.0 ||
            narrow_corridor_side_longitudinal_window_ <= 0.0 ||
            narrow_corridor_max_plane_violation_ <= 0.0) {
            throw std::invalid_argument("invalid narrow corridor configuration");
        }
        if (vertical_guide_floor_max_violation_ < 0.0 ||
            vertical_guide_floor_max_violation_ > 0.02) {
            throw std::invalid_argument("invalid vertical guide floor configuration");
        }
        if (guide_tracking_horizontal_half_width_ <= 0.0 ||
            guide_tracking_monotonic_vertical_band_ <= 0.0 ||
            guide_tracking_max_plane_violation_ <= 0.0) {
            throw std::invalid_argument("invalid guide tracking configuration");
        }
        if (local_collision_replan_max_attempts_ < 0 ||
            local_collision_replan_inflation_radius_ <= 0.0) {
            throw std::invalid_argument("invalid local collision replan parameters");
        }
        if (sample_dt_ <= 0.0 || collision_validation_step_ <= 0.0) {
            throw std::invalid_argument("trajectory validation steps must be positive");
        }
        if (execution_max_velocity_ <= 0.0 || execution_max_velocity_ > max_velocity_ ||
            execution_max_yaw_rate_ <= 0.0 || execution_max_yaw_rate_ > max_yaw_rate_) {
            throw std::invalid_argument("execution speed limits must be positive and no larger than certified limits");
        }
        if (regional_ceiling_enabled_) {
            const std::size_t count = regional_ceiling_x_min_.size();
            if (count == 0 || regional_ceiling_x_max_.size() != count ||
                regional_ceiling_y_min_.size() != count ||
                regional_ceiling_y_max_.size() != count ||
                regional_ceiling_grid_resolution_ <= 0.0 ||
                regional_ceiling_z_ <= collision_clearance_) {
                throw std::invalid_argument("invalid regional ceiling configuration");
            }
            for (std::size_t i = 0; i < count; ++i) {
                if (regional_ceiling_x_min_[i] > regional_ceiling_x_max_[i] ||
                    regional_ceiling_y_min_[i] > regional_ceiling_y_max_[i]) {
                    throw std::invalid_argument("invalid regional ceiling bounds");
                }
            }
            ROS_INFO("[FULL_SMOOTH] regional ceiling: z=%.2f m over %zu regions; %.2f m clearance gives hard vehicle-reference limit z<=%.2f m",
                     regional_ceiling_z_, count, collision_clearance_,
                     regional_ceiling_z_ - collision_clearance_);
        }
        ROS_INFO("[FULL_SMOOTH] safe-corridor mode: %s",
                 use_super_safe_corridor_
                     ? (super_use_recorded_guide_without_astar_
                            ? "validated recorded guide + SFC + MINCO"
                            : "SUPER A* + SFC + MINCO")
                     : "legacy waypoint MINCO");
        private_nh_.param("reload_topic", reload_topic_,
                          std::string("/waypoint_recorder/full_smooth_preview"));
        private_nh_.param("route_param", route_param_,
                          std::string("/waypoint_recorder/generated_route_path"));
        private_nh_.param("map_param", map_param_,
                          std::string("/waypoint_recorder/generated_map_path"));

        command_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>(command_topic_, 20);
        select_pub_ = nh_.advertise<std_msgs::Bool>(select_topic_, 1, true);
        path_pub_ = nh_.advertise<nav_msgs::Path>(path_topic_, 1, true);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 1, true);
        if (auto_land_) {
            land_pub_ = nh_.advertise<quadrotor_msgs::TakeoffLand>(land_topic_, 1);
        }
        odom_sub_ = nh_.subscribe(odom_topic_, 20, &FullSmoothMission::odomCallback, this);
        if (execute_saved_trajectory_ || load_saved_trajectory_for_preview_) {
            known_map_.reset(new pcl::PointCloud<pcl::PointXYZ>);
            if (known_map_pcd_.empty() ||
                pcl::io::loadPCDFile(known_map_pcd_, *known_map_) < 0 || known_map_->empty()) {
                throw std::runtime_error("saved execution requires its certified collision PCD");
            }
            map_tree_.setInputCloud(known_map_);
        }
        if (execute_saved_trajectory_) {
            loadSavedTrajectory(saved_trajectory_path_);
        } else {
            if (load_saved_trajectory_for_preview_) {
                loadSavedTrajectory(saved_trajectory_path_);
                ROS_INFO("[FULL_SMOOTH] displaying saved trajectory before any recorder reload");
            }
            if (wait_for_reload_) {
                reload_sub_ = nh_.subscribe(reload_topic_, 1, &FullSmoothMission::reloadCallback, this);
                ROS_INFO("[FULL_SMOOTH] waiting for a recorder-generated route and PCD on %s",
                         reload_topic_.c_str());
            } else if (!load_saved_trajectory_for_preview_) {
                loadRouteAndMap(route_path_, known_map_pcd_);
                // Offline inspection has no live odometry stream.  Planning from
                // the first recorded point is explicitly preview-only and never
                // publishes commands, so build it immediately for RViz.
                if (preview_only_ && preview_start_at_first_route_point_) {
                    try {
                        buildPreviewFromFirstRoutePoint();
                    } catch (const std::exception& error) {
                        handleBuildFailure(error.what());
                    }
                }
            }
        }
        if (start_trigger_type_ == 3) {
            trigger_sub_ = nh_.subscribe(trigger_topic_, 1, &FullSmoothMission::triggerCallback, this);
        }
        timer_ = nh_.createTimer(ros::Duration(0.01), &FullSmoothMission::timerCallback, this);
        start_time_ = ros::Time::now().toSec();
        publishSelection(false);
        bool exit_after_preview = false;
        private_nh_.param("exit_after_preview", exit_after_preview, false);
        if (exit_after_preview) {
            if (!preview_only_ || !preview_built_ || !trajectory_safe_) {
                throw std::runtime_error("offline final MINCO generation/validation failed");
            }
            ros::shutdown();
        }
    }

private:
    struct Phase {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        struct YawLock {
            double start_time{0.0};
            double end_time{0.0};
            double start_yaw{0.0};
            double end_yaw{0.0};
        };

        MincoTrajectory trajectory;
        double start_yaw{0.0};
        double target_yaw{0.0};
        double start_blend_duration{0.0};
        double terminal_blend_duration{0.0};
        double yaw_profile_dt{0.0};
        std::vector<double> yaw_profile;
        std::vector<YawLock> yaw_locks;
        double dwell{0.0};
    };

    struct YawSample {
        double yaw{0.0};
        double rate{0.0};
    };

    struct SavedTrajectoryHeader {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Vector3d start_position{Eigen::Vector3d::Zero()};
        double start_yaw{0.0};
    };

    struct FailureDiagnostic {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        bool valid{false};
        int phase_index{0};
        Eigen::Vector3d segment_start{Eigen::Vector3d::Zero()};
        Eigen::Vector3d segment_end{Eigen::Vector3d::Zero()};
        Eigen::Vector3d closest_path_point{Eigen::Vector3d::Zero()};
        Eigen::Vector3d closest_obstacle{Eigen::Vector3d::Zero()};
        double clearance{std::numeric_limits<double>::infinity()};
        std::string reason;
    };

    struct NarrowCorridorDiagnostic {
        int phase_index{0};
        int corridor_index{0};
        Eigen::Vector3d guide_point{Eigen::Vector3d::Zero()};
        Eigen::Vector3d centerline{Eigen::Vector3d::Zero()};
        Eigen::Vector3d tangent{Eigen::Vector3d::Zero()};
        Eigen::Vector3d tube_normal{Eigen::Vector3d::Zero()};
        Eigen::Vector3d seed_start{Eigen::Vector3d::Zero()};
        Eigen::Vector3d seed_end{Eigen::Vector3d::Zero()};
        double path_clearance{0.0};
        double left_clearance{0.0};
        double right_clearance{0.0};
        double available_half_width{0.0};
        double applied_half_width{0.0};
        double transition_distance{0.0};
        bool candidate{false};
        bool applied{false};
        std::string tube_normal_source{"local_guide"};
        std::string reason;
    };

    using SuperVec3 = super_utils::Vec3f;
    using SuperPath = super_utils::vec_Vec3f;

    static std::string workspaceOutputFile(const std::string& filename) {
        return std::string(ROOT_DIR) + "../../../" + filename;
    }

    static geometry_msgs::Point toRosPoint(const Eigen::Vector3d& point) {
        geometry_msgs::Point result;
        result.x = point.x();
        result.y = point.y();
        result.z = point.z();
        return result;
    }

    void clearFailureDiagnostic() {
        failure_diagnostic_ = FailureDiagnostic{};
    }

    int regionalCeilingRegion(const Eigen::Vector3d& position,
                              const double horizontal_expansion = 0.0) const {
        if (!regional_ceiling_enabled_) {
            return -1;
        }
        for (std::size_t i = 0; i < regional_ceiling_x_min_.size(); ++i) {
            if (position.x() >= regional_ceiling_x_min_[i] - horizontal_expansion &&
                position.x() <= regional_ceiling_x_max_[i] + horizontal_expansion &&
                position.y() >= regional_ceiling_y_min_[i] - horizontal_expansion &&
                position.y() <= regional_ceiling_y_max_[i] + horizontal_expansion) {
                return static_cast<int>(i);
            }
        }
        return -1;
    }

    double regionalMaximumVehicleZ() const {
        return regional_ceiling_z_ - collision_clearance_;
    }

    bool regionalHeightIsSafe(const Eigen::Vector3d& position,
                              const double motion_reserve = 0.0) const {
        return regionalCeilingRegion(position, motion_reserve) < 0 ||
               position.z() + motion_reserve <= regionalMaximumVehicleZ() + 1.0e-9;
    }

    void appendRegionalCeilingConstraints(pcl::PointCloud<pcl::PointXYZ>& cloud) {
        regional_ceiling_constraint_points_ = 0;
        if (!regional_ceiling_enabled_) {
            return;
        }
        for (std::size_t region = 0; region < regional_ceiling_x_min_.size(); ++region) {
            const int x_count = std::max(1, static_cast<int>(std::ceil(
                (regional_ceiling_x_max_[region] - regional_ceiling_x_min_[region]) /
                regional_ceiling_grid_resolution_)));
            const int y_count = std::max(1, static_cast<int>(std::ceil(
                (regional_ceiling_y_max_[region] - regional_ceiling_y_min_[region]) /
                regional_ceiling_grid_resolution_)));
            for (int xi = 0; xi <= x_count; ++xi) {
                const double x = std::min(
                    regional_ceiling_x_min_[region] +
                        xi * regional_ceiling_grid_resolution_,
                    regional_ceiling_x_max_[region]);
                for (int yi = 0; yi <= y_count; ++yi) {
                    const double y = std::min(
                        regional_ceiling_y_min_[region] +
                            yi * regional_ceiling_grid_resolution_,
                        regional_ceiling_y_max_[region]);
                    pcl::PointXYZ point;
                    point.x = static_cast<float>(x);
                    point.y = static_cast<float>(y);
                    point.z = static_cast<float>(regional_ceiling_z_);
                    cloud.push_back(point);
                    ++regional_ceiling_constraint_points_;
                }
            }
        }
        cloud.width = static_cast<std::uint32_t>(cloud.size());
        cloud.height = 1;
        cloud.is_dense = true;
    }

    bool nearestObstacle(const Eigen::Vector3d& position, Eigen::Vector3d& obstacle,
                         double& distance) {
        if (!known_map_ || known_map_->empty()) {
            return false;
        }
        pcl::PointXYZ query;
        query.x = position.x();
        query.y = position.y();
        query.z = position.z();
        std::vector<int> indices(1);
        std::vector<float> squared_distances(1);
        if (map_tree_.nearestKSearch(query, 1, indices, squared_distances) <= 0) {
            return false;
        }
        const auto& nearest = known_map_->points[indices.front()];
        obstacle = Eigen::Vector3d(nearest.x, nearest.y, nearest.z);
        distance = std::sqrt(squared_distances.front());
        return true;
    }

    double minimumRawPcdClearance(const SuperPath& path) {
        double minimum = std::numeric_limits<double>::infinity();
        for (std::size_t index = 1; index < path.size(); ++index) {
            const Eigen::Vector3d start = path[index - 1];
            const Eigen::Vector3d end = path[index];
            const double length = (end - start).norm();
            const int samples = std::max(1, static_cast<int>(std::ceil(length / 0.01)));
            for (int sample_index = 0; sample_index <= samples; ++sample_index) {
                const Eigen::Vector3d position = start +
                    (end - start) * static_cast<double>(sample_index) / samples;
                Eigen::Vector3d obstacle;
                double distance = std::numeric_limits<double>::infinity();
                if (nearestObstacle(position, obstacle, distance)) {
                    minimum = std::min(minimum, distance);
                }
            }
        }
        return minimum;
    }

    bool rawPcdPointIsSafe(const SuperVec3& point) {
        if (!regionalHeightIsSafe(point)) {
            return false;
        }
        Eigen::Vector3d obstacle;
        double distance = std::numeric_limits<double>::infinity();
        return nearestObstacle(point, obstacle, distance) &&
               distance + 1.0e-6 >= collision_clearance_;
    }

    bool rawPcdSegmentIsSafe(const SuperVec3& start, const SuperVec3& end) {
        const double length = (end - start).norm();
        const int samples = std::max(1, static_cast<int>(std::ceil(length / 0.01)));
        for (int sample_index = 0; sample_index <= samples; ++sample_index) {
            const Eigen::Vector3d position = start +
                (end - start) * static_cast<double>(sample_index) / samples;
            if (!regionalHeightIsSafe(position)) {
                return false;
            }
            Eigen::Vector3d obstacle;
            double distance = std::numeric_limits<double>::infinity();
            if (!nearestObstacle(position, obstacle, distance) ||
                distance + 1.0e-6 < collision_clearance_) {
                return false;
            }
        }
        return true;
    }

    bool estimateNarrowCorridorAt(const Eigen::Vector3d& point,
                                  const Eigen::Vector2d& tangent,
                                  double& path_clearance,
                                  double& left_clearance,
                                  double& right_clearance,
                                  Eigen::Vector3d& centerline) const {
        if (!known_map_ || known_map_->empty() || tangent.norm() < 1.0e-6) {
            return false;
        }
        pcl::PointXYZ query;
        query.x = static_cast<float>(point.x());
        query.y = static_cast<float>(point.y());
        query.z = static_cast<float>(point.z());
        std::vector<int> nearest_index(1);
        std::vector<float> nearest_squared_distance(1);
        if (map_tree_.nearestKSearch(query, 1, nearest_index,
                                     nearest_squared_distance) <= 0) {
            return false;
        }
        path_clearance = std::sqrt(nearest_squared_distance.front());
        const Eigen::Vector2d unit_tangent = tangent.normalized();
        const Eigen::Vector2d normal(-unit_tangent.y(), unit_tangent.x());
        left_clearance = std::numeric_limits<double>::infinity();
        right_clearance = std::numeric_limits<double>::infinity();

        std::vector<int> indices;
        std::vector<float> squared_distances;
        const double search_radius = std::max(
            1.0, narrow_corridor_side_longitudinal_window_ +
                      narrow_corridor_max_width_);
        if (map_tree_.radiusSearch(query, search_radius, indices,
                                   squared_distances) <= 0) {
            return false;
        }
        for (const int index : indices) {
            const auto& obstacle = known_map_->points[index];
            const Eigen::Vector3d delta(
                obstacle.x - point.x(), obstacle.y - point.y(), obstacle.z - point.z());
            if (std::abs(delta.z()) > narrow_corridor_side_vertical_window_) {
                continue;
            }
            const double along = delta.head<2>().dot(unit_tangent);
            if (std::abs(along) > narrow_corridor_side_longitudinal_window_) {
                continue;
            }
            const double lateral = delta.head<2>().dot(normal);
            if (lateral > 0.05) {
                left_clearance = std::min(left_clearance, lateral);
            } else if (lateral < -0.05) {
                right_clearance = std::min(right_clearance, -lateral);
            }
        }
        if (!std::isfinite(left_clearance) || !std::isfinite(right_clearance)) {
            return false;
        }
        const double center_offset = 0.5 * (left_clearance - right_clearance);
        centerline = point;
        centerline.x() += normal.x() * center_offset;
        centerline.y() += normal.y() * center_offset;
        return path_clearance <= narrow_corridor_clearance_threshold_ &&
               left_clearance + right_clearance <= narrow_corridor_max_width_;
    }

    static double pointToLineParameter(const Eigen::Vector3d& point,
                                       const Eigen::Vector3d& start,
                                       const Eigen::Vector3d& end) {
        const Eigen::Vector3d delta = end - start;
        const double denominator = delta.squaredNorm();
        if (denominator <= 1.0e-12) {
            return 0.0;
        }
        return (point - start).dot(delta) / denominator;
    }

    // Recorded guides contain repeated/near-repeated points at observation
    // stops and at clearance-refinement joins.  A one-sample finite
    // difference at such a join is not a useful passage direction.  Estimate
    // the local direction from the nearest points on either side that are at
    // least 5 cm away in accumulated path length; this keeps a short doorway
    // turn local without treating a duplicate point as a wall direction.
    static Eigen::Vector2d robustGuideTangent(const SuperPath& guide_path,
                                              const std::size_t guide_index,
                                              const Eigen::Vector2d& fallback) {
        constexpr double minimum_span = 0.05;
        std::size_t previous = guide_index;
        double previous_span = 0.0;
        while (previous > 0 && previous_span < minimum_span) {
            previous_span += (guide_path[previous] - guide_path[previous - 1]).head<2>().norm();
            --previous;
        }
        std::size_t next = guide_index;
        double next_span = 0.0;
        while (next + 1 < guide_path.size() && next_span < minimum_span) {
            next_span += (guide_path[next + 1] - guide_path[next]).head<2>().norm();
            ++next;
        }
        Eigen::Vector2d tangent = (guide_path[next] - guide_path[previous]).head<2>();
        if (tangent.norm() < 1.0e-6) {
            tangent = fallback;
        }
        return tangent.normalized();
    }

    bool tightenNarrowCorridors(const SuperPath& guide_path,
                                geometry_utils::PolytopeVec& corridor,
                                const int phase_index) {
        narrow_corridor_diagnostics_.clear();
        if (!narrow_corridor_enabled_ || guide_path.size() < 3 || corridor.empty()) {
            return true;
        }

        int candidate_count = 0;
        int applied_count = 0;
        for (std::size_t corridor_index = 0; corridor_index < corridor.size();
             ++corridor_index) {
            if (!corridor[corridor_index].HaveSeedLine()) {
                continue;
            }
            const auto seed = corridor[corridor_index].seed_line;
            const Eigen::Vector3d seed_start(seed.first.x(), seed.first.y(), seed.first.z());
            const Eigen::Vector3d seed_end(seed.second.x(), seed.second.y(), seed.second.z());
            Eigen::Vector2d seed_tangent = (seed_end - seed_start).head<2>();
            if (seed_tangent.norm() < 1.0e-6) {
                continue;
            }
            seed_tangent.normalize();

            NarrowCorridorDiagnostic best;
            best.phase_index = phase_index;
            best.corridor_index = static_cast<int>(corridor_index);
            double best_width = std::numeric_limits<double>::infinity();
            for (std::size_t guide_index = 1; guide_index + 1 < guide_path.size();
                 ++guide_index) {
                const Eigen::Vector3d guide_point(
                    guide_path[guide_index].x(), guide_path[guide_index].y(),
                    guide_path[guide_index].z());
                const double parameter = pointToLineParameter(
                    guide_point, seed_start, seed_end);
                const Eigen::Vector3d projection = seed_start +
                    std::max(0.0, std::min(1.0, parameter)) * (seed_end - seed_start);
                const double distance_to_seed = (guide_point - projection).norm();
                const double along_distance = std::abs(parameter * (seed_end - seed_start).norm());
                if (distance_to_seed > 0.20 ||
                    (parameter < 0.0 && along_distance > narrow_corridor_transition_length_) ||
                    (parameter > 1.0 && along_distance - (seed_end - seed_start).norm() >
                                             narrow_corridor_transition_length_)) {
                    continue;
                }
                const Eigen::Vector2d tangent = robustGuideTangent(
                    guide_path, guide_index, seed_tangent);
                double path_clearance = 0.0, left_clearance = 0.0, right_clearance = 0.0;
                Eigen::Vector3d centerline;
                if (!estimateNarrowCorridorAt(guide_point, tangent, path_clearance,
                                               left_clearance, right_clearance, centerline)) {
                    continue;
                }
                const double width = left_clearance + right_clearance;
                if (width >= best_width) {
                    continue;
                }
                best_width = width;
                best.guide_point = guide_point;
                best.centerline = centerline;
                best.tangent = Eigen::Vector3d(tangent.x(), tangent.y(), 0.0);
                best.path_clearance = path_clearance;
                best.left_clearance = left_clearance;
                best.right_clearance = right_clearance;
                best.available_half_width = std::max(
                    narrow_corridor_min_half_width_,
                    0.5 * width - collision_clearance_ - narrow_corridor_margin_);
                best.transition_distance = std::max(
                    0.0, std::min(std::abs(parameter * (seed_end - seed_start).norm()),
                                  std::abs((parameter - 1.0) * (seed_end - seed_start).norm())));
                best.candidate = true;
            }
            if (!best.candidate) {
                continue;
            }
            ++candidate_count;
            // The added half-spaces must contain the whole CIRI seed.  Use
            // the measured local passage direction for the transverse
            // normal.  The original CIRI seed remains in the intersection
            // check below; it is not used to silently reintroduce a world-
            // or seed-chord-aligned diagonal degree of freedom.
            const Eigen::Vector2d local_tangent = best.tangent.head<2>().normalized();
            const Eigen::Vector2d seed_normal(-seed_tangent.y(), seed_tangent.x());
            Eigen::Vector2d normal(-local_tangent.y(), local_tangent.x());
            best.tube_normal_source = "local_guide";
            best.tube_normal = Eigen::Vector3d(normal.x(), normal.y(), 0.0);
            best.seed_start = seed_start;
            best.seed_end = seed_end;
            // The geometric midpoint is the preferred centerline.  If the
            // sparse PCD makes that midpoint slightly too close to an
            // obstacle, retain the nearest point on the original safe guide
            // segment instead of silently dropping the bottleneck constraint.
            if (!rawPcdPointIsSafe(best.centerline)) {
                const Eigen::Vector3d preferred = best.centerline;
                best.centerline = best.guide_point;
                for (int step = 1; step <= 20; ++step) {
                    const double alpha = static_cast<double>(step) / 20.0;
                    const Eigen::Vector3d candidate = best.guide_point +
                        alpha * (preferred - best.guide_point);
                    if (rawPcdPointIsSafe(candidate)) {
                        best.centerline = candidate;
                    } else {
                        break;
                    }
                }
            }
            double endpoint_span = std::max(
                std::abs((seed_start - best.centerline).head<2>().dot(normal)),
                std::abs((seed_end - best.centerline).head<2>().dot(normal)));
            double half_width = std::min(
                narrow_corridor_max_half_width_,
                std::max(best.available_half_width, endpoint_span + 0.01));
            // A corridor seed can straddle a genuine bend.  In that case the
            // local tube may not contain both seed endpoints even though the
            // neighboring seed in the same bottleneck does.  Preserve a
            // feasible transition constraint in the seed's own local frame,
            // and record that fallback explicitly; never silently drop it.
            if (half_width < narrow_corridor_min_half_width_ ||
                endpoint_span > half_width - 0.005) {
                normal = seed_normal;
                endpoint_span = std::max(
                    std::abs((seed_start - best.centerline).head<2>().dot(normal)),
                    std::abs((seed_end - best.centerline).head<2>().dot(normal)));
                half_width = std::min(
                    narrow_corridor_max_half_width_,
                    std::max(best.available_half_width, endpoint_span + 0.01));
                best.tube_normal_source = "ciri_seed_transition";
                best.tube_normal = Eigen::Vector3d(normal.x(), normal.y(), 0.0);
            }
            best.applied_half_width = half_width;
            if (half_width < narrow_corridor_min_half_width_ ||
                endpoint_span > half_width - 0.005) {
                best.reason = "seed_line_outside_feasible_centered_tube";
                ROS_WARN("[FULL_SMOOTH] narrow candidate phase %d corridor %zu rejected: available half-width %.3f, applied %.3f, endpoint span %.3f, center=[%.3f %.3f %.3f]",
                         phase_index, corridor_index, best.available_half_width, half_width,
                         endpoint_span, best.centerline.x(), best.centerline.y(), best.centerline.z());
                narrow_corridor_diagnostics_.push_back(best);
                continue;
            }
            const Eigen::Vector3d center = best.centerline;
            const auto original_planes = corridor[corridor_index].GetPlanes();
            Eigen::Matrix<double, Eigen::Dynamic, 4> planes(
                original_planes.rows() + 2, 4);
            planes.topRows(original_planes.rows()) = original_planes;
            planes.row(original_planes.rows()) << normal.x(), normal.y(), 0.0,
                -(normal.dot(center.head<2>()) + half_width);
            planes.row(original_planes.rows() + 1) << -normal.x(), -normal.y(), 0.0,
                normal.dot(center.head<2>() ) - half_width;
            Eigen::Vector3d interior;
            if (!geometry_utils::findInterior(planes, interior)) {
                best.reason = "centered_tube_intersection_is_empty";
                ROS_WARN("[FULL_SMOOTH] narrow candidate phase %d corridor %zu rejected: centered tube polytope is empty (half-width %.3f, center=[%.3f %.3f %.3f])",
                         phase_index, corridor_index, half_width, center.x(), center.y(), center.z());
                narrow_corridor_diagnostics_.push_back(best);
                continue;
            }
            if (!rawPcdPointIsSafe(center)) {
                best.reason = "estimated_centerline_is_not_raw_pcd_safe";
                ROS_WARN("[FULL_SMOOTH] narrow candidate phase %d corridor %zu rejected: centerline is not raw-PCD safe (center=[%.3f %.3f %.3f])",
                         phase_index, corridor_index, center.x(), center.y(), center.z());
                narrow_corridor_diagnostics_.push_back(best);
                continue;
            }
            corridor[corridor_index].SetPlanes(planes);
            bool overlap_is_feasible = true;
            for (const int neighbor : {-1, 1}) {
                const int adjacent = static_cast<int>(corridor_index) + neighbor;
                if (adjacent < 0 || adjacent >= static_cast<int>(corridor.size())) {
                    continue;
                }
                const std::size_t first = static_cast<std::size_t>(
                    std::min(adjacent, static_cast<int>(corridor_index)));
                const std::size_t second = static_cast<std::size_t>(
                    std::max(adjacent, static_cast<int>(corridor_index)));
                const auto overlap = corridor[first].CrossWith(corridor[second]);
                Eigen::Vector3d overlap_interior;
                if (!geometry_utils::findInterior(overlap.GetPlanes(), overlap_interior)) {
                    overlap_is_feasible = false;
                    break;
                }
            }
            if (!overlap_is_feasible) {
                // A curved passage may make one seed's local direction
                // incompatible with the previous/next seed even though the
                // same bottleneck has another feasible corridor.  Roll back
                // only this seed; the group-level check below still refuses
                // an entirely unconstrained bottleneck.
                corridor[corridor_index].SetPlanes(original_planes);
                best.reason = "local_tube_breaks_adjacent_corridor_overlap";
                ROS_WARN("[FULL_SMOOTH] narrow candidate phase %d corridor %zu rejected: local tube breaks adjacent corridor overlap",
                         phase_index, corridor_index);
                narrow_corridor_diagnostics_.push_back(best);
                continue;
            }
            best.applied = true;
            best.reason = "applied_xy_centerline_halfspaces";
            narrow_corridor_diagnostics_.push_back(best);
            ++applied_count;
        }

        if (candidate_count == 0) {
            ROS_INFO("[FULL_SMOOTH] narrow-corridor detector found no two-sided bottleneck in phase %d",
                     phase_index);
            return true;
        }
        for (std::size_t index = 1; index < corridor.size(); ++index) {
            const auto overlap = corridor[index - 1].CrossWith(corridor[index]);
            Eigen::Vector3d interior;
            if (!geometry_utils::findInterior(overlap.GetPlanes(), interior)) {
                ROS_ERROR("[FULL_SMOOTH] narrow-corridor constraints destroy corridor overlap between pieces %zu and %zu in phase %d",
                          index - 1, index, phase_index);
                return false;
            }
        }
        // Multiple CIRI seeds can cover the same physical bottleneck.  A
        // local direction constraint is acceptable only if every detected
        // bottleneck group retains at least one applied corridor; otherwise
        // refusing the phase is safer than silently using the unconstrained
        // MINCO solution.
        for (std::size_t index = 0; index < narrow_corridor_diagnostics_.size(); ++index) {
            const auto& candidate = narrow_corridor_diagnostics_[index];
            if (!candidate.candidate) {
                continue;
            }
            bool group_has_applied = candidate.applied;
            for (std::size_t other = 0; other < narrow_corridor_diagnostics_.size(); ++other) {
                const auto& member = narrow_corridor_diagnostics_[other];
                if (member.phase_index != candidate.phase_index || !member.applied) {
                    continue;
                }
                if ((member.centerline - candidate.centerline).head<2>().norm() <= 0.18) {
                    group_has_applied = true;
                    break;
                }
            }
            if (!group_has_applied) {
                ROS_ERROR("[FULL_SMOOTH] narrow-corridor bottleneck at phase %d center=[%.3f %.3f %.3f] has no feasible applied corridor; refusing unconstrained bottleneck",
                          candidate.phase_index, candidate.centerline.x(),
                          candidate.centerline.y(), candidate.centerline.z());
                return false;
            }
        }
        narrow_corridor_report_.insert(narrow_corridor_report_.end(),
                                      narrow_corridor_diagnostics_.begin(),
                                      narrow_corridor_diagnostics_.end());
        ROS_INFO("[FULL_SMOOTH] applied %d automatic XY centerline bottleneck corridor constraints in phase %d",
                 applied_count, phase_index);
        return true;
    }

    double maximumCorridorViolation(const geometry_utils::Trajectory& trajectory,
                                    const geometry_utils::PolytopeVec& corridor) const {
        if (trajectory.getPieceNum() != static_cast<int>(corridor.size())) {
            return std::numeric_limits<double>::infinity();
        }
        double maximum = -std::numeric_limits<double>::infinity();
        for (int piece_index = 0; piece_index < trajectory.getPieceNum(); ++piece_index) {
            auto planes = corridor[piece_index].GetPlanes();
            const Eigen::ArrayXd norms = planes.leftCols<3>().rowwise().norm();
            planes.array().colwise() /= norms;
            const auto& piece = trajectory[piece_index];
            constexpr int sample_count = 200;
            for (int sample_index = 0; sample_index <= sample_count; ++sample_index) {
                const Eigen::Vector3d position = piece.getPos(
                    piece.getDuration() * sample_index / sample_count);
                const Eigen::Vector4d homogeneous(position.x(), position.y(), position.z(), 1.0);
                maximum = std::max(maximum, (planes * homogeneous).maxCoeff());
            }
        }
        return maximum;
    }

    bool applyVerticalGuideFloor(const SuperPath& guide_path,
                                 geometry_utils::PolytopeVec& corridor,
                                 const int phase_index,
                                 double& floor_z) const {
        floor_z = std::numeric_limits<double>::infinity();
        for (const auto& point : guide_path) {
            floor_z = std::min(floor_z, static_cast<double>(point.z()));
        }
        if (!vertical_guide_floor_enabled_ || !std::isfinite(floor_z)) {
            return true;
        }

        // CIRI half spaces use n.x * x + n.y * y + n.z * z + d <= 0.
        // Intersect every corridor in this phase with z >= min(guide.z).
        // The collision-checked guide itself remains feasible, while MINCO is
        // no longer free to create a downward overshoot solely to reduce its
        // smoothness cost.
        for (std::size_t index = 0; index < corridor.size(); ++index) {
            const auto original = corridor[index].GetPlanes();
            Eigen::Matrix<double, Eigen::Dynamic, 4> planes(original.rows() + 1, 4);
            planes.topRows(original.rows()) = original;
            planes.row(original.rows()) << 0.0, 0.0, -1.0, floor_z;
            Eigen::Vector3d interior;
            if (!geometry_utils::findInterior(planes, interior)) {
                ROS_ERROR("[FULL_SMOOTH] vertical guide floor z>=%.3f makes corridor %zu infeasible in phase %d",
                          floor_z, index, phase_index);
                return false;
            }
            corridor[index].SetPlanes(planes);
        }
        for (std::size_t index = 1; index < corridor.size(); ++index) {
            const auto overlap = corridor[index - 1].CrossWith(corridor[index]);
            Eigen::Vector3d interior;
            if (!geometry_utils::findInterior(overlap.GetPlanes(), interior)) {
                ROS_ERROR("[FULL_SMOOTH] vertical guide floor z>=%.3f destroys corridor overlap %zu/%zu in phase %d",
                          floor_z, index - 1, index, phase_index);
                return false;
            }
        }
        ROS_INFO("[FULL_SMOOTH] applied vertical guide floor z>=%.3f m to %zu corridors in phase %d",
                 floor_z, corridor.size(), phase_index);
        return true;
    }

    static double minimumTrajectoryHeight(const geometry_utils::Trajectory& trajectory) {
        double minimum = std::numeric_limits<double>::infinity();
        for (int piece_index = 0; piece_index < trajectory.getPieceNum(); ++piece_index) {
            const auto& piece = trajectory[piece_index];
            constexpr int sample_count = 400;
            for (int sample_index = 0; sample_index <= sample_count; ++sample_index) {
                minimum = std::min(
                    minimum,
                    piece.getPos(piece.getDuration() * sample_index / sample_count).z());
            }
        }
        return minimum;
    }

    bool applyGuideTrackingEnvelope(const SuperPath& guide_path,
                                    geometry_utils::PolytopeVec& corridor,
                                    const int phase_index) const {
        if (!guide_tracking_enabled_ || guide_path.size() < 2 || corridor.empty()) {
            return true;
        }
        constexpr double monotonic_tolerance = 1.0e-4;
        bool nondecreasing = true;
        for (std::size_t index = 1; index < guide_path.size(); ++index) {
            const double delta_z = guide_path[index].z() - guide_path[index - 1].z();
            nondecreasing = nondecreasing && delta_z >= -monotonic_tolerance;
        }
        // The envelope is intended for departure phases whose safe guide is
        // level and then climbs.  Applying the same local tubes to a long
        // descending observation approach can make consecutive bend
        // corridors lose overlap without addressing the departure problem.
        // Those phases retain their original CIRI constraints and the global
        // guide floor; ascending/level phases get both XY tracking and the
        // progressive vertical envelope.
        const bool monotonic_vertical = nondecreasing;
        if (!monotonic_vertical) {
            ROS_INFO("[FULL_SMOOTH] guide tracking envelope skipped in phase %d because guide height is not nondecreasing",
                     phase_index);
            return true;
        }

        for (std::size_t index = 0; index < corridor.size(); ++index) {
            if (!corridor[index].HaveSeedLine()) {
                ROS_ERROR("[FULL_SMOOTH] guide tracking requires a seed line for corridor %zu in phase %d",
                          index, phase_index);
                return false;
            }
            const auto seed = corridor[index].seed_line;
            const Eigen::Vector3d start(seed.first.x(), seed.first.y(), seed.first.z());
            const Eigen::Vector3d end(seed.second.x(), seed.second.y(), seed.second.z());
            Eigen::Vector2d tangent = (end - start).head<2>();
            const auto original = corridor[index].GetPlanes();
            const int horizontal_plane_count = tangent.norm() > 1.0e-6 ? 2 : 0;
            const int vertical_plane_count = monotonic_vertical ? 2 : 0;
            Eigen::Matrix<double, Eigen::Dynamic, 4> planes(
                original.rows() + horizontal_plane_count + vertical_plane_count, 4);
            planes.topRows(original.rows()) = original;
            int row = original.rows();
            if (horizontal_plane_count > 0) {
                tangent.normalize();
                const Eigen::Vector2d normal(-tangent.y(), tangent.x());
                const Eigen::Vector2d center = 0.5 * (start + end).head<2>();
                planes.row(row++) << normal.x(), normal.y(), 0.0,
                    -(normal.dot(center) + guide_tracking_horizontal_half_width_);
                planes.row(row++) << -normal.x(), -normal.y(), 0.0,
                    normal.dot(center) - guide_tracking_horizontal_half_width_;
            }
            if (vertical_plane_count > 0) {
                const double lower = std::min(start.z(), end.z()) -
                                     guide_tracking_monotonic_vertical_band_;
                const double upper = std::max(start.z(), end.z()) +
                                     guide_tracking_monotonic_vertical_band_;
                planes.row(row++) << 0.0, 0.0, -1.0, lower;
                planes.row(row++) << 0.0, 0.0, 1.0, -upper;
            }
            Eigen::Vector3d interior;
            if (!geometry_utils::findInterior(planes, interior)) {
                ROS_ERROR("[FULL_SMOOTH] guide tracking envelope makes corridor %zu infeasible in phase %d",
                          index, phase_index);
                return false;
            }
            corridor[index].SetPlanes(planes);
        }
        for (std::size_t index = 1; index < corridor.size(); ++index) {
            const auto overlap = corridor[index - 1].CrossWith(corridor[index]);
            Eigen::Vector3d interior;
            if (!geometry_utils::findInterior(overlap.GetPlanes(), interior)) {
                ROS_ERROR("[FULL_SMOOTH] guide tracking envelope destroys corridor overlap %zu/%zu in phase %d",
                          index - 1, index, phase_index);
                return false;
            }
        }
        ROS_INFO("[FULL_SMOOTH] applied %.3f m horizontal guide tube to %zu corridors in phase %d%s",
                 guide_tracking_horizontal_half_width_, corridor.size(), phase_index,
                 monotonic_vertical ? " with monotonic vertical envelope" : "");
        return true;
    }

    void addLocalCollisionAvoidance(const FailureDiagnostic& collision) {
        // Updating an initialized ROG map in-place after CIRI has already
        // constructed corridors can leave CIRI's internal search in an
        // unbounded state.  Preserve the immutable PCD map and ask the normal
        // A* + SFC pipeline to pass a temporary local guide instead.  The
        // guide lies outside the hard clearance plus the requested replan
        // margin; the final continuous PCD check remains authoritative.
        Eigen::Vector3d away = collision.closest_path_point - collision.closest_obstacle;
        if (away.norm() < 1.0e-3) {
            away = collision.segment_end - collision.segment_start;
            away.z() = 0.0;
            if (away.norm() > 1.0e-3) {
                away = Eigen::Vector3d(-away.y(), away.x(), 0.0);
            } else {
                away = Eigen::Vector3d::UnitY();
            }
        }
        const Eigen::Vector3d guide = collision.closest_obstacle + away.normalized() *
            (collision_clearance_ + local_collision_replan_inflation_radius_);
        local_collision_guides_.push_back(guide);
        ROS_WARN("[FULL_SMOOTH] local collision replan: added temporary avoidance guide [%.3f, %.3f, %.3f] for obstacle [%.3f, %.3f, %.3f] (required radial clearance %.3f m)",
                 guide.x(), guide.y(), guide.z(), collision.closest_obstacle.x(),
                 collision.closest_obstacle.y(), collision.closest_obstacle.z(),
                 collision_clearance_ + local_collision_replan_inflation_radius_);
    }

    void recordFailurePath(const SuperPath& path, const int phase_index,
                           const std::string& reason) {
        FailureDiagnostic diagnostic;
        diagnostic.valid = path.size() >= 2;
        diagnostic.phase_index = phase_index;
        diagnostic.reason = reason;
        for (std::size_t index = 1; index < path.size(); ++index) {
            const Eigen::Vector3d start(path[index - 1].x(), path[index - 1].y(), path[index - 1].z());
            const Eigen::Vector3d end(path[index].x(), path[index].y(), path[index].z());
            const double length = (end - start).norm();
            const int samples = std::max(1, static_cast<int>(std::ceil(length / 0.05)));
            for (int sample_index = 0; sample_index <= samples; ++sample_index) {
                const Eigen::Vector3d position = start +
                    (end - start) * static_cast<double>(sample_index) / samples;
                Eigen::Vector3d obstacle;
                double distance = std::numeric_limits<double>::infinity();
                if (nearestObstacle(position, obstacle, distance) &&
                    distance < diagnostic.clearance) {
                    diagnostic.clearance = distance;
                    diagnostic.segment_start = start;
                    diagnostic.segment_end = end;
                    diagnostic.closest_path_point = position;
                    diagnostic.closest_obstacle = obstacle;
                }
            }
        }
        if (diagnostic.valid) {
            failure_diagnostic_ = std::move(diagnostic);
        }
    }

    void recordFailureLine(const Eigen::Vector3d& start, const Eigen::Vector3d& end,
                           const int phase_index, const std::string& reason) {
        SuperPath line;
        line.emplace_back(start.x(), start.y(), start.z());
        line.emplace_back(end.x(), end.y(), end.z());
        recordFailurePath(line, phase_index, reason);
    }

    void writeSavedTrajectory(const Eigen::Vector3d& planned_start,
                              const double planned_start_yaw) const {
        if (phases_.empty()) {
            throw std::runtime_error("cannot save an empty full-smooth trajectory");
        }
        const std::string temporary_path = saved_trajectory_path_ + ".tmp";
        std::ofstream output(temporary_path, std::ios::out | std::ios::trunc);
        if (!output.is_open()) {
            throw std::runtime_error("cannot write saved trajectory: " + temporary_path);
        }
        output << std::setprecision(17);
        output << "FULL_SMOOTH_TRAJECTORY_V1\n";
        output << "START " << planned_start.x() << ' ' << planned_start.y() << ' '
               << planned_start.z() << ' ' << planned_start_yaw << "\n";
        for (const auto& phase : phases_) {
            const auto& trajectory = phase.trajectory.rawTrajectory();
            output << "PHASE " << phase.dwell << ' ' << phase.start_yaw << ' '
                   << phase.target_yaw << ' ' << phase.yaw_profile_dt << ' '
                   << phase.yaw_profile.size() << ' ' << trajectory.getPieceNum() << "\n";
            output << "YAW";
            for (const double yaw : phase.yaw_profile) {
                output << ' ' << yaw;
            }
            output << "\n";
            for (int piece_index = 0; piece_index < trajectory.getPieceNum(); ++piece_index) {
                const auto& piece = trajectory[piece_index];
                const auto& coefficients = piece.getCoeffMat();
                output << "PIECE " << piece.getDuration() << ' ' << coefficients.rows() << ' '
                       << coefficients.cols() << "\n";
                for (int row = 0; row < coefficients.rows(); ++row) {
                    for (int column = 0; column < coefficients.cols(); ++column) {
                        output << (column == 0 ? "" : " ") << coefficients(row, column);
                    }
                    output << "\n";
                }
            }
            output << "END_PHASE\n";
        }
        output << "END\n";
        output.close();
        if (!output) {
            throw std::runtime_error("failed while writing saved trajectory: " + temporary_path);
        }
        if (std::rename(temporary_path.c_str(), saved_trajectory_path_.c_str()) != 0) {
            std::remove(temporary_path.c_str());
            throw std::runtime_error("cannot replace saved trajectory: " + saved_trajectory_path_);
        }
    }

    void writeNarrowCorridorReport() const {
        if (saved_trajectory_path_.empty()) {
            return;
        }
        const std::string suffix = "/final_minco.txt";
        std::string report_path = saved_trajectory_path_;
        const std::size_t suffix_position = report_path.rfind(suffix);
        if (suffix_position != std::string::npos &&
            suffix_position + suffix.size() == report_path.size()) {
            report_path.replace(suffix_position, suffix.size(),
                                "/narrow_corridor_report.json");
        } else {
            report_path += ".narrow_corridor.json";
        }
        std::ofstream output(report_path + ".tmp", std::ios::out | std::ios::trunc);
        if (!output.is_open()) {
            throw std::runtime_error("cannot write narrow corridor report: " + report_path);
        }
        output << std::setprecision(17);
        output << "{\n  \"format\": \"pre_map_vln.narrow_corridor_report.v2\",\n"
               << "  \"enabled\": " << (narrow_corridor_enabled_ ? "true" : "false") << ",\n"
               << "  \"clearance_threshold_m\": " << narrow_corridor_clearance_threshold_ << ",\n"
               << "  \"max_width_m\": " << narrow_corridor_max_width_ << ",\n"
               << "  \"transition_length_m\": " << narrow_corridor_transition_length_ << ",\n"
               << "  \"max_half_width_m\": " << narrow_corridor_max_half_width_ << ",\n"
               << "  \"min_half_width_m\": " << narrow_corridor_min_half_width_ << ",\n"
               << "  \"margin_m\": " << narrow_corridor_margin_ << ",\n"
               << "  \"constraints\": [\n";
        for (std::size_t index = 0; index < narrow_corridor_report_.size(); ++index) {
            const auto& diagnostic = narrow_corridor_report_[index];
            const auto write_vector = [&output](const Eigen::Vector3d& value) {
                output << "[" << value.x() << ", " << value.y() << ", " << value.z() << "]";
            };
            output << "    {\"phase\": " << diagnostic.phase_index
                   << ", \"corridor\": " << diagnostic.corridor_index
                   << ", \"candidate\": " << (diagnostic.candidate ? "true" : "false")
                   << ", \"applied\": " << (diagnostic.applied ? "true" : "false")
                   << ", \"guide_point\": ";
            write_vector(diagnostic.guide_point);
            output << ", \"centerline\": ";
            write_vector(diagnostic.centerline);
            output << ", \"tangent\": ";
            write_vector(diagnostic.tangent);
            output << ", \"tube_normal\": ";
            write_vector(diagnostic.tube_normal);
            output << ", \"tube_normal_source\": \"" << diagnostic.tube_normal_source << "\"";
            output << ", \"seed_start\": ";
            write_vector(diagnostic.seed_start);
            output << ", \"seed_end\": ";
            write_vector(diagnostic.seed_end);
            output << ", \"path_clearance_m\": " << diagnostic.path_clearance
                   << ", \"left_clearance_m\": " << diagnostic.left_clearance
                   << ", \"right_clearance_m\": " << diagnostic.right_clearance
                   << ", \"available_half_width_m\": " << diagnostic.available_half_width
                   << ", \"applied_half_width_m\": " << diagnostic.applied_half_width
                   << ", \"transition_distance_m\": " << diagnostic.transition_distance
                   << ", \"reason\": \"" << diagnostic.reason << "\"}"
                   << (index + 1 == narrow_corridor_report_.size() ? "\n" : ",\n");
        }
        output << "  ]\n}\n";
        output.close();
        if (!output || std::rename((report_path + ".tmp").c_str(), report_path.c_str()) != 0) {
            std::remove((report_path + ".tmp").c_str());
            throw std::runtime_error("cannot replace narrow corridor report: " + report_path);
        }
    }

    void validateSavedTrajectoryForExecution() {
        saved_peak_velocity_ = 0.0;
        saved_peak_yaw_rate_ = 0.0;
        for (std::size_t phase_index = 0; phase_index < phases_.size(); ++phase_index) {
            const auto& phase = phases_[phase_index];
            const double exact_max_speed =
                phase.trajectory.rawTrajectory().getMaxVelRate();
            if (exact_max_speed > max_velocity_ + 1.0e-6) {
                std::ostringstream error;
                error << "saved trajectory phase " << phase_index + 1
                      << " exceeds execution velocity limit: " << exact_max_speed
                      << " m/s > " << max_velocity_ << " m/s";
                throw std::runtime_error(error.str());
            }
            saved_peak_velocity_ = std::max(saved_peak_velocity_, exact_max_speed);
            const double validation_dt = exact_max_speed > 1.0e-6
                ? std::min(sample_dt_, collision_validation_step_ / exact_max_speed)
                : sample_dt_;
            const int count = std::max(1, static_cast<int>(
                std::ceil(phase.trajectory.duration() / validation_dt)));
            const double motion_reserve = 0.5 * exact_max_speed *
                phase.trajectory.duration() / count;
            for (int i = 0; i <= count; ++i) {
                const double t = phase.trajectory.duration() * i / count;
                const auto sample = phase.trajectory.sample(t);
                Eigen::Vector3d obstacle;
                double distance;
                if (!nearestObstacle(sample.position, obstacle, distance) ||
                    distance - motion_reserve < collision_clearance_) {
                    throw std::runtime_error("saved MINCO violates certified PCD clearance");
                }
                const double yaw_rate = std::abs(commandYaw(phase, t).rate);
                saved_peak_yaw_rate_ = std::max(saved_peak_yaw_rate_, yaw_rate);
                if (yaw_rate > max_yaw_rate_ + 1e-6) {
                    throw std::runtime_error("saved MINCO exceeds execution yaw-rate limit");
                }
                if (!regionalHeightIsSafe(sample.position, motion_reserve)) {
                    std::ostringstream error;
                    error << "saved trajectory phase " << phase_index + 1
                          << " exceeds regional vehicle-reference height limit: z="
                          << sample.position.z() << " m, limit="
                          << regionalMaximumVehicleZ() << " m";
                    throw std::runtime_error(error.str());
                }
                if (sample.acceleration.norm() > max_acceleration_ ||
                    sample.jerk.norm() > max_jerk_ ||
                    sample.snap.norm() > max_snap_) {
                    std::ostringstream error;
                    error << "saved trajectory phase " << phase_index + 1
                          << " exceeds execution dynamics limits";
                    throw std::runtime_error(error.str());
                }
            }
        }
        execution_time_scale_ = 1.0;
        if (saved_peak_velocity_ > 1.0e-6) {
            execution_time_scale_ = std::min(
                execution_time_scale_, execution_max_velocity_ / saved_peak_velocity_);
        }
        if (saved_peak_yaw_rate_ > 1.0e-6) {
            execution_time_scale_ = std::min(
                execution_time_scale_, execution_max_yaw_rate_ / saved_peak_yaw_rate_);
        }
        execution_time_scale_ = std::max(0.05, std::min(1.0, execution_time_scale_));
        ROS_WARN("[FULL_SMOOTH] runtime time scale %.3f: certified peaks v=%.3f m/s, yaw_rate=%.3f rad/s; execution limits v=%.3f m/s, yaw_rate=%.3f rad/s",
                 execution_time_scale_, saved_peak_velocity_, saved_peak_yaw_rate_,
                 execution_max_velocity_, execution_max_yaw_rate_);
    }

    void loadSavedTrajectory(const std::string& path) {
        std::ifstream input(path);
        if (!input.is_open()) {
            throw std::runtime_error("cannot open saved full-smooth trajectory: " + path);
        }
        std::string token;
        if (!(input >> token) || token != "FULL_SMOOTH_TRAJECTORY_V1") {
            throw std::runtime_error("unsupported saved full-smooth trajectory format: " + path);
        }
        if (!(input >> token) || token != "START" ||
            !(input >> saved_header_.start_position.x() >> saved_header_.start_position.y() >>
              saved_header_.start_position.z() >> saved_header_.start_yaw)) {
            throw std::runtime_error("saved trajectory has no valid START record: " + path);
        }

        phases_.clear();
        while (input >> token) {
            if (token == "END") {
                break;
            }
            if (token != "PHASE") {
                throw std::runtime_error("invalid saved trajectory phase record: " + path);
            }
            Phase phase;
            std::size_t yaw_count = 0;
            int piece_count = 0;
            if (!(input >> phase.dwell >> phase.start_yaw >> phase.target_yaw >>
                  phase.yaw_profile_dt >> yaw_count >> piece_count) ||
                phase.dwell < 0.0 || phase.yaw_profile_dt <= 0.0 || yaw_count < 2 ||
                piece_count <= 0) {
                throw std::runtime_error("invalid saved trajectory phase metadata: " + path);
            }
            if (!(input >> token) || token != "YAW") {
                throw std::runtime_error("saved trajectory phase has no yaw profile: " + path);
            }
            phase.yaw_profile.resize(yaw_count);
            for (double& yaw : phase.yaw_profile) {
                if (!(input >> yaw) || !std::isfinite(yaw)) {
                    throw std::runtime_error("invalid saved trajectory yaw profile: " + path);
                }
            }
            geometry_utils::Trajectory trajectory;
            trajectory.reserve(piece_count);
            for (int piece_index = 0; piece_index < piece_count; ++piece_index) {
                int rows = 0, columns = 0;
                double duration = 0.0;
                if (!(input >> token) || token != "PIECE" ||
                    !(input >> duration >> rows >> columns) || duration <= 0.0 ||
                    rows != 3 || columns < 2 || columns > 16) {
                    throw std::runtime_error("invalid saved trajectory polynomial: " + path);
                }
                Eigen::MatrixXd coefficients(rows, columns);
                for (int row = 0; row < rows; ++row) {
                    for (int column = 0; column < columns; ++column) {
                        if (!(input >> coefficients(row, column)) ||
                            !std::isfinite(coefficients(row, column))) {
                            throw std::runtime_error("invalid saved trajectory coefficient: " + path);
                        }
                    }
                }
                trajectory.emplace_back(duration, coefficients);
            }
            if (!(input >> token) || token != "END_PHASE") {
                throw std::runtime_error("unterminated saved trajectory phase: " + path);
            }
            phase.trajectory.setCorridorTrajectory(trajectory);
            phases_.push_back(std::move(phase));
        }
        if (phases_.empty() || token != "END") {
            throw std::runtime_error("saved trajectory contains no complete phases: " + path);
        }
        validateSavedTrajectoryForExecution();
        saved_trajectory_loaded_ = true;
        preview_built_ = true;
        trajectory_safe_ = true;
        state_ = State::WAIT_TRIGGER;
        publishVisualization();
        ROS_INFO("[FULL_SMOOTH] loaded %zu saved global phases from %s; no online replanning will run",
                 phases_.size(), path.c_str());
    }

    void initializeSuperCorridor(const Eigen::Vector3d& start) {
        (void)start;
        if (super_map_) {
            return;
        }
        if (super_config_path_.empty()) {
            throw std::runtime_error("super_config_path is required for safe-corridor planning");
        }
        super_ros_ = std::make_shared<ros_interface::Ros1Interface>(private_nh_);
        super_map_ = std::make_shared<rog_map::ROGMapROS>(
            private_nh_, super_config_path_, !super_static_map_only_, !super_static_map_only_);
        rog_map::PointCloud cloud;
        if (!planning_map_ || planning_map_->empty()) {
            throw std::runtime_error("planning map is unavailable");
        }
        cloud.reserve(planning_map_->size());
        for (const auto& point : planning_map_->points) {
            rog_map::PclPoint occupied;
            occupied.x = point.x;
            occupied.y = point.y;
            occupied.z = point.z;
            occupied.intensity = 1.0f;
            cloud.push_back(occupied);
        }
        super_map_->updateOccPointCloud(cloud);
        // ROG-Map starts with the fixed map origin from the SUPER configuration.
        // This preview uses a 50 m static map; the recorder's PCD is inserted as
        // occupied-only data, so no synthetic free-space ray is created.
        const super_planner::Config config(super_config_path_);
        // The PCD validator below is the final safety authority.  Do not let a
        // custom SUPER config build corridors for a smaller radius and then
        // reject the resulting MINCO trajectory afterwards.
        // ROG still supplies the coarse A* occupancy map, while CIRI below is
        // explicitly given the original PCD.  Any requested extra margin is
        // therefore a physical/tracking allowance, not a voxel-size patch.
        const double safe_corridor_radius =
            std::max(config.robot_r, collision_clearance_) + super_corridor_extra_margin_;
        // This limits only how much path geometry is bundled into one CIRI
        // seed.  It is not a clearance/radius.  Use SUPER's configured seed
        // span (0.8 m for the real-flight config) so valid longer guide edges
        // are subdivided/covered instead of failing against an implicit
        // two-radii cap.  CIRI still validates every corridor with the exact
        // configured safe_corridor_radius and filtered PCD.
        const double full_smooth_seed_line_length = config.corridor_line_max_length;
        super_astar_ = std::make_shared<path_search::Astar>(super_config_path_, super_ros_, super_map_);
        super_astar_->setTransitionValidator(
            [this](const SuperVec3& from, const SuperVec3& to) {
                (void)from;
                return rawPcdPointIsSafe(to);
            });
        super_astar_->setFineInfNeighbors(
            std::max(0, static_cast<int>(std::floor(safe_corridor_radius / config.resolution))));
        // CorridorGenerator receives physical virtual planes and adds its
        // radius once while constructing center-position CIRI bounds.  ROG's
        // effective inflated bounds remain exclusively in the A* map.
        const auto& map_config = super_map_->getMapConfig();
        super_corridor_ = std::make_shared<super_planner::CorridorGenerator>(
            super_ros_, super_map_, config.corridor_bound_dis, full_smooth_seed_line_length,
            config.resolution, map_config.virtual_ground_height_raw,
            map_config.virtual_ceil_height_raw,
            safe_corridor_radius, config.obs_skip_num, config.iris_iter_num);
        super_corridor_->SetLineNeighborList(config.seed_line_neighbour);
        SuperPath raw_obstacles;
        raw_obstacles.reserve(planning_map_->size());
        for (const auto& point : planning_map_->points) {
            raw_obstacles.emplace_back(point.x, point.y, point.z);
        }
        super_corridor_->SetStaticObstacleCloud(raw_obstacles);
        // Keep a small numerical/tracking margin below this node's hard
        // validator limit.  SUPER's soft velocity penalty can otherwise land a
        // few mm/s above its configured bound and reject an otherwise safe path.
        auto corridor_opt_config = config.exp_traj_cfg;
        // The config is also used by the traditional SUPER workflow, where a
        // waypoint formulation may be intentional.  This node, however,
        // promises a globally safe PCD trajectory: use the polytope
        // parameterization so every MINCO internal point remains inside the
        // CIRI safe corridor instead of merely being attracted to its guide.
        corridor_opt_config.pos_constraint_type = traj_opt::CORRIDOR;
        // A MINCO piece can bow between its internal points.  Sample the
        // corridor penalty more densely and make it dominant, instead of
        // globally inflating the corridor and unnecessarily closing unrelated
        // narrow but physically valid passages.
        corridor_opt_config.integral_reso = std::max(corridor_opt_config.integral_reso, 40);
        // Corridor planes are penalties in the upstream optimizer rather than
        // exact algebraic constraints.  A high weight is required for the
        // guide-floor plane: at a low observation stop, even a visually small
        // downward bow is undesirable although it remains collision-free.
        corridor_opt_config.penna_pos = std::max(corridor_opt_config.penna_pos, 5.0e9);
        corridor_opt_config.max_vel = std::min(corridor_opt_config.max_vel,
                                               std::max(0.2, max_velocity_ - 0.05));
        super_optimizer_ = std::make_shared<traj_opt::ExpTrajOpt>(corridor_opt_config, super_ros_);
        ROS_INFO("[FULL_SMOOTH] SUPER safe-corridor backend initialized from %s with %zu raw PCD obstacles and %zu regional-ceiling points, corridor radius %.3f m, seed length %.3f m (hard clearance %.3f m, extra margin %.3f m; %s map input)",
                 super_config_path_.c_str(), known_map_->size(),
                 regional_ceiling_constraint_points_,
                 safe_corridor_radius, full_smooth_seed_line_length,
                 collision_clearance_, super_corridor_extra_margin_,
                 super_static_map_only_ ? "static PCD only" : "live ROS callbacks enabled");
    }

    bool buildSuperCorridorPhase(
        const Eigen::Vector3d& phase_start,
        const std::vector<SmoothRoutePoint,
                          Eigen::aligned_allocator<SmoothRoutePoint>>& guides,
        MincoTrajectory& output, const int phase_index) {
        initializeSuperCorridor(phase_start);
        SuperPath guide_path;
        guide_path.emplace_back(phase_start.x(), phase_start.y(), phase_start.z());
        SuperVec3 segment_start = guide_path.back();
        const int search_flags = path_search::ON_INF_MAP | path_search::UNKNOWN_AS_FREE |
                                 path_search::DONT_USE_INF_NEIGHBOR;
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> search_goals;
        search_goals.reserve(local_collision_guides_.size() + guides.size());
        search_goals.insert(search_goals.end(), local_collision_guides_.begin(),
                            local_collision_guides_.end());
        for (const auto& guide : guides) {
            search_goals.push_back(guide.position);
        }
        for (const auto& guide_position : search_goals) {
            const SuperVec3 goal(guide_position.x(), guide_position.y(), guide_position.z());
            if (super_use_recorded_guide_without_astar_) {
                if (!rawPcdSegmentIsSafe(segment_start, goal)) {
                    recordFailureLine(
                        Eigen::Vector3d(segment_start.x(), segment_start.y(), segment_start.z()),
                        guide_position, phase_index,
                        "recorded guide segment violates raw-PCD clearance");
                    ROS_WARN("[FULL_SMOOTH] recorded guide segment violates raw-PCD clearance");
                    return false;
                }
                if ((goal - guide_path.back()).norm() > 1.0e-6) {
                    guide_path.push_back(goal);
                }
                segment_start = goal;
                continue;
            }
            SuperPath segment;
            const auto code = super_astar_->pointToPointPathSearch(
                segment_start, goal, search_flags, 9999.0, segment, super_astar_timeout_);
            if (code != super_utils::REACH_GOAL || segment.empty()) {
                recordFailureLine(Eigen::Vector3d(segment_start.x(), segment_start.y(), segment_start.z()),
                                  guide_position, phase_index,
                                  "A* cannot connect this recorded guide point safely");
                ROS_WARN("[FULL_SMOOTH] SUPER A* cannot safely connect a recorded guide point (code=%d)",
                         static_cast<int>(code));
                return false;
            }

            SuperPath exact_segment;
            exact_segment.push_back(segment_start);
            for (const auto& point : segment) {
                if ((point - exact_segment.back()).norm() > 1.0e-6) {
                    exact_segment.push_back(point);
                }
            }
            if ((goal - exact_segment.back()).norm() > 1.0e-6) {
                exact_segment.push_back(goal);
            }
            bool exact_segment_safe = true;
            for (std::size_t index = 1; index < exact_segment.size(); ++index) {
                if (!rawPcdSegmentIsSafe(exact_segment[index - 1], exact_segment[index])) {
                    exact_segment_safe = false;
                    break;
                }
            }
            if (!exact_segment_safe) {
                recordFailurePath(exact_segment, phase_index,
                                  "A* endpoint connector violates raw-PCD clearance");
                ROS_WARN("[FULL_SMOOTH] SUPER A* grid path has an unsafe raw-PCD endpoint connector");
                return false;
            }
            for (std::size_t index = 1; index < exact_segment.size(); ++index) {
                guide_path.push_back(exact_segment[index]);
            }
            segment_start = goal;
        }
        if (guide_path.size() < 2) {
            recordFailureLine(phase_start, guides.back().position, phase_index,
                              "safe-corridor guide path has fewer than two points");
            return false;
        }
        ROS_INFO("[FULL_SMOOTH] A* guide path raw-PCD clearance: %.3f m",
                 minimumRawPcdClearance(guide_path));
        geometry_utils::PolytopeVec corridor;
        SuperVec3 shifted_start;
        bool ascending_guide = guide_tracking_enabled_;
        for (std::size_t i = 1; i < guide_path.size(); ++i) {
            ascending_guide = ascending_guide && guide_path[i].z() >= guide_path[i-1].z() - 1e-8;
        }
        bool corridor_ready = true;
        if (ascending_guide) {
            // Preserve a contiguous seed chain. SearchPolytopeOnPath prunes
            // intervening polytopes on the basis of their ORIGINAL overlap;
            // that overlap need not survive adding guide tracking planes.
            std::size_t first = 0;
            for (std::size_t i = 1; i < guide_path.size(); ++i) {
                if ((guide_path[i] - guide_path[first]).norm() < 0.40 &&
                    i + 1 < guide_path.size()) continue;
                super_utils::Line line(guide_path[first], guide_path[i]);
                geometry_utils::Polytope poly;
                if (!super_corridor_->GeneratePolytopeFromLine(line, poly)) {
                    corridor_ready = false;
                    break;
                }
                corridor.push_back(poly);
                first = i;
            }
        } else {
            corridor_ready = super_corridor_->SearchPolytopeOnPath(
                guide_path, corridor, shifted_start, false);
        }
        if (!corridor_ready ||
            corridor.empty()) {
            recordFailurePath(guide_path, phase_index,
                              "CIRI cannot inflate this guide path to the required safety clearance");
            ROS_WARN("[FULL_SMOOTH] SUPER cannot construct a continuous safe corridor");
            return false;
        }
        if (!tightenNarrowCorridors(guide_path, corridor, phase_index)) {
            recordFailurePath(guide_path, phase_index,
                              "automatic narrow-corridor constraints are infeasible");
            return false;
        }
        double vertical_guide_floor = -std::numeric_limits<double>::infinity();
        if (!applyVerticalGuideFloor(guide_path, corridor, phase_index,
                                     vertical_guide_floor)) {
            recordFailurePath(guide_path, phase_index,
                              "vertical guide floor constraints are infeasible");
            return false;
        }
        if (!applyGuideTrackingEnvelope(guide_path, corridor, phase_index)) {
            recordFailurePath(guide_path, phase_index,
                              "guide tracking envelope constraints are infeasible");
            return false;
        }
        std::vector<double> guide_times;
        guide_times.reserve(guide_path.size());
        guide_times.push_back(0.0);
        double elapsed = 0.0;
        const double requested_speed = std::max(0.2, guides.back().speed);
        for (std::size_t index = 1; index < guide_path.size(); ++index) {
            // A* returns a dense voxel-by-voxel polyline.  min_segment_time_
            // belongs to recorded waypoint segments; applying it to every
            // 0.1 m grid edge stretches a normal route to hundreds of seconds
            // and gives MINCO a badly conditioned geometric initialization.
            elapsed += std::max(1.0e-3,
                                static_cast<double>((guide_path[index] - guide_path[index - 1]).norm()) /
                                    requested_speed);
            guide_times.push_back(elapsed);
        }
        super_utils::StatePVAJ head = super_utils::StatePVAJ::Zero();
        super_utils::StatePVAJ tail = super_utils::StatePVAJ::Zero();
        head.col(0) = guide_path.front();
        tail.col(0) = guide_path.back();
        geometry_utils::Trajectory trajectory;
        super_optimizer_->setMinimumVerticalVelocity(ascending_guide ? 0.0 : -1.0e6);
        if (!super_optimizer_->optimize(head, tail, guide_path, guide_times, corridor, trajectory)) {
            recordFailurePath(guide_path, phase_index,
                              "corridor-constrained MINCO optimization failed");
            ROS_WARN("[FULL_SMOOTH] SUPER corridor-constrained MINCO optimization failed");
            return false;
        }
        const double corridor_violation = maximumCorridorViolation(trajectory, corridor);
        ROS_INFO("[FULL_SMOOTH] MINCO maximum normalized CIRI/tube-plane violation: %.6f m",
                 corridor_violation);
        if (narrow_corridor_enabled_ && !narrow_corridor_diagnostics_.empty() &&
            corridor_violation > narrow_corridor_max_plane_violation_) {
            ROS_ERROR("[FULL_SMOOTH] optimized MINCO exceeds automatic narrow-corridor tube by %.6f m (limit %.6f m)",
                      corridor_violation, narrow_corridor_max_plane_violation_);
            recordFailurePath(guide_path, phase_index,
                              "optimized MINCO violates automatic narrow-corridor tube");
            return false;
        }
        if (guide_tracking_enabled_ &&
            corridor_violation > guide_tracking_max_plane_violation_) {
            ROS_ERROR("[FULL_SMOOTH] optimized MINCO exceeds guide tracking envelope by %.6f m (limit %.6f m)",
                      corridor_violation, guide_tracking_max_plane_violation_);
            recordFailurePath(guide_path, phase_index,
                              "optimized MINCO violates guide tracking envelope");
            return false;
        }
        if (vertical_guide_floor_enabled_) {
            const double minimum_height = minimumTrajectoryHeight(trajectory);
            ROS_INFO("[FULL_SMOOTH] MINCO phase %d vertical guide floor %.3f m, sampled minimum z %.3f m",
                     phase_index, vertical_guide_floor, minimum_height);
            if (minimum_height <
                vertical_guide_floor - vertical_guide_floor_max_violation_) {
                ROS_ERROR("[FULL_SMOOTH] optimized MINCO undershoots vertical guide floor by %.6f m (limit %.6f m)",
                          vertical_guide_floor - minimum_height,
                          vertical_guide_floor_max_violation_);
                recordFailurePath(guide_path, phase_index,
                                  "optimized MINCO violates vertical guide floor");
                return false;
            }
        }
        output.setCorridorTrajectory(trajectory);
        ROS_INFO("[FULL_SMOOTH] SUPER A* + %zu safe corridors + constrained MINCO: %.2f s",
                 corridor.size(), trajectory.getTotalDuration());
        return true;
    }

    enum class State { WAIT_TRIGGER, ACTIVE, DWELL, COMPLETE, BLOCKED };

    void odomCallback(const nav_msgs::OdometryConstPtr& msg) {
        have_odom_ = true;
        odom_time_ = ros::Time::now().toSec();
        position_ = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y,
                                    msg->pose.pose.position.z);
        const auto& q = msg->pose.pose.orientation;
        yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        // In real flight (trigger type 3), the node is normally started while the
        // vehicle is still on the floor.  Build from the post-takeoff pose instead
        // of permanently rejecting that ground pose as a PCD collision.
        if (!wait_for_reload_ && route_loaded_ && !preview_built_ &&
            (start_trigger_type_ != 3 || trigger_received_)) {
            try {
                buildAndValidate(position_, yaw_);
            } catch (const std::exception& error) {
                handleBuildFailure(error.what());
            }
        }
    }

    void triggerCallback(const geometry_msgs::PoseStampedConstPtr&) {
        if (state_ != State::WAIT_TRIGGER || !have_odom_ ||
            ros::Time::now().toSec() - odom_time_ > 0.15 || !position_.allFinite()) {
            ROS_ERROR("[FULL_SMOOTH] trigger rejected: not waiting or invalid/stale odometry");
            return;
        }
        if (execute_saved_trajectory_ && saved_trajectory_loaded_) {
            const double position_error = (position_ - saved_header_.start_position).norm();
            const double yaw_error = std::abs(angleDifference(saved_header_.start_yaw, yaw_));
            if (position_error > start_position_tolerance_ || yaw_error > start_yaw_tolerance_) {
                trajectory_safe_ = false;
                state_ = State::BLOCKED;
                ROS_ERROR("[FULL_SMOOTH] saved trajectory start mismatch: position %.3f m (limit %.3f), yaw %.1f deg (limit %.1f); no replanning performed",
                          position_error, start_position_tolerance_, yaw_error * 180.0 / M_PI,
                          start_yaw_tolerance_ * 180.0 / M_PI);
                return;
            }
            trigger_received_ = true;
            ROS_INFO("[FULL_SMOOTH] saved trajectory start matched: position %.3f m, yaw %.1f deg; executing without replanning",
                     position_error, yaw_error * 180.0 / M_PI);
            return;
        }
        if (route_loaded_ && !route_.empty()) {
            const double position_error =
                (position_ - route_.front().position).norm();
            if (position_error > start_position_tolerance_) {
                trajectory_safe_ = false;
                state_ = State::BLOCKED;
                ROS_ERROR("[FULL_SMOOTH] route start mismatch: position %.3f m (limit %.3f); no trajectory started",
                          position_error, start_position_tolerance_);
                return;
            }
            ROS_INFO("[FULL_SMOOTH] route start matched: position %.3f m (limit %.3f)",
                     position_error, start_position_tolerance_);
        }
        trigger_received_ = true;
        if (!wait_for_reload_ && route_loaded_ && have_odom_ && !preview_built_) {
            try {
                buildAndValidate(position_, yaw_);
            } catch (const std::exception& error) {
                handleBuildFailure(error.what());
            }
        }
    }

    void reloadCallback(const std_msgs::EmptyConstPtr&) {
        std::string route_path;
        std::string map_path;
        if (!nh_.getParam(route_param_, route_path) || !nh_.getParam(map_param_, map_path)) {
            ROS_ERROR("[FULL_SMOOTH] preview reload ignored: recorder output paths are unavailable");
            return;
        }
        try {
            clearFailureDiagnostic();
            if (save_generated_trajectory_) {
                // A failed preview must not leave an older executable trajectory
                // that could accidentally be selected for this recording session.
                std::remove(saved_trajectory_path_.c_str());
            }
            loadRouteAndMap(route_path, map_path);
            if (preview_only_ && preview_start_at_first_route_point_) {
                buildPreviewFromFirstRoutePoint();
            } else if (have_odom_) {
                buildAndValidate(position_, yaw_);
            }
        } catch (const std::exception& error) {
            handleBuildFailure(error.what(), "preview reload failed");
        }
    }

    void handleBuildFailure(const std::string& error,
                            const std::string& context = "trajectory build failed") {
        trajectory_safe_ = false;
        preview_built_ = true;
        state_ = State::BLOCKED;
        publishFailureVisualization(error);
        ROS_ERROR("[FULL_SMOOTH] %s: %s", context.c_str(), error.c_str());
    }

    void buildPreviewFromFirstRoutePoint() {
        if (route_.empty()) {
            throw std::runtime_error("cannot build a preview from an empty route");
        }
        const Eigen::Vector3d preview_start = route_.front().position;
        double preview_yaw = std::isfinite(route_.front().yaw) ? route_.front().yaw : 0.0;
        if (!std::isfinite(route_.front().yaw)) {
            for (std::size_t index = 1; index < route_.size(); ++index) {
                const Eigen::Vector2d direction =
                    (route_[index].position - preview_start).head<2>();
                if (direction.norm() > 1.0e-4) {
                    preview_yaw = std::atan2(direction.y(), direction.x());
                    break;
                }
            }
        }
        ROS_INFO("[FULL_SMOOTH] preview starts at recorded route point 1, not the current UAV pose");
        buildAndValidate(preview_start, preview_yaw, true);
    }

    void loadRouteAndMap(const std::string& route_path, const std::string& map_path) {
        if (map_path.empty()) {
            throw std::runtime_error("known_map_pcd is required; pass the scanned PCD in the current world frame");
        }
        SmoothRoute loaded_route = LoadSmoothRoute(route_path, recorded_waypoint_speed_);
        pcl::PointCloud<pcl::PointXYZ>::Ptr loaded_map(new pcl::PointCloud<pcl::PointXYZ>());
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, *loaded_map) != 0 ||
            loaded_map->empty()) {
            throw std::runtime_error("cannot load known-map PCD: " + map_path);
        }
        route_path_ = route_path;
        known_map_pcd_ = map_path;
        route_ = std::move(loaded_route);
        known_map_ = loaded_map;
        planning_map_.reset(new pcl::PointCloud<pcl::PointXYZ>(*loaded_map));
        appendRegionalCeilingConstraints(*planning_map_);
        map_tree_.setInputCloud(known_map_);
        phases_.clear();
        super_optimizer_.reset();
        super_corridor_.reset();
        super_astar_.reset();
        super_map_.reset();
        super_ros_.reset();
        local_collision_replan_attempt_ = 0;
        local_collision_guides_.clear();
        saved_trajectory_loaded_ = false;
        preview_built_ = false;
        trajectory_safe_ = false;
        route_loaded_ = true;
        state_ = State::WAIT_TRIGGER;
        ROS_INFO("[FULL_SMOOTH] loaded %zu route points, %zu original PCD points and %zu planning-only regional-ceiling points",
                 route_.size(), known_map_->size(), regional_ceiling_constraint_points_);
    }

    static double angleDifference(const double target, const double source) {
        return std::atan2(std::sin(target - source), std::cos(target - source));
    }

    static void quinticBlend(const double normalized_time, double& value,
                             double& derivative) {
        const double s = std::max(0.0, std::min(normalized_time, 1.0));
        value = 10.0 * s * s * s - 15.0 * s * s * s * s +
                6.0 * s * s * s * s * s;
        derivative = 30.0 * s * s - 60.0 * s * s * s +
                     30.0 * s * s * s * s;
    }

    double rawMotionHeading(const Phase& phase, const double elapsed) const {
        const double duration = phase.trajectory.duration();
        const double t = std::max(0.0, std::min(elapsed, duration));
        const double before = std::max(0.0, t - yaw_lookahead_time_);
        const double after = std::min(duration, t + yaw_lookahead_time_);
        const auto before_sample = phase.trajectory.sample(before);
        const auto after_sample = phase.trajectory.sample(after);
        const auto current_sample = phase.trajectory.sample(t);
        Eigen::Vector2d direction =
            (after_sample.position - before_sample.position).head<2>();

        if (direction.norm() < yaw_velocity_threshold_ * std::max(after - before, 0.01)) {
            // Horizontal velocity is numerically ill-conditioned while climbing or
            // descending almost vertically. Use the stronger one-sided route chord
            // instead of atan2() on a near-zero instantaneous velocity.
            const Eigen::Vector2d backward_direction =
                (current_sample.position - before_sample.position).head<2>();
            const Eigen::Vector2d forward_direction =
                (after_sample.position - current_sample.position).head<2>();
            direction = forward_direction.norm() >= backward_direction.norm()
                            ? forward_direction : backward_direction;
        }
        if (direction.norm() < 1.0e-6) {
            return phase.start_yaw;
        }
        const double raw_yaw = std::atan2(direction.y(), direction.x());
        return phase.start_yaw + angleDifference(raw_yaw, phase.start_yaw);
    }

    YawSample motionHeading(const Phase& phase, const double elapsed) const {
        const double duration = phase.trajectory.duration();
        const double t = std::max(0.0, std::min(elapsed, duration));
        YawSample result;
        result.yaw = rawMotionHeading(phase, t);
        const double derivative_dt = 0.02;
        const double before = std::max(0.0, t - derivative_dt);
        const double after = std::min(duration, t + derivative_dt);
        if (after - before > 1.0e-6) {
            result.rate = angleDifference(rawMotionHeading(phase, after),
                                          rawMotionHeading(phase, before)) /
                          (after - before);
        }
        return result;
    }

    YawSample desiredYaw(const Phase& phase, const double elapsed) const {
        const double duration = phase.trajectory.duration();
        const double t = std::max(0.0, std::min(elapsed, duration));
        YawSample result = motionHeading(phase, t);

        if (phase.start_blend_duration > 1.0e-6 && t < phase.start_blend_duration) {
            double blend = 0.0, blend_derivative = 0.0;
            quinticBlend(t / phase.start_blend_duration, blend, blend_derivative);
            blend_derivative /= phase.start_blend_duration;
            const double delta = angleDifference(result.yaw, phase.start_yaw);
            result.rate = blend * result.rate + blend_derivative * delta;
            result.yaw = phase.start_yaw + blend * delta;
        }

        const double terminal_start = duration - phase.terminal_blend_duration;
        if (phase.terminal_blend_duration > 1.0e-6 && t > terminal_start) {
            double blend = 0.0, blend_derivative = 0.0;
            quinticBlend((t - terminal_start) / phase.terminal_blend_duration,
                         blend, blend_derivative);
            blend_derivative /= phase.terminal_blend_duration;
            const double delta = angleDifference(phase.target_yaw, result.yaw);
            result.rate = (1.0 - blend) * result.rate + blend_derivative * delta;
            result.yaw += blend * delta;
        }
        for (const auto& lock : phase.yaw_locks) {
            if (t < lock.start_time || t > lock.end_time) {
                continue;
            }
            const double lock_duration = lock.end_time - lock.start_time;
            const double ratio = lock_duration > 1.0e-6
                                     ? (t - lock.start_time) / lock_duration : 1.0;
            const double delta = angleDifference(lock.end_yaw, lock.start_yaw);
            result.yaw = lock.start_yaw + ratio * delta;
            result.rate = lock_duration > 1.0e-6 ? delta / lock_duration : 0.0;
            break;
        }
        return result;
    }

    void buildYawProfile(Phase& phase) const {
        const double duration = phase.trajectory.duration();
        const int count = std::max(1, static_cast<int>(std::ceil(duration / 0.01)));
        phase.yaw_profile_dt = duration / count;
        std::vector<double> desired(count + 1, phase.start_yaw);
        for (int i = 1; i <= count; ++i) {
            const double candidate = desiredYaw(phase, i * phase.yaw_profile_dt).yaw;
            desired[i] = desired[i - 1] + angleDifference(candidate, desired[i - 1]);
        }

        // Choose the equivalent terminal angle closest to the direction-following
        // reference, then track that reference under an explicit yaw-rate bound.
        const double target = desired[count - 1] +
                              angleDifference(phase.target_yaw, desired[count - 1]);
        const double step_limit = 0.9 * max_yaw_rate_ * phase.yaw_profile_dt;
        if (std::abs(target - phase.start_yaw) > count * step_limit + 1.0e-9) {
            throw std::runtime_error("phase is too short to satisfy its terminal yaw-rate limit");
        }

        phase.yaw_profile.resize(count + 1);
        phase.yaw_profile.front() = phase.start_yaw;
        for (int i = 1; i <= count; ++i) {
            const double remaining_limit = (count - i) * step_limit;
            const double lower = std::max(phase.yaw_profile[i - 1] - step_limit,
                                          target - remaining_limit);
            const double upper = std::min(phase.yaw_profile[i - 1] + step_limit,
                                          target + remaining_limit);
            phase.yaw_profile[i] = std::max(lower, std::min(desired[i], upper));
        }
        phase.yaw_profile.back() = target;
    }

    YawSample commandYaw(const Phase& phase, const double elapsed) const {
        if (phase.yaw_profile.size() < 2 || phase.yaw_profile_dt <= 0.0) {
            return desiredYaw(phase, elapsed);
        }
        const double t = std::max(0.0, std::min(elapsed, phase.trajectory.duration()));
        const double index = t / phase.yaw_profile_dt;
        const int lower = std::min(static_cast<int>(index),
                                   static_cast<int>(phase.yaw_profile.size()) - 2);
        const double alpha = std::max(0.0, std::min(index - lower, 1.0));
        YawSample result;
        result.yaw = (1.0 - alpha) * phase.yaw_profile[lower] +
                     alpha * phase.yaw_profile[lower + 1];
        result.rate = (phase.yaw_profile[lower + 1] - phase.yaw_profile[lower]) /
                      phase.yaw_profile_dt;
        return result;
    }

    bool trajectoryMeetsSpatialLimits(const MincoTrajectory& trajectory) const {
        if (trajectory.maximumJerkDiscontinuity() > max_jerk_discontinuity_) {
            return false;
        }
        const int count = std::max(
            1, static_cast<int>(std::ceil(trajectory.duration() / sample_dt_)));
        for (int i = 0; i <= count; ++i) {
            const auto sample = trajectory.sample(trajectory.duration() * i / count);
            if (!regionalHeightIsSafe(sample.position)) {
                return false;
            }
            pcl::PointXYZ query;
            query.x = sample.position.x();
            query.y = sample.position.y();
            query.z = sample.position.z();
            std::vector<int> index(1);
            std::vector<float> squared_distance(1);
            if (map_tree_.nearestKSearch(query, 1, index, squared_distance) > 0 &&
                std::sqrt(squared_distance.front()) < collision_clearance_) {
                return false;
            }
            if (sample.velocity.norm() > max_velocity_ ||
                sample.acceleration.norm() > max_acceleration_ ||
                sample.jerk.norm() > max_jerk_ || sample.snap.norm() > max_snap_) {
                return false;
            }
        }
        return true;
    }

    bool canRelaxPassPoint(
        const Eigen::Vector3d& phase_start,
        const std::vector<SmoothRoutePoint,
                          Eigen::aligned_allocator<SmoothRoutePoint>>& guides,
        const double phase_start_yaw, const double phase_target_yaw) const {
        try {
            std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> positions;
            std::vector<double> speeds;
            positions.reserve(guides.size());
            speeds.reserve(guides.size());
            for (const auto& guide : guides) {
                positions.push_back(guide.position);
                speeds.push_back(guide.speed);
            }
            MincoTrajectory candidate;
            candidate.build(phase_start, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                            Eigen::Vector3d::Zero(), positions, speeds, min_segment_time_);
            const double yaw_rate_margin = std::max(0.1, 0.9 * max_yaw_rate_);
            if (std::abs(angleDifference(phase_target_yaw, phase_start_yaw)) >
                candidate.duration() * yaw_rate_margin) {
                return false;
            }
            return trajectoryMeetsSpatialLimits(candidate);
        } catch (const std::exception&) {
            return false;
        }
    }

    void buildAndValidate(const Eigen::Vector3d& start, const double start_yaw,
                          const bool skip_first_route_point = false) {
        phases_.clear();
        clearFailureDiagnostic();
        narrow_corridor_report_.clear();
        Eigen::Vector3d phase_start = start;
        double phase_start_yaw = start_yaw;
        std::vector<SmoothRoutePoint, Eigen::aligned_allocator<SmoothRoutePoint>> guides;
        int relaxed_pass_points = 0;
        int unsafe_relaxations_rejected = 0;
        const std::size_t first_route_index = skip_first_route_point ? 1 : 0;
        for (std::size_t route_index = first_route_index; route_index < route_.size(); ++route_index) {
            const auto& point = route_[route_index];
            guides.push_back(point);
            if (point.dwell <= 0.0) {
                continue;
            }
            if (std::isnan(point.yaw)) {
                throw std::runtime_error("every full-smooth phase endpoint needs a finite yaw");
            }

            // Legacy six-column routes remain exact. New recorder routes label
            // ordinary samples as `pass`; remove such a knot only when the full
            // candidate remains clear of the known map and within motion limits.
            for (std::size_t index = 0; index + 1 < guides.size();) {
                if (!guides[index].soft_pass) {
                    ++index;
                    continue;
                }
                auto candidate = guides;
                candidate.erase(candidate.begin() + static_cast<long>(index));
                const Eigen::Vector3d shortcut_start =
                    index == 0 ? phase_start : candidate[index - 1].position;
                const Eigen::Vector3d shortcut_end = candidate[index].position;
                const bool shortcut_safe =
                    !use_super_safe_corridor_ ||
                    rawPcdSegmentIsSafe(shortcut_start, shortcut_end);
                if (shortcut_safe &&
                    canRelaxPassPoint(phase_start, candidate, phase_start_yaw, point.yaw)) {
                    guides.swap(candidate);
                    ++relaxed_pass_points;
                } else {
                    if (!shortcut_safe) {
                        ++unsafe_relaxations_rejected;
                    }
                    ++index;
                }
            }

            std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> positions;
            std::vector<double> speeds;
            std::vector<double> guide_yaws;
            positions.reserve(guides.size());
            speeds.reserve(guides.size());
            guide_yaws.reserve(guides.size());
            for (const auto& guide : guides) {
                positions.push_back(guide.position);
                speeds.push_back(guide.speed);
                guide_yaws.push_back(guide.yaw);
            }
            Phase phase;
            if (use_super_safe_corridor_) {
                if (!buildSuperCorridorPhase(phase_start, guides, phase.trajectory,
                                             static_cast<int>(phases_.size()) + 1)) {
                    throw std::runtime_error(
                        "SUPER cannot find a safe corridor-constrained trajectory for this route phase");
                }
            } else {
                phase.trajectory.build(phase_start, Eigen::Vector3d::Zero(),
                                       Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                       positions, speeds, min_segment_time_);
            }
            phase.start_yaw = phase_start_yaw;
            phase.target_yaw = point.yaw;
            phase.dwell = point.dwell;
            // A corridor trajectory has one polynomial piece per generated safe
            // corridor, not per recorded guide point.  Its knot array therefore
            // cannot be indexed with the guide-point index.  Associate an
            // explicitly headed intermediate guide with its nearest point on the
            // optimized trajectory instead; ordinary PASS points use yaw=nan.
            const auto nearest_trajectory_time = [&](const Eigen::Vector3d& guide_position,
                                                     const double earliest_time) {
                const int sample_count = std::max(
                    1, static_cast<int>(std::ceil(phase.trajectory.duration() /
                                                   std::min(sample_dt_, 0.03))));
                double best_time = earliest_time;
                double best_distance = std::numeric_limits<double>::infinity();
                for (int sample_index = 0; sample_index <= sample_count; ++sample_index) {
                    const double time = phase.trajectory.duration() * sample_index / sample_count;
                    if (time + 1.0e-6 < earliest_time) {
                        continue;
                    }
                    const double distance =
                        (phase.trajectory.sample(time).position - guide_position).squaredNorm();
                    if (distance < best_distance) {
                        best_distance = distance;
                        best_time = time;
                    }
                }
                return best_time;
            };
            std::vector<std::pair<double, double>> yaw_anchors;
            double earliest_yaw_anchor = 0.0;
            for (std::size_t i = 0; i + 1 < guide_yaws.size(); ++i) {
                if (std::isfinite(guide_yaws[i])) {
                    const double anchor_time = nearest_trajectory_time(
                        guides[i].position, earliest_yaw_anchor);
                    yaw_anchors.emplace_back(anchor_time, guide_yaws[i]);
                    earliest_yaw_anchor = anchor_time;
                }
            }
            for (std::size_t i = 1; i < yaw_anchors.size(); ++i) {
                phase.yaw_locks.push_back({yaw_anchors[i - 1].first,
                                           yaw_anchors[i].first,
                                           yaw_anchors[i - 1].second,
                                           yaw_anchors[i].second});
            }
            const double yaw_rate_margin = std::max(0.1, 0.9 * max_yaw_rate_);
            const YawSample initial_heading = motionHeading(phase, 0.0);
            const YawSample terminal_heading =
                motionHeading(phase, phase.trajectory.duration());
            phase.start_blend_duration = std::max(
                yaw_start_blend_duration_,
                1.875 * std::abs(angleDifference(initial_heading.yaw, phase.start_yaw)) /
                    yaw_rate_margin);
            phase.terminal_blend_duration = std::max(
                yaw_terminal_blend_duration_,
                1.875 * std::abs(angleDifference(phase.target_yaw, terminal_heading.yaw)) /
                    yaw_rate_margin);
            buildYawProfile(phase);
            phases_.push_back(phase);
            phase_start = point.position;
            phase_start_yaw = point.yaw;
            guides.clear();
        }

        ROS_INFO("[FULL_SMOOTH] relaxed %d optional pass-point constraints", relaxed_pass_points);
        ROS_INFO("[FULL_SMOOTH] retained %d pass points because shortcut raw-PCD clearance was unsafe",
                 unsafe_relaxations_rejected);

        minimum_clearance_ = std::numeric_limits<double>::infinity();
        certified_clearance_lower_bound_ = std::numeric_limits<double>::infinity();
        maximum_velocity_ = maximum_acceleration_ = maximum_jerk_ = maximum_snap_seen_ = 0.0;
        maximum_jerk_discontinuity_seen_ = maximum_yaw_rate_seen_ = 0.0;
        maximum_yaw_lock_variation_seen_ = 0.0;
        maximum_regional_height_seen_ = -std::numeric_limits<double>::infinity();
        int maximum_yaw_rate_phase = 0;
        double maximum_yaw_rate_time = 0.0;
        int minimum_clearance_phase = 0;
        int yaw_alignment_samples = 0;
        int yaw_alignment_within_45_deg = 0;
        double yaw_alignment_error_sum = 0.0;
        bool collision = false;
        for (int phase_index = 0; phase_index < static_cast<int>(phases_.size());
             ++phase_index) {
            const auto& phase = phases_[phase_index];
            maximum_jerk_discontinuity_seen_ = std::max(
                maximum_jerk_discontinuity_seen_,
                phase.trajectory.maximumJerkDiscontinuity());
            for (const auto& lock : phase.yaw_locks) {
                const int lock_count = std::max(
                    1, static_cast<int>(std::ceil(
                           (lock.end_time - lock.start_time) / sample_dt_)));
                double previous_yaw = commandYaw(phase, lock.start_time).yaw;
                double lock_variation = 0.0;
                for (int i = 1; i <= lock_count; ++i) {
                    const double t = lock.start_time +
                                     (lock.end_time - lock.start_time) * i / lock_count;
                    const double yaw = commandYaw(phase, t).yaw;
                    lock_variation += std::abs(yaw - previous_yaw);
                    previous_yaw = yaw;
                }
                maximum_yaw_lock_variation_seen_ = std::max(
                    maximum_yaw_lock_variation_seen_, lock_variation);
            }
            const double exact_max_speed =
                phase.trajectory.rawTrajectory().getMaxVelRate();
            const double collision_dt = exact_max_speed > 1.0e-6
                ? collision_validation_step_ / exact_max_speed
                : sample_dt_;
            const int count = std::max(1, static_cast<int>(
                std::ceil(phase.trajectory.duration() /
                          std::min(sample_dt_, collision_dt))));
            // The distance-to-point-cloud function is 1-Lipschitz.  This
            // reserve converts spatial samples into a lower bound for every
            // point between samples, using the polynomial's exact peak speed.
            const double clearance_reserve = 0.5 * exact_max_speed *
                phase.trajectory.duration() / count;
            for (int i = 0; i <= count; ++i) {
                const double t = phase.trajectory.duration() * i / count;
                const auto sample = phase.trajectory.sample(t);
                if (regionalCeilingRegion(sample.position) >= 0) {
                    maximum_regional_height_seen_ =
                        std::max(maximum_regional_height_seen_, sample.position.z());
                }
                if (!regionalHeightIsSafe(sample.position, clearance_reserve)) {
                    collision = true;
                    const double vertical_clearance = regional_ceiling_z_ - sample.position.z();
                    if (!failure_diagnostic_.valid ||
                        vertical_clearance < failure_diagnostic_.clearance) {
                        failure_diagnostic_.valid = true;
                        failure_diagnostic_.phase_index = phase_index + 1;
                        failure_diagnostic_.segment_start = sample.position;
                        failure_diagnostic_.segment_end = sample.position;
                        failure_diagnostic_.closest_path_point = sample.position;
                        failure_diagnostic_.closest_obstacle = Eigen::Vector3d(
                            sample.position.x(), sample.position.y(), regional_ceiling_z_);
                        failure_diagnostic_.clearance = vertical_clearance;
                        failure_diagnostic_.reason =
                            "MINCO trajectory exceeds the regional height limit";
                    }
                }
                pcl::PointXYZ query;
                query.x = sample.position.x();
                query.y = sample.position.y();
                query.z = sample.position.z();
                std::vector<int> index(1);
                std::vector<float> squared_distance(1);
                if (map_tree_.nearestKSearch(query, 1, index, squared_distance) > 0) {
                    const double distance = std::sqrt(squared_distance.front());
                    certified_clearance_lower_bound_ = std::min(
                        certified_clearance_lower_bound_, distance - clearance_reserve);
                    if (distance < minimum_clearance_) {
                        minimum_clearance_ = distance;
                        minimum_clearance_phase = phase_index + 1;
                        failure_diagnostic_.valid = true;
                        failure_diagnostic_.phase_index = minimum_clearance_phase;
                        failure_diagnostic_.segment_start = sample.position;
                        failure_diagnostic_.segment_end = sample.position;
                        failure_diagnostic_.closest_path_point = sample.position;
                        const auto& obstacle = known_map_->points[index.front()];
                        failure_diagnostic_.closest_obstacle =
                            Eigen::Vector3d(obstacle.x, obstacle.y, obstacle.z);
                        failure_diagnostic_.clearance = distance;
                        failure_diagnostic_.reason = "MINCO trajectory is closer to the validation PCD than the required clearance";
                    }
                    collision = collision ||
                        distance - clearance_reserve < collision_clearance_;
                }
                maximum_velocity_ = std::max(maximum_velocity_, sample.velocity.norm());
                maximum_acceleration_ = std::max(maximum_acceleration_, sample.acceleration.norm());
                maximum_jerk_ = std::max(maximum_jerk_, sample.jerk.norm());
                maximum_snap_seen_ = std::max(maximum_snap_seen_, sample.snap.norm());
                const double yaw_rate = std::abs(commandYaw(phase, t).rate);
                if (yaw_rate > maximum_yaw_rate_seen_) {
                    maximum_yaw_rate_seen_ = yaw_rate;
                    maximum_yaw_rate_phase = phase_index + 1;
                    maximum_yaw_rate_time = t;
                }
                if (sample.velocity.head<2>().norm() >= 0.20) {
                    const double velocity_yaw =
                        std::atan2(sample.velocity.y(), sample.velocity.x());
                    const double yaw_error = std::abs(angleDifference(
                        commandYaw(phase, t).yaw, velocity_yaw));
                    yaw_alignment_error_sum += yaw_error;
                    ++yaw_alignment_samples;
                    if (yaw_error <= M_PI / 4.0) {
                        ++yaw_alignment_within_45_deg;
                    }
                }
            }
        }
        trajectory_safe_ = !collision && maximum_velocity_ <= max_velocity_ &&
                           maximum_acceleration_ <= max_acceleration_ &&
                           maximum_jerk_ <= max_jerk_ &&
                           maximum_snap_seen_ <= max_snap_ &&
                           maximum_jerk_discontinuity_seen_ <= max_jerk_discontinuity_ &&
                           maximum_yaw_rate_seen_ <= max_yaw_rate_ &&
                           maximum_yaw_lock_variation_seen_ <= max_yaw_lock_variation_;
        if (!trajectory_safe_ && collision && use_super_safe_corridor_ &&
            local_collision_replan_enabled_ && failure_diagnostic_.valid &&
            local_collision_replan_attempt_ < local_collision_replan_max_attempts_) {
            ++local_collision_replan_attempt_;
            ROS_WARN("[FULL_SMOOTH] continuous PCD check certified only %.3f m clearance (sampled %.3f m) in phase %d; local replan %d/%d",
                     certified_clearance_lower_bound_, minimum_clearance_, minimum_clearance_phase,
                     local_collision_replan_attempt_, local_collision_replan_max_attempts_);
            addLocalCollisionAvoidance(failure_diagnostic_);
            buildAndValidate(start, start_yaw, skip_first_route_point);
            return;
        }
        if (trajectory_safe_ && save_generated_trajectory_) {
            try {
                writeNarrowCorridorReport();
                writeSavedTrajectory(start, start_yaw);
                ROS_INFO("[FULL_SMOOTH] saved exact executable global trajectory to %s",
                         saved_trajectory_path_.c_str());
            } catch (const std::exception& error) {
                trajectory_safe_ = false;
                ROS_ERROR("[FULL_SMOOTH] trajectory is not executable because saving failed: %s",
                          error.what());
            }
        }
        preview_built_ = true;
        publishVisualization();
        ROS_INFO("[FULL_SMOOTH] peak yaw rate occurs in phase %d at t=%.2f s",
                 maximum_yaw_rate_phase, maximum_yaw_rate_time);
        if (yaw_alignment_samples > 0) {
            ROS_INFO("[FULL_SMOOTH] yaw follows velocity: mean error=%.1f deg, within 45 deg=%.1f%%",
                     yaw_alignment_error_sum / yaw_alignment_samples * 180.0 / M_PI,
                     100.0 * yaw_alignment_within_45_deg / yaw_alignment_samples);
        }
        ROS_INFO("[FULL_SMOOTH] maximum yaw variation inside a locked gate interval: %.1f deg",
                 maximum_yaw_lock_variation_seen_ * 180.0 / M_PI);
        if (trajectory_safe_) {
            ROS_INFO("[FULL_SMOOTH] global MINCO SAFE: continuous clearance>=%.3f m (sampled %.3f m), max v/a/j/snap/yaw_rate=%.3f/%.3f/%.3f/%.3f/%.3f, jerk_jump=%.3g",
                     certified_clearance_lower_bound_, minimum_clearance_, maximum_velocity_, maximum_acceleration_, maximum_jerk_,
                     maximum_snap_seen_, maximum_yaw_rate_seen_,
                     maximum_jerk_discontinuity_seen_);
        } else {
            ROS_ERROR("[FULL_SMOOTH] global MINCO FAILED: continuous clearance>=%.3f (sampled %.3f, need %.3f), max v/a/j/snap/yaw_rate=%.3f/%.3f/%.3f/%.3f/%.3f, jerk_jump=%.3g",
                      certified_clearance_lower_bound_, minimum_clearance_, collision_clearance_, maximum_velocity_,
                      maximum_acceleration_, maximum_jerk_, maximum_snap_seen_,
                      maximum_yaw_rate_seen_, maximum_jerk_discontinuity_seen_);
        }
        if (regional_ceiling_enabled_ && std::isfinite(maximum_regional_height_seen_)) {
            ROS_INFO("[FULL_SMOOTH] regional maximum vehicle-reference height: %.3f m (hard limit %.3f m below %.3f m virtual ceiling)",
                     maximum_regional_height_seen_, regionalMaximumVehicleZ(),
                     regional_ceiling_z_);
        }
    }

    void appendFailureMarkers(visualization_msgs::MarkerArray& markers,
                              const std_msgs::Header& header) const {
        if (!failure_diagnostic_.valid) {
            return;
        }
        visualization_msgs::Marker segment;
        segment.header = header;
        segment.ns = "full_smooth_failure_segment";
        segment.id = 10;
        segment.type = visualization_msgs::Marker::LINE_LIST;
        segment.action = visualization_msgs::Marker::ADD;
        segment.pose.orientation.w = 1.0;
        segment.scale.x = 0.12;
        segment.color.r = 1.0;
        segment.color.a = 1.0;
        segment.points.push_back(toRosPoint(failure_diagnostic_.segment_start));
        segment.points.push_back(toRosPoint(failure_diagnostic_.segment_end));
        markers.markers.push_back(segment);

        visualization_msgs::Marker path_point;
        path_point.header = header;
        path_point.ns = "full_smooth_failure_path_point";
        path_point.id = 11;
        path_point.type = visualization_msgs::Marker::SPHERE;
        path_point.action = visualization_msgs::Marker::ADD;
        path_point.pose.position = toRosPoint(failure_diagnostic_.closest_path_point);
        path_point.pose.orientation.w = 1.0;
        path_point.scale.x = path_point.scale.y = path_point.scale.z = 0.20;
        path_point.color.r = 1.0;
        path_point.color.g = 0.15;
        path_point.color.a = 1.0;
        markers.markers.push_back(path_point);

        visualization_msgs::Marker obstacle_point = path_point;
        obstacle_point.ns = "full_smooth_failure_obstacle";
        obstacle_point.id = 12;
        obstacle_point.pose.position = toRosPoint(failure_diagnostic_.closest_obstacle);
        obstacle_point.scale.x = obstacle_point.scale.y = obstacle_point.scale.z = 0.16;
        obstacle_point.color.r = 1.0;
        obstacle_point.color.g = 0.9;
        obstacle_point.color.b = 0.0;
        markers.markers.push_back(obstacle_point);

        visualization_msgs::Marker text;
        text.header = header;
        text.ns = "full_smooth_failure_text";
        text.id = 13;
        text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::Marker::ADD;
        text.pose.position = toRosPoint(failure_diagnostic_.closest_path_point);
        text.pose.position.z += 0.45;
        text.pose.orientation.w = 1.0;
        text.scale.z = 0.25;
        text.color.r = 1.0;
        text.color.g = 0.85;
        text.color.a = 1.0;
        std::ostringstream message;
        message << "BLOCKED phase " << failure_diagnostic_.phase_index;
        if (std::isfinite(failure_diagnostic_.clearance)) {
            message << ": clearance " << std::fixed << std::setprecision(3)
                    << failure_diagnostic_.clearance << " m (need " << collision_clearance_ << " m)";
        }
        message << "\n" << failure_diagnostic_.reason;
        text.text = message.str();
        markers.markers.push_back(text);
    }

    void publishFailureVisualization(const std::string& error) {
        nav_msgs::Path empty_path;
        empty_path.header.frame_id = "world";
        empty_path.header.stamp = ros::Time::now();
        visualization_msgs::MarkerArray markers;
        visualization_msgs::Marker clear;
        clear.header = empty_path.header;
        clear.action = visualization_msgs::Marker::DELETEALL;
        markers.markers.push_back(clear);
        if (!failure_diagnostic_.valid) {
            failure_diagnostic_.valid = true;
            failure_diagnostic_.phase_index = 0;
            failure_diagnostic_.segment_start = position_;
            failure_diagnostic_.segment_end = position_;
            failure_diagnostic_.closest_path_point = position_;
            failure_diagnostic_.closest_obstacle = position_;
            failure_diagnostic_.reason = error;
        }
        appendFailureMarkers(markers, empty_path.header);
        path_pub_.publish(empty_path);
        marker_pub_.publish(markers);
    }

    void publishVisualization() {
        nav_msgs::Path path;
        path.header.frame_id = "world";
        path.header.stamp = ros::Time::now();
        visualization_msgs::Marker line, envelope, speed_points, yaw_arrows, avoidance_guides, text;
        line.header = envelope.header = speed_points.header = yaw_arrows.header =
            avoidance_guides.header = text.header = path.header;
        line.ns = "full_smooth_centerline";
        line.id = 0;
        line.type = visualization_msgs::Marker::LINE_STRIP;
        line.action = visualization_msgs::Marker::ADD;
        line.pose.orientation.w = 1.0;
        line.scale.x = 0.05;
        line.color.a = 1.0;
        envelope.ns = "full_smooth_safety_envelope";
        envelope.id = 1;
        envelope.type = visualization_msgs::Marker::SPHERE_LIST;
        envelope.action = visualization_msgs::Marker::ADD;
        envelope.pose.orientation.w = 1.0;
        envelope.scale.x = envelope.scale.y = envelope.scale.z = 2.0 * collision_clearance_;
        envelope.color.a = 0.13;
        speed_points.ns = "full_smooth_speed";
        speed_points.id = 2;
        speed_points.type = visualization_msgs::Marker::SPHERE_LIST;
        speed_points.action = visualization_msgs::Marker::ADD;
        speed_points.pose.orientation.w = 1.0;
        speed_points.scale.x = speed_points.scale.y = speed_points.scale.z = 0.12;
        yaw_arrows.ns = "full_smooth_yaw";
        yaw_arrows.id = 3;
        yaw_arrows.type = visualization_msgs::Marker::LINE_LIST;
        yaw_arrows.action = visualization_msgs::Marker::ADD;
        yaw_arrows.pose.orientation.w = 1.0;
        yaw_arrows.scale.x = 0.035;
        yaw_arrows.color.a = 1.0;
        // Cyan arrows are spaced in time along the trajectory.  They show the
        // commanded yaw, including the fixed headings at STOP waypoints.
        yaw_arrows.color.g = 0.95;
        yaw_arrows.color.b = 1.0;
        avoidance_guides.ns = "full_smooth_local_replan_guides";
        avoidance_guides.id = 5;
        avoidance_guides.type = visualization_msgs::Marker::SPHERE_LIST;
        avoidance_guides.action = visualization_msgs::Marker::ADD;
        avoidance_guides.pose.orientation.w = 1.0;
        avoidance_guides.scale.x = avoidance_guides.scale.y = avoidance_guides.scale.z = 0.18;
        avoidance_guides.color.r = 0.8;
        avoidance_guides.color.g = 0.2;
        avoidance_guides.color.b = 1.0;
        avoidance_guides.color.a = 1.0;
        for (const auto& guide : local_collision_guides_) {
            avoidance_guides.points.push_back(toRosPoint(guide));
        }
        if (trajectory_safe_) {
            line.color.g = envelope.color.g = 1.0;
        } else {
            line.color.r = envelope.color.r = 1.0;
        }
        for (const auto& phase : phases_) {
            const int count = std::max(1, static_cast<int>(
                std::ceil(phase.trajectory.duration() / sample_dt_)));
            for (int i = 0; i <= count; ++i) {
                const double t = phase.trajectory.duration() * i / count;
                const auto sample = phase.trajectory.sample(t);
                const auto yaw_sample = commandYaw(phase, t);
                geometry_msgs::PoseStamped pose;
                pose.header = path.header;
                pose.pose.position.x = sample.position.x();
                pose.pose.position.y = sample.position.y();
                pose.pose.position.z = sample.position.z();
                pose.pose.orientation.z = std::sin(0.5 * yaw_sample.yaw);
                pose.pose.orientation.w = std::cos(0.5 * yaw_sample.yaw);
                path.poses.push_back(pose);
                line.points.push_back(pose.pose.position);
                if (i % std::max(1, static_cast<int>(0.08 / sample_dt_)) == 0) {
                    const double ratio = std::max(
                        0.0, std::min(sample.velocity.norm() / std::max(max_velocity_, 1.0e-3), 1.0));
                    std_msgs::ColorRGBA color;
                    // SUPER-style speed heatmap: blue -> cyan -> green -> yellow -> red.
                    color.r = static_cast<float>(std::max(0.0, std::min(2.0 * ratio - 0.5, 1.0)));
                    color.g = static_cast<float>(std::max(0.0, std::min(2.0 - 2.0 * std::abs(ratio - 0.5), 1.0)));
                    color.b = static_cast<float>(std::max(0.0, std::min(1.5 - 2.0 * ratio, 1.0)));
                    color.a = 1.0;
                    speed_points.points.push_back(pose.pose.position);
                    speed_points.colors.push_back(color);
                }
                if (i % std::max(1, static_cast<int>(0.15 / sample_dt_)) == 0) {
                    envelope.points.push_back(pose.pose.position);
                }
                if (i == 0 || i == count ||
                    i % std::max(1, static_cast<int>(0.50 / sample_dt_)) == 0) {
                    const double arrow_length = 0.32;
                    const double arrow_head = 0.11;
                    geometry_msgs::Point base = pose.pose.position;
                    geometry_msgs::Point tip = base;
                    tip.x += arrow_length * std::cos(yaw_sample.yaw);
                    tip.y += arrow_length * std::sin(yaw_sample.yaw);
                    geometry_msgs::Point left = tip;
                    geometry_msgs::Point right = tip;
                    left.x -= arrow_head * std::cos(yaw_sample.yaw - 0.55);
                    left.y -= arrow_head * std::sin(yaw_sample.yaw - 0.55);
                    right.x -= arrow_head * std::cos(yaw_sample.yaw + 0.55);
                    right.y -= arrow_head * std::sin(yaw_sample.yaw + 0.55);
                    yaw_arrows.points.insert(yaw_arrows.points.end(),
                                             {base, tip, tip, left, tip, right});
                }
            }
        }
        text.ns = "full_smooth_status";
        text.id = 4;
        text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::Marker::ADD;
        text.pose.orientation.w = 1.0;
        text.pose.position.x = position_.x();
        text.pose.position.y = position_.y();
        text.pose.position.z = position_.z() + 0.5;
        text.scale.z = 0.25;
        text.color = line.color;
        text.text = trajectory_safe_ ? "FULL MINCO: SAFE" : "FULL MINCO: BLOCKED";
        visualization_msgs::MarkerArray markers;
        visualization_msgs::Marker clear;
        clear.header = path.header;
        clear.action = visualization_msgs::Marker::DELETEALL;
        markers.markers = {clear, line, envelope, speed_points, yaw_arrows, avoidance_guides, text};
        if (!trajectory_safe_) {
            appendFailureMarkers(markers, path.header);
        }
        path_pub_.publish(path);
        marker_pub_.publish(markers);
    }

    quadrotor_msgs::PositionCommand makeCommand(const MincoTrajectory::Sample& sample,
                                                 const double yaw, const double yaw_rate,
                                                 const int phase_id,
                                                 const double time_scale = 1.0) const {
        quadrotor_msgs::PositionCommand cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.header.frame_id = "world";
        cmd.position.x = sample.position.x(); cmd.position.y = sample.position.y(); cmd.position.z = sample.position.z();
        const double acceleration_scale = time_scale * time_scale;
        const double jerk_scale = acceleration_scale * time_scale;
        cmd.velocity.x = sample.velocity.x() * time_scale; cmd.velocity.y = sample.velocity.y() * time_scale; cmd.velocity.z = sample.velocity.z() * time_scale;
        cmd.acceleration.x = sample.acceleration.x() * acceleration_scale; cmd.acceleration.y = sample.acceleration.y() * acceleration_scale; cmd.acceleration.z = sample.acceleration.z() * acceleration_scale;
        cmd.jerk.x = sample.jerk.x() * jerk_scale; cmd.jerk.y = sample.jerk.y() * jerk_scale; cmd.jerk.z = sample.jerk.z() * jerk_scale;
        cmd.yaw = yaw;
        cmd.yaw_dot = yaw_rate * time_scale;
        cmd.vel_norm = sample.velocity.norm() * time_scale;
        cmd.acc_norm = sample.acceleration.norm() * acceleration_scale;
        cmd.trajectory_id = 910000 + phase_id;
        cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
        return cmd;
    }

    void publishSelection(const bool active) {
        std_msgs::Bool selected;
        selected.data = active;
        select_pub_.publish(selected);
    }

    void timerCallback(const ros::TimerEvent&) {
        const double now = ros::Time::now().toSec();
        if (!have_odom_ || now - odom_time_ > 0.5 || !preview_built_) {
            return;
        }
        if (preview_only_) {
            return;
        }
        if (state_ == State::WAIT_TRIGGER) {
            const bool start = start_trigger_type_ == 2 ? now - start_time_ >= start_delay_
                                                       : trigger_received_;
            if (!start) return;
            if (!trajectory_safe_) {
                state_ = State::BLOCKED;
                ROS_ERROR("[FULL_SMOOTH] mission start blocked by known-PCD or dynamics validation");
                return;
            }
            publishSelection(true);
            phase_index_ = 0;
            phase_start_time_ = now;
            state_ = State::ACTIVE;
            ROS_INFO("[FULL_SMOOTH] start %zu preplanned phases", phases_.size());
        }
        if (state_ == State::ACTIVE) {
            const auto& phase = phases_[phase_index_];
            const double trajectory_time = std::min(
                (now - phase_start_time_) * execution_time_scale_,
                phase.trajectory.duration());
            const auto yaw_sample = commandYaw(phase, trajectory_time);
            command_pub_.publish(makeCommand(phase.trajectory.sample(trajectory_time),
                                              yaw_sample.yaw, yaw_sample.rate, phase_index_,
                                              execution_time_scale_));
            if (trajectory_time >= phase.trajectory.duration()) {
                state_ = State::DWELL;
                dwell_start_time_ = now;
                ROS_INFO("[FULL_SMOOTH] phase %d/%zu reached; dwell %.2f s at yaw %.1f deg",
                         phase_index_ + 1, phases_.size(), phase.dwell,
                         phase.target_yaw * 180.0 / M_PI);
            }
            return;
        }
        if (state_ == State::DWELL) {
            const auto& phase = phases_[phase_index_];
            command_pub_.publish(makeCommand(phase.trajectory.sample(phase.trajectory.duration()),
                                              phase.target_yaw, 0.0, phase_index_,
                                              execution_time_scale_));
            if (now - dwell_start_time_ < phase.dwell) return;
            if (++phase_index_ < static_cast<int>(phases_.size())) {
                phase_start_time_ = now;
                state_ = State::ACTIVE;
                return;
            }
            state_ = State::COMPLETE;
            complete_time_ = now;
            publishSelection(false);
            ROS_INFO("[FULL_SMOOTH] Preset mission completed.");
            return;
        }
        if (state_ == State::COMPLETE && auto_land_ && !land_sent_ && now - complete_time_ > 0.3) {
            quadrotor_msgs::TakeoffLand land;
            land.takeoff_land_cmd = quadrotor_msgs::TakeoffLand::LAND;
            land_pub_.publish(land);
            land_sent_ = true;
            ROS_INFO("[FULL_SMOOTH] request PX4Ctrl landing");
        }
    }

    ros::NodeHandle nh_, private_nh_;
    ros::Publisher command_pub_, select_pub_, path_pub_, marker_pub_, land_pub_;
    ros::Subscriber odom_sub_, trigger_sub_, reload_sub_;
    ros::Timer timer_;
    std::string route_path_, known_map_pcd_, saved_trajectory_path_, odom_topic_, command_topic_, select_topic_;
    std::string path_topic_, marker_topic_, trigger_topic_, land_topic_, reload_topic_;
    std::string route_param_, map_param_, super_config_path_;
    int start_trigger_type_{2};
    double start_delay_{5.0}, min_segment_time_{0.6}, collision_clearance_{0.21};
    double sample_dt_{0.03}, collision_validation_step_{0.01};
    double max_velocity_{2.45}, max_acceleration_{6.0}, max_jerk_{120.0};
    double max_snap_{500.0}, max_jerk_discontinuity_{1.0e-4};
    double yaw_start_blend_duration_{1.0}, yaw_terminal_blend_duration_{1.5};
    double yaw_velocity_threshold_{0.08}, yaw_lookahead_time_{0.30}, max_yaw_rate_{2.5};
    double execution_max_velocity_{0.30}, execution_max_yaw_rate_{0.60};
    double execution_time_scale_{1.0}, saved_peak_velocity_{0.0}, saved_peak_yaw_rate_{0.0};
    double max_yaw_lock_variation_{1.0}, super_astar_timeout_{5.0};
    double super_corridor_extra_margin_{0.0};
    double narrow_corridor_clearance_threshold_{0.40};
    double narrow_corridor_max_width_{1.00};
    double narrow_corridor_transition_length_{0.45};
    double narrow_corridor_max_half_width_{0.18};
    double narrow_corridor_min_half_width_{0.10};
    double narrow_corridor_margin_{0.03};
    double narrow_corridor_side_vertical_window_{0.35};
    double narrow_corridor_side_longitudinal_window_{0.35};
    double narrow_corridor_max_plane_violation_{0.005};
    double vertical_guide_floor_max_violation_{0.002};
    double guide_tracking_horizontal_half_width_{0.12};
    double guide_tracking_monotonic_vertical_band_{0.01};
    double guide_tracking_max_plane_violation_{0.005};
    double regional_ceiling_z_{2.0}, regional_ceiling_grid_resolution_{0.2};
    double recorded_waypoint_speed_{1.2};
    double local_collision_replan_inflation_radius_{0.15};
    int local_collision_replan_max_attempts_{1};
    bool auto_land_{false}, have_odom_{false}, trigger_received_{false};
    bool wait_for_reload_{false}, preview_only_{false};
    bool save_generated_trajectory_{false}, execute_saved_trajectory_{false};
    bool load_saved_trajectory_for_preview_{false};
    bool preview_start_at_first_route_point_{false}, route_loaded_{false};
    bool saved_trajectory_loaded_{false};
    bool use_super_safe_corridor_{false}, super_static_map_only_{true};
    bool super_use_recorded_guide_without_astar_{false};
    bool local_collision_replan_enabled_{true};
    bool regional_ceiling_enabled_{false};
    bool narrow_corridor_enabled_{true};
    bool vertical_guide_floor_enabled_{true};
    bool guide_tracking_enabled_{true};
    bool preview_built_{false}, trajectory_safe_{false}, land_sent_{false};
    double start_time_{0.0}, odom_time_{0.0}, yaw_{0.0};
    double start_position_tolerance_{0.20}, start_yaw_tolerance_{M_PI / 4.0};
    double phase_start_time_{0.0}, dwell_start_time_{0.0}, complete_time_{0.0};
    double minimum_clearance_{0.0}, maximum_velocity_{0.0}, maximum_acceleration_{0.0};
    double certified_clearance_lower_bound_{0.0};
    double maximum_jerk_{0.0}, maximum_snap_seen_{0.0};
    double maximum_jerk_discontinuity_seen_{0.0}, maximum_yaw_rate_seen_{0.0};
    double maximum_yaw_lock_variation_seen_{0.0};
    double maximum_regional_height_seen_{-std::numeric_limits<double>::infinity()};
    int local_collision_replan_attempt_{0};
    std::size_t regional_ceiling_constraint_points_{0};
    std::vector<double> regional_ceiling_x_min_, regional_ceiling_x_max_;
    std::vector<double> regional_ceiling_y_min_, regional_ceiling_y_max_;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
        local_collision_guides_;
    std::vector<NarrowCorridorDiagnostic> narrow_corridor_diagnostics_;
    std::vector<NarrowCorridorDiagnostic> narrow_corridor_report_;
    Eigen::Vector3d position_{Eigen::Vector3d::Zero()};
    FailureDiagnostic failure_diagnostic_;
    SavedTrajectoryHeader saved_header_;
    SmoothRoute route_;
    std::vector<Phase, Eigen::aligned_allocator<Phase>> phases_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr known_map_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr planning_map_;
    pcl::KdTreeFLANN<pcl::PointXYZ> map_tree_;
    ros_interface::RosInterface::Ptr super_ros_;
    rog_map::ROGMapROS::Ptr super_map_;
    path_search::Astar::Ptr super_astar_;
    super_planner::CorridorGenerator::Ptr super_corridor_;
    traj_opt::ExpTrajOpt::Ptr super_optimizer_;
    State state_{State::WAIT_TRIGGER};
    int phase_index_{0};
};

}  // namespace mission_planner

int main(int argc, char** argv) {
    ros::init(argc, argv, "full_smooth_mission");
    try {
        mission_planner::FullSmoothMission mission;
        ros::spin();
    } catch (const std::exception& error) {
        ROS_FATAL("[FULL_SMOOTH] %s", error.what());
        return 1;
    }
    return 0;
}
