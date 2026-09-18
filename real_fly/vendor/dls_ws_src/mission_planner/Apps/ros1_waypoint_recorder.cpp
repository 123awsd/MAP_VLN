#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <interactive_markers/interactive_marker_server.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <unordered_map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace {

struct Waypoint {
    double x;
    double y;
    double z;
    double yaw;
    bool stop;
};

struct MapVoxelKey {
    int x{0};
    int y{0};
    int z{0};

    bool operator==(const MapVoxelKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct MapVoxelKeyHash {
    std::size_t operator()(const MapVoxelKey& key) const {
        const auto hash_combine = [](std::size_t seed, const std::size_t value) {
            return seed ^ (value + 0x9e3779b9U + (seed << 6U) + (seed >> 2U));
        };
        std::size_t value = std::hash<int>{}(key.x);
        value = hash_combine(value, std::hash<int>{}(key.y));
        return hash_combine(value, std::hash<int>{}(key.z));
    }
};

struct MapVoxelObservation {
    int count{0};
    ros::Time first_seen;
    ros::Time last_seen;
};

class WaypointRecorder {
public:
    WaypointRecorder() : private_nh_("~") {
        private_nh_.param("odom_topic", odom_topic_, std::string("/ekf_quat/ekf_odom"));
        private_nh_.param("output_file", output_file_, std::string());
        if (output_file_.empty()) {
            output_file_ = workspaceOutputFile("recorded_waypoints.txt");
        }
        private_nh_.param("overwrite_existing", overwrite_existing_, true);
        private_nh_.param("record_yaw", record_yaw_, true);
        private_nh_.param("use_relative_origin", use_relative_origin_, false);
        private_nh_.param("default_switch_distance", switch_distance_, 0.65);
        private_nh_.param("default_dwell_time", dwell_time_, 0.0);
        private_nh_.param("default_stop_dwell_time", stop_dwell_time_, 1.0);
        private_nh_.param("default_smooth_speed", smooth_speed_, 1.2);
        private_nh_.param("default_yaw_tolerance_deg", yaw_tolerance_deg_, 25.0);
        private_nh_.param("max_capture_speed", max_capture_speed_, 0.15);
        private_nh_.param("min_capture_interval", min_capture_interval_, 0.5);
        private_nh_.param("finalize_cooldown", finalize_cooldown_, 3.0);
        private_nh_.param("cloud_topic", cloud_topic_, std::string("/cloud_registered"));
        private_nh_.param("map_output_file", map_output_file_, std::string());
        if (map_output_file_.empty()) {
            map_output_file_ = workspaceOutputFile("recorded_environment_map.pcd");
        }
        private_nh_.param("existing_map_file", existing_map_file_, std::string());
        if (existing_map_file_.empty()) {
            existing_map_file_ = map_output_file_;
        }
        private_nh_.param("map_topic", map_topic_,
                          std::string("/waypoint_recorder/accumulated_map"));
        private_nh_.param("expected_map_frame", expected_map_frame_, std::string("world"));
        private_nh_.param("map_voxel_size", map_voxel_size_, 0.08);
        private_nh_.param("map_visualization_voxel_size", map_visualization_voxel_size_, 0.16);
        private_nh_.param("map_publish_interval", map_publish_interval_, 1.0);
        private_nh_.param("max_map_points_before_compaction", max_map_points_before_compaction_, 500000);
        private_nh_.param("map_min_x", map_min_x_, -1.0);
        private_nh_.param("map_max_x", map_max_x_, 15.0);
        private_nh_.param("map_min_y", map_min_y_, -14.0);
        private_nh_.param("map_max_y", map_max_y_, 1.0);
        private_nh_.param("map_min_z", map_min_z_, -0.2);
        private_nh_.param("map_max_z", map_max_z_, 2.2);
        private_nh_.param("map_min_observations", map_min_observations_, 3);
        private_nh_.param("map_observation_interval", map_observation_interval_, 0.20);
        private_nh_.param("map_min_persistence_time", map_min_persistence_time_, 1.0);
        // Reject the aircraft, the operator's hands, and other near-field
        // returns while the recorder is carried.  A 0.6 m sphere is inside
        // the vehicle's non-flyable envelope and therefore does not remove
        // usable competition-environment structure.
        private_nh_.param("map_self_exclusion_radius", map_self_exclusion_radius_, 0.60);
        private_nh_.param("map_capture_enabled", map_capture_enabled_, true);
        chooseSafeOutputFile();
        private_nh_.param("smooth_output_file", smooth_output_file_, std::string());
        if (smooth_output_file_.empty()) {
            smooth_output_file_ = workspaceOutputFile("recorded_full_smooth_route.txt");
        }
        chooseSafeSmoothOutputFile();
        chooseSafeMapOutputFile();
        if (map_voxel_size_ <= 0.0 || map_visualization_voxel_size_ <= 0.0 ||
            map_publish_interval_ <= 0.0 ||
            max_map_points_before_compaction_ <= 0 || map_min_observations_ <= 0 ||
            map_observation_interval_ < 0.0 || map_min_persistence_time_ < 0.0 ||
            map_self_exclusion_radius_ < 0.0 ||
            finalize_cooldown_ < 0.0 ||
            map_min_x_ >= map_max_x_ || map_min_y_ >= map_max_y_ ||
            map_min_z_ >= map_max_z_) {
            throw std::runtime_error("map voxel sizes, publish interval, and maximum point count must be positive");
        }

        odom_sub_ = nh_.subscribe(odom_topic_, 10, &WaypointRecorder::odomCallback, this);
        cloud_sub_ = nh_.subscribe(cloud_topic_, 5, &WaypointRecorder::cloudCallback, this,
                                   ros::TransportHints().tcpNoDelay());
        capture_sub_ = private_nh_.subscribe("capture", 1, &WaypointRecorder::captureCallback, this);
        capture_stop_sub_ = private_nh_.subscribe("capture_stop", 1,
                                                   &WaypointRecorder::captureStopCallback, this);
        capture_click_sub_ = private_nh_.subscribe("capture_click", 1, &WaypointRecorder::captureClickCallback, this);
        capture_stop_click_sub_ = private_nh_.subscribe("capture_stop_click", 1,
                                                         &WaypointRecorder::captureStopClickCallback, this);
        set_origin_sub_ = private_nh_.subscribe("set_origin", 1, &WaypointRecorder::setOriginCallback, this);
        undo_sub_ = private_nh_.subscribe("undo", 1, &WaypointRecorder::undoCallback, this);
        clear_sub_ = private_nh_.subscribe("clear", 1, &WaypointRecorder::clearCallback, this);
        load_sub_ = private_nh_.subscribe("load_last", 1, &WaypointRecorder::loadLastCallback, this);
        map_capture_toggle_sub_ = private_nh_.subscribe("toggle_map_capture", 1,
                                                         &WaypointRecorder::toggleMapCaptureCallback, this);
        finalize_sub_ = private_nh_.subscribe("finalize", 1, &WaypointRecorder::finalizeCallback, this);
        marker_pub_ = private_nh_.advertise<visualization_msgs::MarkerArray>("markers", 1, true);
        odom_visual_pub_ = private_nh_.advertise<nav_msgs::Odometry>("odom", 10);
        map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(map_topic_, 1, true);
        preview_reload_pub_ = nh_.advertise<std_msgs::Empty>(
            "/waypoint_recorder/full_smooth_preview", 1);
        control_server_.reset(new interactive_markers::InteractiveMarkerServer(
            "waypoint_recorder_controls", "", false));
        createInteractiveControls();
        control_timer_ = nh_.createTimer(ros::Duration(0.25),
                                         &WaypointRecorder::controlTimerCallback, this);

        ROS_INFO_STREAM("[waypoint_recorder] Waiting for odometry on " << odom_topic_ << ".\n"
                        << "  Capture:  rostopic pub -1 /waypoint_recorder/capture std_msgs/Empty '{}'\n"
                        << "  Stop:     rostopic pub -1 /waypoint_recorder/capture_stop std_msgs/Empty '{}'\n"
                        << "  Finalize: rostopic pub -1 /waypoint_recorder/finalize std_msgs/Empty '{}'\n"
                        << "  RViz has separate Capture pass / Capture stop tools; clicks always record the current UAV pose.\n"
                        << "  Set origin: rostopic pub -1 /waypoint_recorder/set_origin std_msgs/Empty '{}'\n"
                        << "  Waypoints: " << output_file_ << "\n"
                        << "  Full-smooth route (written by Finalize): " << smooth_output_file_ << "\n"
                        << "  Same-frame accumulated PCD (written by Finalize): " << map_output_file_ << "\n"
                        << "  Map ROI: [" << map_min_x_ << ", " << map_max_x_ << "] x ["
                        << map_min_y_ << ", " << map_max_y_ << "] x [" << map_min_z_ << ", "
                        << map_max_z_ << "], stable observations >= " << map_min_observations_
                        << " across " << map_min_persistence_time_ << " s\n"
                        << "  RViz: use the colored PASS / STOP / FINISH buttons around the UAV; no terminal command is needed.");
        bool load_existing_waypoints = false;
        private_nh_.param("load_existing_waypoints", load_existing_waypoints, false);
        if (load_existing_waypoints) {
            loadLastWaypoints();
        }
        bool load_existing_map = false;
        private_nh_.param("load_existing_map", load_existing_map, false);
        if (load_existing_map) {
            loadExistingMap();
        }
        writeFile();
        publishMarkers();
    }

private:
    void odomCallback(const nav_msgs::OdometryConstPtr& msg) {
        odom_visual_pub_.publish(msg);
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        current_z_ = msg->pose.pose.position.z;
        const auto& q = msg->pose.pose.orientation;
        current_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        const auto& velocity = msg->twist.twist.linear;
        current_speed_ = std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z);
        odom_frame_ = msg->header.frame_id;
        have_odom_ = true;
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        if (!map_capture_enabled_) {
            return;
        }
        if (!expected_map_frame_.empty() && msg->header.frame_id != expected_map_frame_) {
            ROS_ERROR_THROTTLE(1.0,
                               "[waypoint_recorder] cloud frame '%s' is not expected frame '%s'; map update dropped",
                               msg->header.frame_id.c_str(), expected_map_frame_.c_str());
            return;
        }
        pcl::PointCloud<pcl::PointXYZ> raw_cloud;
        pcl::fromROSMsg(*msg, raw_cloud);
        std::vector<int> valid_indices;
        pcl::removeNaNFromPointCloud(raw_cloud, raw_cloud, valid_indices);
        if (raw_cloud.empty()) {
            return;
        }
        pcl::PointCloud<pcl::PointXYZ> venue_cloud;
        venue_cloud.reserve(raw_cloud.size());
        for (const auto& point : raw_cloud.points) {
            if (!pointInsideMapRoi(point)) {
                continue;
            }
            if (map_self_exclusion_radius_ > 0.0 && have_odom_ &&
                std::hypot(std::hypot(point.x - current_x_, point.y - current_y_),
                           point.z - current_z_) < map_self_exclusion_radius_) {
                continue;
            }
            venue_cloud.push_back(point);
        }
        if (venue_cloud.empty()) {
            return;
        }
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(venue_cloud.makeShared());
        voxel_filter.setLeafSize(map_voxel_size_, map_voxel_size_, map_voxel_size_);
        pcl::PointCloud<pcl::PointXYZ> reduced_cloud;
        voxel_filter.filter(reduced_cloud);
        const ros::Time observation_time = msg->header.stamp.isZero() ? ros::Time::now()
                                                                        : msg->header.stamp;
        for (const auto& point : reduced_cloud.points) {
            auto& observation = map_voxel_observations_[mapVoxelKey(point)];
            if (observation.count == 0) {
                observation.count = 1;
                observation.first_seen = observation_time;
                observation.last_seen = observation_time;
            } else if ((observation_time - observation.last_seen).toSec() >=
                       map_observation_interval_) {
                ++observation.count;
                observation.last_seen = observation_time;
            }
        }
        *accumulated_map_ += reduced_cloud;
        map_frame_ = msg->header.frame_id;
        if (static_cast<int>(accumulated_map_->size()) > max_map_points_before_compaction_) {
            compactMap();
        }
        const ros::Time now = ros::Time::now();
        if (last_map_publish_.isZero() ||
            (now - last_map_publish_).toSec() >= map_publish_interval_) {
            publishAccumulatedMap();
        }
    }

    MapVoxelKey mapVoxelKey(const pcl::PointXYZ& point) const {
        return {static_cast<int>(std::floor(point.x / map_voxel_size_)),
                static_cast<int>(std::floor(point.y / map_voxel_size_)),
                static_cast<int>(std::floor(point.z / map_voxel_size_))};
    }

    bool pointInsideMapRoi(const pcl::PointXYZ& point) const {
        return point.x >= map_min_x_ && point.x <= map_max_x_ &&
               point.y >= map_min_y_ && point.y <= map_max_y_ &&
               point.z >= map_min_z_ && point.z <= map_max_z_;
    }

    pcl::PointCloud<pcl::PointXYZ> confirmedMap() const {
        pcl::PointCloud<pcl::PointXYZ> confirmed;
        confirmed.reserve(accumulated_map_->size());
        for (const auto& point : accumulated_map_->points) {
            const auto observation = map_voxel_observations_.find(mapVoxelKey(point));
            if (observation != map_voxel_observations_.end() &&
                observation->second.count >= map_min_observations_ &&
                (observation->second.last_seen - observation->second.first_seen).toSec() >=
                    map_min_persistence_time_) {
                confirmed.push_back(point);
            }
        }
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(confirmed.makeShared());
        voxel_filter.setLeafSize(map_voxel_size_, map_voxel_size_, map_voxel_size_);
        pcl::PointCloud<pcl::PointXYZ> compacted;
        voxel_filter.filter(compacted);
        return compacted;
    }

    void compactMap() {
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(accumulated_map_);
        voxel_filter.setLeafSize(map_voxel_size_, map_voxel_size_, map_voxel_size_);
        pcl::PointCloud<pcl::PointXYZ> compacted;
        voxel_filter.filter(compacted);
        *accumulated_map_ = std::move(compacted);
    }

    void publishAccumulatedMap() {
        if (accumulated_map_->empty() || map_frame_.empty()) {
            return;
        }
        const pcl::PointCloud<pcl::PointXYZ> stable_map = confirmedMap();
        if (stable_map.empty()) {
            return;
        }
        // Keep the full-resolution accumulated cloud for PCD export and
        // collision checking. RViz receives a separate coarser cloud so a
        // long recording remains interactive.
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(stable_map.makeShared());
        voxel_filter.setLeafSize(map_visualization_voxel_size_,
                                 map_visualization_voxel_size_,
                                 map_visualization_voxel_size_);
        pcl::PointCloud<pcl::PointXYZ> visualization_cloud;
        voxel_filter.filter(visualization_cloud);
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(visualization_cloud, output);
        output.header.stamp = ros::Time::now();
        output.header.frame_id = map_frame_;
        map_pub_.publish(output);
        last_map_publish_ = output.header.stamp;
    }

    bool mapAndOdomFramesMatch() const {
        return !map_frame_.empty() &&
               (odom_frame_.empty() || odom_frame_ == map_frame_);
    }

    void writeMapFile() const {
        pcl::PointCloud<pcl::PointXYZ> output_map = confirmedMap();
        if (output_map.empty()) {
            throw std::runtime_error("no map voxels met the stable-observation threshold");
        }
        if (use_relative_origin_) {
            for (auto& point : output_map.points) {
                point.x -= static_cast<float>(origin_x_);
                point.y -= static_cast<float>(origin_y_);
                point.z -= static_cast<float>(origin_z_);
            }
        }
        if (pcl::io::savePCDFileBinary(map_output_file_, output_map) != 0) {
            throw std::runtime_error("cannot write accumulated map PCD: " + map_output_file_);
        }
    }

    bool loadExistingMap() {
        pcl::PointCloud<pcl::PointXYZ> loaded;
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(existing_map_file_, loaded) != 0 ||
            loaded.empty()) {
            ROS_WARN_STREAM("[waypoint_recorder] Cannot load previous validation PCD: "
                            << existing_map_file_);
            return false;
        }
        std::vector<int> valid_indices;
        pcl::removeNaNFromPointCloud(loaded, loaded, valid_indices);
        pcl::PointCloud<pcl::PointXYZ> roi_cloud;
        roi_cloud.reserve(loaded.size());
        for (const auto& point : loaded.points) {
            if (pointInsideMapRoi(point)) {
                roi_cloud.push_back(point);
            }
        }
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(roi_cloud.makeShared());
        voxel_filter.setLeafSize(map_voxel_size_, map_voxel_size_, map_voxel_size_);
        pcl::PointCloud<pcl::PointXYZ> compacted;
        voxel_filter.filter(compacted);
        if (compacted.empty()) {
            ROS_WARN_STREAM("[waypoint_recorder] Previous validation PCD has no points inside the configured ROI: "
                            << existing_map_file_);
            return false;
        }
        *accumulated_map_ = std::move(compacted);
        map_voxel_observations_.clear();
        const ros::Time now = ros::Time::now();
        const ros::Duration persistence(map_min_persistence_time_);
        for (const auto& point : accumulated_map_->points) {
            map_voxel_observations_[mapVoxelKey(point)] = {
                map_min_observations_, now - persistence, now};
        }
        map_frame_ = expected_map_frame_;
        // Reusing a surveyed map must not silently mix new scans (and people)
        // into it. The operator may explicitly resume capture from RViz.
        map_capture_enabled_ = false;
        publishAccumulatedMap();
        ROS_INFO_STREAM("[waypoint_recorder] Loaded " << accumulated_map_->size()
                        << " previous validation PCD points from " << existing_map_file_
                        << "; map capture is PAUSED. FINISH can now revalidate this session.");
        return true;
    }

    void createInteractiveControls() {
        addInteractiveButton("record_pass", "PASS", 0.10f, 0.65f, 1.00f);
        addInteractiveButton("record_stop", "STOP", 1.00f, 0.45f, 0.05f);
        addInteractiveButton("record_finish", "FINISH", 0.15f, 0.95f, 0.20f);
        addInteractiveButton("record_undo", "UNDO", 0.95f, 0.25f, 0.25f);
        addInteractiveButton("record_origin", "ORIGIN", 0.75f, 0.35f, 0.95f);
        control_server_->applyChanges();
    }

    void addInteractiveButton(const std::string& name, const std::string& label,
                              const float red, const float green, const float blue) {
        visualization_msgs::InteractiveMarker marker;
        marker.header.frame_id = expected_map_frame_;
        marker.name = name;
        marker.description = label;
        marker.scale = 0.8;
        marker.pose.orientation.w = 1.0;

        visualization_msgs::InteractiveMarkerControl control;
        control.always_visible = true;
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;

        visualization_msgs::Marker box;
        box.type = visualization_msgs::Marker::CUBE;
        box.pose.orientation.w = 1.0;
        box.scale.x = 0.72;
        box.scale.y = 0.42;
        box.scale.z = 0.13;
        box.color.r = red;
        box.color.g = green;
        box.color.b = blue;
        box.color.a = 0.92;
        control.markers.push_back(box);

        visualization_msgs::Marker text;
        text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text.pose.position.z = 0.14;
        text.pose.orientation.w = 1.0;
        text.scale.z = 0.18;
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0;
        text.text = label;
        control.markers.push_back(text);
        marker.controls.push_back(control);
        control_server_->insert(marker, [this](
                                    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {
            controlFeedback(feedback);
        });
    }

    void controlTimerCallback(const ros::TimerEvent&) {
        if (!have_odom_) {
            return;
        }
        setInteractiveButtonPose("record_pass", 0.85, 0.55, 0.65);
        setInteractiveButtonPose("record_stop", 0.85, -0.55, 0.65);
        setInteractiveButtonPose("record_finish", -0.85, 0.55, 0.65);
        setInteractiveButtonPose("record_undo", -0.85, -0.55, 0.65);
        setInteractiveButtonPose("record_origin", 0.0, 0.0, 1.05);
        control_server_->applyChanges();
    }

    void setInteractiveButtonPose(const std::string& name, const double offset_x,
                                  const double offset_y, const double offset_z) {
        geometry_msgs::Pose pose;
        pose.position.x = current_x_ + offset_x;
        pose.position.y = current_y_ + offset_y;
        pose.position.z = current_z_ + offset_z;
        pose.orientation.w = 1.0;
        control_server_->setPose(name, pose);
    }

    void controlFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {
        if (feedback->event_type != visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK) {
            return;
        }
        if (feedback->marker_name == "record_pass") {
            captureCurrentPose(false);
        } else if (feedback->marker_name == "record_stop") {
            captureCurrentPose(true);
        } else if (feedback->marker_name == "record_finish") {
            finalizeCallback(std_msgs::EmptyConstPtr());
        } else if (feedback->marker_name == "record_undo") {
            undoCallback(std_msgs::EmptyConstPtr());
        } else if (feedback->marker_name == "record_origin") {
            setOriginCallback(std_msgs::EmptyConstPtr());
        }
    }

    void setOriginCallback(const std_msgs::EmptyConstPtr&) {
        if (!have_odom_) {
            ROS_WARN("[waypoint_recorder] Cannot set origin before receiving odometry.");
            return;
        }
        origin_x_ = current_x_;
        origin_y_ = current_y_;
        origin_z_ = current_z_;
        origin_set_ = true;
        ROS_INFO("[waypoint_recorder] Origin set to current odometry position.");
    }

    void captureCallback(const std_msgs::EmptyConstPtr&) {
        captureCurrentPose(false);
    }

    void captureStopCallback(const std_msgs::EmptyConstPtr&) {
        captureCurrentPose(true);
    }

    void captureClickCallback(const geometry_msgs::PointStampedConstPtr&) {
        // The click is only a convenient GUI trigger. The measured EKF pose, not
        // the mouse position, is the recorded waypoint.
        captureCurrentPose(false);
    }

    void captureStopClickCallback(const geometry_msgs::PointStampedConstPtr&) {
        captureCurrentPose(true);
    }

    void captureCurrentPose(const bool stop) {
        if (!have_odom_) {
            ROS_WARN("[waypoint_recorder] Capture ignored: no odometry received yet.");
            return;
        }
        if (use_relative_origin_ && !origin_set_) {
            ROS_WARN("[waypoint_recorder] Capture ignored: set the relative origin first.");
            return;
        }
        if (current_speed_ > max_capture_speed_) {
            ROS_WARN_STREAM("[waypoint_recorder] Capture ignored: speed " << current_speed_
                            << " m/s exceeds " << max_capture_speed_ << " m/s.");
            return;
        }
        const ros::Time now = ros::Time::now();
        if (!last_capture_time_.isZero() && (now - last_capture_time_).toSec() < min_capture_interval_) {
            ROS_WARN("[waypoint_recorder] Capture ignored: duplicate trigger.");
            return;
        }

        // A PASS point deliberately has no yaw constraint.  It is used by the
        // ordinary SUPER mission as a positional switch target and by
        // full_smooth as an optional spatial guide; retaining the yaw measured
        // while the aircraft is carried would unnecessarily constrain both.
        Waypoint waypoint{current_x_, current_y_, current_z_,
                          stop ? current_yaw_ : std::numeric_limits<double>::quiet_NaN(), stop};
        if (use_relative_origin_) {
            waypoint.x -= origin_x_;
            waypoint.y -= origin_y_;
            waypoint.z -= origin_z_;
        }
        waypoints_.push_back(waypoint);
        last_capture_time_ = now;
        last_finalize_time_ = ros::Time();
        writeFile();
        publishMarkers();
        ROS_INFO_STREAM("[waypoint_recorder] Captured " << waypoints_.size() << ": ["
                        << waypoint.x << ", " << waypoint.y << ", " << waypoint.z << "], "
                        << (stop ? "yaw=" + std::to_string(waypoint.yaw * 180.0 / M_PI) + " deg, STOP"
                                 : "yaw=nan (unconstrained), PASS") << ".");
    }

    void undoCallback(const std_msgs::EmptyConstPtr&) {
        if (waypoints_.empty()) {
            ROS_WARN("[waypoint_recorder] Undo ignored: no recorded waypoint.");
            return;
        }
        waypoints_.pop_back();
        last_finalize_time_ = ros::Time();
        writeFile();
        publishMarkers();
        ROS_INFO_STREAM("[waypoint_recorder] Removed the last waypoint; " << waypoints_.size() << " remain.");
    }

    void clearCallback(const std_msgs::EmptyConstPtr&) {
        waypoints_.clear();
        last_finalize_time_ = ros::Time();
        writeFile();
        publishMarkers();
        ROS_INFO("[waypoint_recorder] Cleared all recorded waypoints.");
    }

    bool loadLastWaypoints() {
        std::ifstream input(output_file_);
        if (!input.is_open()) {
            ROS_WARN_STREAM("[waypoint_recorder] Cannot load previous waypoints: " << output_file_);
            return false;
        }
        std::vector<Waypoint> loaded;
        std::string line;
        int line_number = 0;
        while (std::getline(input, line)) {
            ++line_number;
            const auto comment = line.find('#');
            if (comment != std::string::npos) {
                line.erase(comment);
            }
            std::istringstream parser(line);
            Waypoint waypoint{};
            std::string yaw_text;
            double ignored_switch_distance = 0.0;
            double dwell = 0.0;
            double ignored_yaw_tolerance = 0.0;
            if (!(parser >> waypoint.x)) {
                continue;
            }
            if (!(parser >> waypoint.y >> waypoint.z >> yaw_text >> ignored_switch_distance >> dwell >>
                  ignored_yaw_tolerance)) {
                ROS_WARN_STREAM("[waypoint_recorder] Ignoring malformed waypoint row " << line_number
                                << " in " << output_file_);
                continue;
            }
            try {
                waypoint.yaw = std::stod(yaw_text);
            } catch (const std::exception&) {
                ROS_WARN_STREAM("[waypoint_recorder] Ignoring malformed waypoint yaw at row "
                                << line_number << " in " << output_file_);
                continue;
            }
            waypoint.yaw = std::isnan(waypoint.yaw) ? waypoint.yaw : waypoint.yaw * M_PI / 180.0;
            waypoint.stop = dwell > 1.0e-6;
            if (waypoint.stop && !std::isfinite(waypoint.yaw)) {
                ROS_WARN_STREAM("[waypoint_recorder] Ignoring STOP row without finite yaw at row "
                                << line_number << " in " << output_file_);
                continue;
            }
            loaded.push_back(waypoint);
        }
        if (loaded.empty()) {
            ROS_WARN_STREAM("[waypoint_recorder] No valid previous waypoints found in " << output_file_);
            return false;
        }
        waypoints_ = std::move(loaded);
        last_capture_time_ = ros::Time();
        last_finalize_time_ = ros::Time();
        ROS_INFO_STREAM("[waypoint_recorder] Loaded " << waypoints_.size()
                        << " previous waypoints from " << output_file_);
        return true;
    }

    void loadLastCallback(const std_msgs::EmptyConstPtr&) {
        if (loadLastWaypoints()) {
            writeFile();
            publishMarkers();
        }
    }

    void toggleMapCaptureCallback(const std_msgs::EmptyConstPtr&) {
        map_capture_enabled_ = !map_capture_enabled_;
        ROS_WARN_STREAM("[waypoint_recorder] Accumulated map capture "
                        << (map_capture_enabled_ ? "RESUMED" : "PAUSED")
                        << ". Pause it while carrying the aircraft beside people; resume only for"
                           " unobstructed venue scans.");
    }

    void writeFile() const {
        std::ofstream output(output_file_, std::ios::out | std::ios::trunc);
        if (!output.is_open()) {
            ROS_ERROR_STREAM("[waypoint_recorder] Cannot write " << output_file_);
            return;
        }
        output << "# Recorded by mission_planner/waypoint_recorder\n";
        output << "# x[m] y[m] z[m] yaw[deg|nan=velocity heading] switch_distance[m] dwell[s] yaw_tolerance[deg]\n";
        output << "# PASS points always use yaw=nan (no heading constraint); STOP points use recorded yaw and default_stop_dwell_time.\n";
        output << std::fixed << std::setprecision(3);
        for (const auto& waypoint : waypoints_) {
            output << waypoint.x << "  " << waypoint.y << "  " << waypoint.z << "  ";
            if (waypoint.stop && record_yaw_) {
                output << waypoint.yaw * 180.0 / M_PI;
            } else {
                output << "nan";
            }
            output << "  " << switch_distance_ << "  "
                   << (waypoint.stop ? stop_dwell_time_ : dwell_time_) << "  "
                   << yaw_tolerance_deg_ << "\n";
        }
    }

    void finalizeCallback(const std_msgs::EmptyConstPtr&) {
        const ros::Time now = ros::Time::now();
        if (!last_finalize_time_.isZero() &&
            (now - last_finalize_time_).toSec() < finalize_cooldown_) {
            ROS_WARN_STREAM("[waypoint_recorder] Finalize ignored: duplicate trigger within "
                            << finalize_cooldown_ << " s.");
            return;
        }
        if (waypoints_.empty()) {
            ROS_WARN("[waypoint_recorder] Finalize ignored: no recorded waypoint.");
            return;
        }
        if (!waypoints_.back().stop) {
            ROS_WARN("[waypoint_recorder] Finalize ignored: the final point must be a STOP point with yaw and dwell.");
            return;
        }
        if (stop_dwell_time_ <= 0.0 || smooth_speed_ <= 0.0) {
            ROS_ERROR("[waypoint_recorder] default_stop_dwell_time and default_smooth_speed must be positive.");
            return;
        }
        if (use_relative_origin_) {
            ROS_ERROR("[waypoint_recorder] Finalize blocked: full_smooth uses the live world-frame odometry as its start, so use_relative_origin must be false for a same-frame PCD validation.");
            return;
        }
        if (accumulated_map_->empty()) {
            ROS_ERROR("[waypoint_recorder] Finalize blocked: no registered cloud has been accumulated yet.");
            return;
        }
        if (!mapAndOdomFramesMatch()) {
            ROS_ERROR_STREAM("[waypoint_recorder] Finalize blocked: odometry frame '" << odom_frame_
                             << "' does not match accumulated cloud frame '" << map_frame_
                             << "'. Do not validate a route against a different coordinate frame.");
            return;
        }
        compactMap();
        publishAccumulatedMap();
        try {
            writeMapFile();
        } catch (const std::exception& error) {
            ROS_ERROR("[waypoint_recorder] Finalize blocked: %s", error.what());
            return;
        }
        writeSmoothFile();
        private_nh_.setParam("generated_route_path", smooth_output_file_);
        private_nh_.setParam("generated_map_path", map_output_file_);
        preview_reload_pub_.publish(std_msgs::Empty());
        last_finalize_time_ = now;
        publishMarkers();
        ROS_INFO_STREAM("[waypoint_recorder] Full-smooth route written to " << smooth_output_file_
                        << "; same-frame accumulated map written to " << map_output_file_
                        << ". Triggered safe full-smooth preview generation in RViz.");
    }

    void writeSmoothFile() const {
        std::ofstream output(smooth_output_file_, std::ios::out | std::ios::trunc);
        if (!output.is_open()) {
            ROS_ERROR_STREAM("[waypoint_recorder] Cannot write " << smooth_output_file_);
            return;
        }
        output << "# Generated by mission_planner/waypoint_recorder\n";
        output << "# x[m] y[m] z[m] speed[m/s] yaw[deg|nan] dwell[s] mode[pass|stop]\n";
        output << "# pass: optional guide; the full-smooth planner may relax it only if the complete candidate stays safe.\n";
        output << "# stop: exact phase endpoint, fixed yaw, and dwell. The final route row must be stop.\n";
        output << std::fixed << std::setprecision(3);
        for (const auto& waypoint : waypoints_) {
            output << waypoint.x << "  " << waypoint.y << "  " << waypoint.z << "  "
                   << smooth_speed_ << "  ";
            if (waypoint.stop) {
                output << waypoint.yaw * 180.0 / M_PI << "  " << stop_dwell_time_ << "  stop\n";
            } else {
                output << "nan  0.000  pass\n";
            }
        }
    }

    static std::string workspaceOutputFile(const std::string& filename) {
        // ROOT_DIR is configured from this package's source directory. Ascending
        // mission_planner/SUPER/src yields the active workspace root on every
        // machine that builds this workspace; no user-specific absolute path is
        // embedded in the launch file.
        return std::string(ROOT_DIR) + "../../../" + filename;
    }

    void chooseSafeOutputFile() {
        if (overwrite_existing_) {
            return;
        }
        std::ifstream existing(output_file_);
        if (!existing.good()) {
            return;
        }
        existing.close();
        const std::string original = output_file_;
        for (int suffix = 1; suffix < 10000; ++suffix) {
            output_file_ = original + "." + std::to_string(suffix);
            std::ifstream candidate(output_file_);
            if (!candidate.good()) {
                ROS_WARN_STREAM("[waypoint_recorder] " << original << " already exists; write to "
                                << output_file_ << " instead.");
                return;
            }
        }
        ROS_FATAL_STREAM("[waypoint_recorder] Cannot find a free output name based on " << original);
        ros::shutdown();
    }

    void chooseSafeSmoothOutputFile() {
        if (overwrite_existing_) {
            return;
        }
        std::ifstream existing(smooth_output_file_);
        if (!existing.good()) {
            return;
        }
        existing.close();
        const std::string original = smooth_output_file_;
        for (int suffix = 1; suffix < 10000; ++suffix) {
            smooth_output_file_ = original + "." + std::to_string(suffix);
            std::ifstream candidate(smooth_output_file_);
            if (!candidate.good()) {
                ROS_WARN_STREAM("[waypoint_recorder] " << original << " already exists; full-smooth output will be "
                                << smooth_output_file_ << " instead.");
                return;
            }
        }
        ROS_FATAL_STREAM("[waypoint_recorder] Cannot find a free output name based on " << original);
        ros::shutdown();
    }

    void chooseSafeMapOutputFile() {
        if (overwrite_existing_) {
            return;
        }
        std::ifstream existing(map_output_file_);
        if (!existing.good()) {
            return;
        }
        existing.close();
        const std::string original = map_output_file_;
        for (int suffix = 1; suffix < 10000; ++suffix) {
            map_output_file_ = original + "." + std::to_string(suffix);
            std::ifstream candidate(map_output_file_);
            if (!candidate.good()) {
                ROS_WARN_STREAM("[waypoint_recorder] " << original << " already exists; accumulated map output will be "
                                << map_output_file_ << " instead.");
                return;
            }
        }
        ROS_FATAL_STREAM("[waypoint_recorder] Cannot find a free output name based on " << original);
        ros::shutdown();
    }

    void publishMarkers() const {
        visualization_msgs::MarkerArray markers;
        visualization_msgs::Marker clear;
        clear.header.frame_id = "world";
        clear.header.stamp = ros::Time::now();
        clear.action = visualization_msgs::Marker::DELETEALL;
        markers.markers.push_back(clear);

        visualization_msgs::Marker line;
        line.header = clear.header;
        line.ns = "recorded_waypoint_path";
        line.id = 0;
        line.type = visualization_msgs::Marker::LINE_STRIP;
        line.action = visualization_msgs::Marker::ADD;
        line.scale.x = 0.04;
        line.color.r = 0.1;
        line.color.g = 0.8;
        line.color.b = 1.0;
        line.color.a = 1.0;

        for (std::size_t index = 0; index < waypoints_.size(); ++index) {
            const auto& waypoint = waypoints_[index];
            geometry_msgs::Point point;
            point.x = waypoint.x + (use_relative_origin_ ? origin_x_ : 0.0);
            point.y = waypoint.y + (use_relative_origin_ ? origin_y_ : 0.0);
            point.z = waypoint.z + (use_relative_origin_ ? origin_z_ : 0.0);
            line.points.push_back(point);

            visualization_msgs::Marker sphere;
            sphere.header = clear.header;
            sphere.ns = "recorded_waypoints";
            sphere.id = static_cast<int>(index);
            sphere.type = visualization_msgs::Marker::SPHERE;
            sphere.action = visualization_msgs::Marker::ADD;
            sphere.pose.position = point;
            sphere.pose.orientation.w = 1.0;
            sphere.scale.x = 0.22;
            sphere.scale.y = 0.22;
            sphere.scale.z = 0.22;
            sphere.color.r = waypoint.stop ? 1.0 : 0.1;
            sphere.color.g = waypoint.stop ? 0.55 : 0.75;
            sphere.color.b = waypoint.stop ? 0.0 : 1.0;
            sphere.color.a = 1.0;
            markers.markers.push_back(sphere);

            if (waypoint.stop) {
                visualization_msgs::Marker yaw_arrow;
                yaw_arrow.header = clear.header;
                yaw_arrow.ns = "recorded_waypoint_yaws";
                yaw_arrow.id = static_cast<int>(index);
                yaw_arrow.type = visualization_msgs::Marker::ARROW;
                yaw_arrow.action = visualization_msgs::Marker::ADD;
                yaw_arrow.scale.x = 0.05;
                yaw_arrow.scale.y = 0.12;
                yaw_arrow.scale.z = 0.16;
                yaw_arrow.color.r = 0.1;
                yaw_arrow.color.g = 1.0;
                yaw_arrow.color.b = 0.1;
                yaw_arrow.color.a = 1.0;
                yaw_arrow.points.push_back(point);
                geometry_msgs::Point arrow_tip = point;
                arrow_tip.x += 0.55 * std::cos(waypoint.yaw);
                arrow_tip.y += 0.55 * std::sin(waypoint.yaw);
                yaw_arrow.points.push_back(arrow_tip);
                markers.markers.push_back(yaw_arrow);
            }

            visualization_msgs::Marker text;
            text.header = clear.header;
            text.ns = "recorded_waypoint_ids";
            text.id = 1000 + static_cast<int>(index);
            text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text.action = visualization_msgs::Marker::ADD;
            text.pose.position = point;
            text.pose.position.z += 0.22;
            text.pose.orientation.w = 1.0;
            text.scale.z = 0.25;
            text.color.r = 1.0;
            text.color.g = 1.0;
            text.color.b = 1.0;
            text.color.a = 1.0;
            std::ostringstream label;
            label << std::fixed << std::setprecision(2)
                  << "#" << index + 1 << " " << (waypoint.stop ? "STOP" : "PASS") << "  ["
                  << waypoint.x << ", " << waypoint.y << ", "
                  << waypoint.z << "]\n"
                  << (waypoint.stop ? "yaw " + std::to_string(waypoint.yaw * 180.0 / M_PI) +
                                         " deg, dwell " + std::to_string(stop_dwell_time_) + " s"
                                    : "optional safe pass");
            text.text = label.str();
            markers.markers.push_back(text);
        }
        if (!line.points.empty()) {
            markers.markers.push_back(line);
        }
        marker_pub_.publish(markers);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber capture_sub_;
    ros::Subscriber capture_stop_sub_;
    ros::Subscriber capture_click_sub_;
    ros::Subscriber capture_stop_click_sub_;
    ros::Subscriber set_origin_sub_;
    ros::Subscriber undo_sub_;
    ros::Subscriber clear_sub_;
    ros::Subscriber load_sub_;
    ros::Subscriber map_capture_toggle_sub_;
    ros::Subscriber finalize_sub_;
    ros::Publisher marker_pub_;
    ros::Publisher odom_visual_pub_;
    ros::Publisher map_pub_;
    ros::Publisher preview_reload_pub_;
    ros::Timer control_timer_;
    std::unique_ptr<interactive_markers::InteractiveMarkerServer> control_server_;

    std::string odom_topic_;
    std::string output_file_;
    std::string smooth_output_file_;
    std::string cloud_topic_;
    std::string map_output_file_;
    std::string existing_map_file_;
    std::string map_topic_;
    std::string expected_map_frame_;
    std::string odom_frame_;
    std::string map_frame_;
    bool overwrite_existing_{false};
    bool record_yaw_{true};
    bool use_relative_origin_{false};
    bool have_odom_{false};
    bool origin_set_{false};
    bool map_capture_enabled_{true};
    double switch_distance_{0.65};
    double dwell_time_{0.0};
    double stop_dwell_time_{1.0};
    double smooth_speed_{1.2};
    double map_voxel_size_{0.08};
    double map_visualization_voxel_size_{0.16};
    double map_publish_interval_{1.0};
    int max_map_points_before_compaction_{500000};
    double map_min_x_{-1.0}, map_max_x_{15.0};
    double map_min_y_{-14.0}, map_max_y_{1.0};
    double map_min_z_{-0.2}, map_max_z_{2.2};
    int map_min_observations_{3};
    double map_observation_interval_{0.20};
    double map_min_persistence_time_{1.0};
    double map_self_exclusion_radius_{0.0};
    double yaw_tolerance_deg_{25.0};
    double max_capture_speed_{0.15};
    double min_capture_interval_{0.5};
    double finalize_cooldown_{3.0};
    double current_x_{0.0};
    double current_y_{0.0};
    double current_z_{0.0};
    double current_yaw_{0.0};
    double current_speed_{0.0};
    double origin_x_{0.0};
    double origin_y_{0.0};
    double origin_z_{0.0};
    ros::Time last_capture_time_;
    ros::Time last_finalize_time_;
    ros::Time last_map_publish_;
    std::vector<Waypoint> waypoints_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_map_{new pcl::PointCloud<pcl::PointXYZ>()};
    std::unordered_map<MapVoxelKey, MapVoxelObservation, MapVoxelKeyHash> map_voxel_observations_;
};

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_recorder");
    WaypointRecorder recorder;
    ros::spin();
    return 0;
}
