//
// Created by yunfan on 2022/3/26.
//

#ifndef MISSION_PLANNER_CONFIG
#define MISSION_PLANNER_CONFIG

#include "vector"
#include "string"
#include "iostream"
#include <iomanip>
#include <fstream>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utils/eigen_alias.hpp>
#include <utils/yaml_loader.hpp>
#include <utils/color_text.hpp>
namespace mission_planner {
    using namespace super_utils;
    using namespace color_text;
    using namespace std;
    enum TriggerType {
        RVIZ_CLICK = 0,
        MAVROS_RC = 1,
        TARGET_ODOM = 2,
        CONTROLLER_READY = 3
    };

    enum CmdType {
        PATH = 0,
        WAYPOINT = 1
    };

    class MissionConfig {
    public:
        // Bool Params

        int start_trigger_type;
        int cmd_type;
        double start_program_delay;
        string path_pub_topic, goal_pub_topic, odom_topic, waypoints_file_name;
        string start_trigger_topic, position_cmd_topic, super_position_cmd_topic, land_topic;
        string d_gate_mode, d_gate_cmd_topic, d_gate_select_topic, d_gate_super_pause_topic;
        string d_gate_collision_cloud_topic, d_gate_path_topic, d_gate_marker_topic;
        vec_E<Vec3f> waypoints;
        vector<double> switch_dis_vec;
        vector<double> waypoint_yaws;
        vector<double> dwell_times;
        vector<double> yaw_tolerances;
        double switch_dis;
        double odom_timeout;
        double publish_dt;
        bool auto_land;
        double land_after_cmd_silence;
        int d_gate_start_waypoint;
        int d_gate_end_waypoint;
        double d_gate_speed;
        double d_gate_min_segment_time;
        double d_gate_start_tolerance;
        double d_gate_start_max_speed;
        double d_gate_start_dwell;
        bool d_gate_start_require_yaw;
        double d_gate_handoff_min_hold;
        double d_gate_handoff_timeout;
        double d_gate_yaw;
        bool d_gate_finishes_mission;
        double d_gate_final_dwell;
        bool d_gate_collision_check_required;
        double d_gate_collision_clearance;
        double d_gate_collision_coverage_max_distance;
        double d_gate_collision_cloud_timeout;
        int d_gate_collision_min_points;
        double d_gate_sample_dt;
        vector<double> d_gate_guide_x;
        vector<double> d_gate_guide_y;
        vector<double> d_gate_guide_z;
        vector<double> d_gate_guide_speed;

        double str2double(string s) {
            size_t parsed = 0;
            const double value = std::stod(s, &parsed);
            if (parsed != s.size()) {
                throw std::invalid_argument("Invalid numeric waypoint value: " + s);
            }
            return value;
        }

        void LoadWaypoint(string file_name) {
            ifstream theFile(file_name);
            if (!theFile.is_open()) {
                throw std::runtime_error("Cannot open waypoint file: " + file_name);
            }
            std::string line;
            int line_number = 0;
            while (std::getline(theFile, line)) {
                line_number++;
                const auto comment_pos = line.find('#');
                if (comment_pos != std::string::npos) {
                    line = line.substr(0, comment_pos);
                }
                std::vector<std::string> result;
                std::istringstream iss(line);
                for (std::string s; iss >> s;) {
                    result.push_back(s);
                }
                if (result.empty()) {
                    continue;
                }
                if (result.size() < 4 || result.size() > 7) {
                    throw std::runtime_error(
                        "Invalid waypoint column count at " + file_name + ":" + std::to_string(line_number));
                }

                Vec3f log;
                log.x() = str2double(result[0]);
                log.y() = str2double(result[1]);
                log.z() = str2double(result[2]);
                waypoints.push_back(log);

                // Legacy format: x y z switch_distance
                if (result.size() == 4) {
                    waypoint_yaws.push_back(std::numeric_limits<double>::quiet_NaN());
                    switch_dis_vec.push_back(str2double(result[3]));
                    dwell_times.push_back(0.0);
                    yaw_tolerances.push_back(std::numeric_limits<double>::infinity());
                    continue;
                }

                // Extended format:
                // x y z yaw_deg switch_distance [dwell_sec [yaw_tolerance_deg]]
                const double yaw_deg = str2double(result[3]);
                waypoint_yaws.push_back(std::isnan(yaw_deg) ? yaw_deg : yaw_deg * M_PI / 180.0);
                switch_dis_vec.push_back(str2double(result[4]));
                dwell_times.push_back(result.size() >= 6 ? str2double(result[5]) : 0.0);
                const double yaw_tol_deg = result.size() >= 7 ? str2double(result[6]) : 10.0;
                yaw_tolerances.push_back(yaw_tol_deg * M_PI / 180.0);
            }


            cout << GREEN << " -- [MISSION] Load " << waypoints.size() << " waypoints." << RESET << endl;
            for (int i = 0; i < waypoints.size(); i++) {
                cout << BLUE << " -- [MISSION] Waypoint " << i << " at (" << waypoints[i].x() << ", "
                     << waypoints[i].y() << ", " << waypoints[i].z() << ")"
                     << " Yaw = " << (std::isnan(waypoint_yaws[i]) ? "auto" :
                                      std::to_string(waypoint_yaws[i] * 180.0 / M_PI) + " deg")
                     << " Switch dis = " << switch_dis_vec[i]
                     << " Dwell = " << dwell_times[i] << " s"
                     << RESET << endl;
            }
        }

        void ValidateGateConfig() const {
            if (d_gate_mode != "super" && d_gate_mode != "smooth") {
                throw std::runtime_error("d_gate/mode must be 'super' or 'smooth'");
            }
            if (d_gate_mode == "super") {
                return;
            }
            if (d_gate_start_waypoint < 0 || d_gate_end_waypoint <= d_gate_start_waypoint ||
                d_gate_end_waypoint >= static_cast<int>(waypoints.size()) ||
                (!d_gate_finishes_mission &&
                 d_gate_end_waypoint + 1 >= static_cast<int>(waypoints.size()))) {
                throw std::runtime_error("invalid smooth D-gate waypoint range");
            }
            if (d_gate_guide_x.empty() || d_gate_guide_x.size() != d_gate_guide_y.size() ||
                d_gate_guide_x.size() != d_gate_guide_z.size() ||
                d_gate_guide_x.size() != d_gate_guide_speed.size()) {
                throw std::runtime_error("D-gate guide x/y/z arrays must be non-empty and equal length");
            }
            if (d_gate_speed <= 0.0 || d_gate_min_segment_time <= 0.0 ||
                d_gate_start_tolerance <= 0.0 || d_gate_start_max_speed < 0.0 ||
                d_gate_start_dwell < 0.0 || d_gate_handoff_min_hold < 0.0 ||
                d_gate_handoff_timeout <= d_gate_handoff_min_hold || d_gate_final_dwell < 0.0) {
                throw std::runtime_error("invalid smooth D-gate timing, speed, or tolerance parameter");
            }
            if (d_gate_collision_clearance <= 0.0 ||
                d_gate_collision_coverage_max_distance <= d_gate_collision_clearance ||
                d_gate_collision_cloud_timeout <= 0.0 ||
                d_gate_collision_min_points < 1 || d_gate_sample_dt <= 0.0) {
                throw std::runtime_error("invalid smooth D-gate collision-check parameter");
            }
        }
        
        MissionConfig() {};

        MissionConfig(const std::string & cfg_path) {
            yaml_loader::YamlLoader loader(cfg_path);

            loader.LoadParam("start_trigger_type", start_trigger_type, 1);
            loader.LoadParam("start_program_delay", start_program_delay, 1.0);
            loader.LoadParam("cmd_type", cmd_type, 1);
            loader.LoadParam("switch_dis", switch_dis, 1.0);
            loader.LoadParam("odom_timeout", odom_timeout, 0.1);
            loader.LoadParam("publish_dt", publish_dt, 1.0);
            loader.LoadParam("auto_land", auto_land, false);
            loader.LoadParam("land_after_cmd_silence", land_after_cmd_silence, 0.2);
            loader.LoadParam("goal_pub_topic", goal_pub_topic, string("/planner/goal"));
            loader.LoadParam("odom_topic", odom_topic, string("/lidar_slam/odom"));
            loader.LoadParam("path_pub_topic", path_pub_topic, string("/planning/waypoint_path"));
            loader.LoadParam("start_trigger_topic", start_trigger_topic, string("/traj_start_trigger"));
            loader.LoadParam("position_cmd_topic", position_cmd_topic, string("/planning/pos_cmd"));
            loader.LoadParam("super_position_cmd_topic", super_position_cmd_topic,
                             string("/planning/super_pos_cmd"));
            loader.LoadParam("land_topic", land_topic, string("/px4ctrl/takeoff_land"));
            loader.LoadParam("d_gate/mode", d_gate_mode, string("super"));
            loader.LoadParam("d_gate/cmd_topic", d_gate_cmd_topic, string("/planning/gate_pos_cmd"));
            loader.LoadParam("d_gate/select_topic", d_gate_select_topic, string("/planning/gate_active"));
            loader.LoadParam("d_gate/super_pause_topic", d_gate_super_pause_topic,
                             string("/planning/super_pause"));
            loader.LoadParam("d_gate/collision_cloud_topic", d_gate_collision_cloud_topic,
                             string("/competition/planning_cloud"));
            loader.LoadParam("d_gate/path_topic", d_gate_path_topic,
                             string("/planning/d_gate_smooth_path"));
            loader.LoadParam("d_gate/marker_topic", d_gate_marker_topic,
                             string("/planning/d_gate_smooth_markers"));
            loader.LoadParam("d_gate/start_waypoint", d_gate_start_waypoint, 7);
            loader.LoadParam("d_gate/end_waypoint", d_gate_end_waypoint, 10);
            loader.LoadParam("d_gate/speed", d_gate_speed, 0.7);
            loader.LoadParam("d_gate/min_segment_time", d_gate_min_segment_time, 0.35);
            loader.LoadParam("d_gate/start_tolerance", d_gate_start_tolerance, 0.12);
            loader.LoadParam("d_gate/start_max_speed", d_gate_start_max_speed, 0.25);
            loader.LoadParam("d_gate/start_dwell", d_gate_start_dwell, 0.2);
            loader.LoadParam("d_gate/start_require_yaw", d_gate_start_require_yaw, true);
            loader.LoadParam("d_gate/handoff_min_hold", d_gate_handoff_min_hold, 0.25);
            loader.LoadParam("d_gate/handoff_timeout", d_gate_handoff_timeout, 2.0);
            double d_gate_yaw_deg;
            loader.LoadParam("d_gate/yaw_deg", d_gate_yaw_deg, 90.0);
            d_gate_yaw = d_gate_yaw_deg * M_PI / 180.0;
            loader.LoadParam("d_gate/finishes_mission", d_gate_finishes_mission, false);
            loader.LoadParam("d_gate/final_dwell", d_gate_final_dwell, 0.0);
            loader.LoadParam("d_gate/collision_check_required", d_gate_collision_check_required, true);
            loader.LoadParam("d_gate/collision_clearance", d_gate_collision_clearance, 0.21);
            loader.LoadParam("d_gate/collision_coverage_max_distance",
                             d_gate_collision_coverage_max_distance, 0.8);
            loader.LoadParam("d_gate/collision_cloud_timeout", d_gate_collision_cloud_timeout, 0.5);
            loader.LoadParam("d_gate/collision_min_points", d_gate_collision_min_points, 100);
            loader.LoadParam("d_gate/sample_dt", d_gate_sample_dt, 0.04);
            loader.LoadParam("d_gate/guide_x", d_gate_guide_x, vector<double>{});
            loader.LoadParam("d_gate/guide_y", d_gate_guide_y, vector<double>{});
            loader.LoadParam("d_gate/guide_z", d_gate_guide_z, vector<double>{});
            loader.LoadParam("d_gate/guide_speed", d_gate_guide_speed, vector<double>{});
//            loader.LoadParam("waypoints_file_name", waypoints_file_name, string("a_working_waypoints.txt"));

        }

    };
}
#endif //PLANNER_CONFIG_HPP
