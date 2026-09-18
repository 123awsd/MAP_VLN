#ifndef MISSION_PLANNER_SMOOTH_ROUTE_HPP
#define MISSION_PLANNER_SMOOTH_ROUTE_HPP

#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>

namespace mission_planner {

struct SmoothRoutePoint {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    double speed{1.0};
    double yaw{std::numeric_limits<double>::quiet_NaN()};
    double dwell{0.0};
    // Legacy six-column routes keep every point as an exact MINCO knot.  New
    // recorder routes explicitly mark ordinary samples as soft pass points.
    // The full-smooth planner may remove those knots when doing so remains
    // collision-free and dynamically feasible.
    bool soft_pass{false};
};

using SmoothRoute =
    std::vector<SmoothRoutePoint, Eigen::aligned_allocator<SmoothRoutePoint>>;

inline SmoothRoute LoadSmoothRoute(const std::string& path,
                                   const double recorded_waypoint_speed = 1.2) {
    if (recorded_waypoint_speed <= 0.0) {
        throw std::invalid_argument("recorded waypoint replay speed must be positive");
    }
    std::ifstream input(path);
    if (!input.is_open()) {
        throw std::runtime_error("cannot open full smooth route: " + path);
    }
    SmoothRoute route;
    std::string line;
    int line_number = 0;
    while (std::getline(input, line)) {
        ++line_number;
        const auto comment = line.find('#');
        if (comment != std::string::npos) {
            line.erase(comment);
        }
        // The recorder's editable waypoint file has seven numeric columns:
        // x y z yaw switch_distance dwell yaw_tolerance.  Accept it directly
        // for read-only offline review, converting PASS/STOP semantics to the
        // generated full-smooth route format.  A generated route also has
        // seven columns, but its final column is the explicit pass/stop tag.
        std::istringstream token_parser(line);
        std::vector<std::string> tokens;
        std::string token_text;
        while (token_parser >> token_text) {
            tokens.push_back(token_text);
        }
        if (tokens.size() == 7 && tokens.back() != "pass" && tokens.back() != "stop") {
            try {
                const double x = std::stod(tokens[0]);
                const double y = std::stod(tokens[1]);
                const double z = std::stod(tokens[2]);
                const double yaw_deg = std::stod(tokens[3]);
                const double dwell = std::stod(tokens[5]);
                if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || dwell < 0.0 ||
                    (dwell > 0.0 && !std::isfinite(yaw_deg))) {
                    throw std::runtime_error("invalid recorded waypoint values");
                }
                SmoothRoutePoint point;
                point.position = Eigen::Vector3d(x, y, z);
                point.speed = recorded_waypoint_speed;
                point.yaw = std::isnan(yaw_deg) ? yaw_deg : yaw_deg * M_PI / 180.0;
                point.dwell = dwell;
                point.soft_pass = dwell <= 0.0;
                route.push_back(point);
                continue;
            } catch (const std::exception&) {
                throw std::runtime_error("invalid recorded waypoint row " +
                                         std::to_string(line_number));
            }
        }
        std::istringstream parser(line);
        double x, y, z, speed, dwell;
        std::string yaw_text;
        if (!(parser >> x)) {
            continue;
        }
        if (!(parser >> y >> z >> speed >> yaw_text >> dwell)) {
            throw std::runtime_error("invalid full smooth route row " +
                                     std::to_string(line_number));
        }
        std::size_t parsed = 0;
        const double yaw_deg = std::stod(yaw_text, &parsed);
        if (parsed != yaw_text.size()) {
            throw std::runtime_error("invalid yaw at full smooth route row " +
                                     std::to_string(line_number));
        }
        std::string mode;
        std::string extra;
        if ((parser >> mode && (parser >> extra)) || speed <= 0.0 || dwell < 0.0) {
            throw std::runtime_error("invalid full smooth route value at row " +
                                     std::to_string(line_number));
        }
        SmoothRoutePoint point;
        point.position = Eigen::Vector3d(x, y, z);
        point.speed = speed;
        point.yaw = std::isnan(yaw_deg) ? yaw_deg : yaw_deg * M_PI / 180.0;
        point.dwell = dwell;
        if (!mode.empty()) {
            if (mode == "pass") {
                if (dwell != 0.0 || !std::isnan(yaw_deg)) {
                    throw std::runtime_error(
                        "a soft pass point must use yaw=nan and dwell=0 at row " +
                        std::to_string(line_number));
                }
                point.soft_pass = true;
            } else if (mode == "stop") {
                if (dwell <= 0.0 || !std::isfinite(yaw_deg)) {
                    throw std::runtime_error(
                        "a stop point needs finite yaw and positive dwell at row " +
                        std::to_string(line_number));
                }
            } else {
                throw std::runtime_error("unknown full smooth route mode at row " +
                                         std::to_string(line_number));
            }
        }
        route.push_back(point);
    }
    if (route.empty() || route.back().dwell <= 0.0) {
        throw std::runtime_error("full smooth route must end with a positive dwell");
    }
    return route;
}

}  // namespace mission_planner

#endif
