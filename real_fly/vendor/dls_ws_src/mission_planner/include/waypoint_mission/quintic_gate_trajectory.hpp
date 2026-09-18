#ifndef MISSION_PLANNER_QUINTIC_GATE_TRAJECTORY_HPP
#define MISSION_PLANNER_QUINTIC_GATE_TRAJECTORY_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>

namespace mission_planner {

class QuinticGateTrajectory {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    struct Sample {
        Eigen::Vector3d position{Eigen::Vector3d::Zero()};
        Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
        Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
        Eigen::Vector3d jerk{Eigen::Vector3d::Zero()};
    };

    void build(const Eigen::Vector3d& start,
               const Eigen::Vector3d& start_velocity,
               const Eigen::Vector3d& start_acceleration,
               const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& guides,
               const double speed,
               const double minimum_segment_time) {
        std::vector<double> guide_speeds(guides.size(), speed);
        build(start, start_velocity, start_acceleration, guides, guide_speeds,
              minimum_segment_time);
    }

    void build(const Eigen::Vector3d& start,
               const Eigen::Vector3d& start_velocity,
               const Eigen::Vector3d& start_acceleration,
               const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& guides,
               const std::vector<double>& guide_speeds,
               const double minimum_segment_time) {
        if (guides.size() != guide_speeds.size() || minimum_segment_time <= 0.0) {
            throw std::invalid_argument("gate guide positions and speeds must have equal size");
        }
        if (std::any_of(guide_speeds.begin(), guide_speeds.end(),
                        [](const double speed) { return speed <= 0.0; })) {
            throw std::invalid_argument("gate trajectory speed and segment time must be positive");
        }

        Points points;
        std::vector<double> segment_speeds;
        points.push_back(start);
        for (std::size_t i = 0; i < guides.size(); ++i) {
            const auto& guide = guides[i];
            if ((guide - points.back()).norm() > 1.0e-4) {
                points.push_back(guide);
                segment_speeds.push_back(guide_speeds[i]);
            }
        }
        if (points.size() < 2) {
            throw std::invalid_argument("gate trajectory needs at least one guide after its start");
        }

        const std::size_t count = points.size();
        std::vector<double> knot_time(count, 0.0);
        for (std::size_t i = 1; i < count; ++i) {
            const double distance = (points[i] - points[i - 1]).norm();
            knot_time[i] = knot_time[i - 1] +
                           std::max(distance / segment_speeds[i - 1], minimum_segment_time);
        }

        Points velocities(count, Eigen::Vector3d::Zero());
        Points accelerations(count, Eigen::Vector3d::Zero());
        velocities.front() = clampNorm(start_velocity, segment_speeds.front());
        accelerations.front() = start_acceleration;
        for (std::size_t i = 1; i + 1 < count; ++i) {
            const double dt = knot_time[i + 1] - knot_time[i - 1];
            const double local_speed_limit = std::min(segment_speeds[i - 1], segment_speeds[i]);
            velocities[i] = clampNorm((points[i + 1] - points[i - 1]) / dt,
                                      local_speed_limit);
        }
        // End at rest so SUPER can safely take over from the gate exit.
        velocities.back().setZero();
        accelerations.back().setZero();

        segments_.clear();
        for (std::size_t i = 0; i + 1 < count; ++i) {
            Segment segment;
            segment.start_time = knot_time[i];
            segment.duration = knot_time[i + 1] - knot_time[i];
            computeCoefficients(points[i], velocities[i], accelerations[i],
                                points[i + 1], velocities[i + 1], accelerations[i + 1],
                                segment.duration, segment.coefficients);
            segments_.push_back(segment);
        }
        duration_ = knot_time.back();
        final_position_ = points.back();
    }

    Sample sample(double elapsed) const {
        if (segments_.empty()) {
            throw std::runtime_error("gate trajectory was sampled before build");
        }
        elapsed = std::max(0.0, std::min(elapsed, duration_));
        const Segment* segment = &segments_.back();
        for (const auto& candidate : segments_) {
            if (elapsed <= candidate.start_time + candidate.duration) {
                segment = &candidate;
                break;
            }
        }
        const double t = std::max(0.0, std::min(elapsed - segment->start_time, segment->duration));
        const auto& c = segment->coefficients;
        const double t2 = t * t;
        const double t3 = t2 * t;
        const double t4 = t3 * t;
        const double t5 = t4 * t;

        Sample result;
        result.position = c[0] + c[1] * t + c[2] * t2 + c[3] * t3 + c[4] * t4 + c[5] * t5;
        result.velocity = c[1] + 2.0 * c[2] * t + 3.0 * c[3] * t2 +
                          4.0 * c[4] * t3 + 5.0 * c[5] * t4;
        result.acceleration = 2.0 * c[2] + 6.0 * c[3] * t +
                              12.0 * c[4] * t2 + 20.0 * c[5] * t3;
        result.jerk = 6.0 * c[3] + 24.0 * c[4] * t + 60.0 * c[5] * t2;
        return result;
    }

    double duration() const { return duration_; }
    const Eigen::Vector3d& finalPosition() const { return final_position_; }

private:
    using Coefficients = std::array<Eigen::Vector3d, 6>;
    using Points = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;

    struct Segment {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        double start_time{0.0};
        double duration{0.0};
        Coefficients coefficients;
    };

    static Eigen::Vector3d clampNorm(const Eigen::Vector3d& value, const double maximum) {
        const double norm = value.norm();
        return norm > maximum && norm > 1.0e-9 ? value * (maximum / norm) : value;
    }

    static void computeCoefficients(const Eigen::Vector3d& p0,
                                    const Eigen::Vector3d& v0,
                                    const Eigen::Vector3d& a0,
                                    const Eigen::Vector3d& p1,
                                    const Eigen::Vector3d& v1,
                                    const Eigen::Vector3d& a1,
                                    const double t,
                                    Coefficients& c) {
        const Eigen::Vector3d delta = p1 - p0;
        const double t2 = t * t;
        const double t3 = t2 * t;
        const double t4 = t3 * t;
        const double t5 = t4 * t;
        c[0] = p0;
        c[1] = v0;
        c[2] = 0.5 * a0;
        c[3] = (20.0 * delta - (12.0 * v0 + 8.0 * v1) * t -
                (3.0 * a0 - a1) * t2) / (2.0 * t3);
        c[4] = (-30.0 * delta + (16.0 * v0 + 14.0 * v1) * t +
                (3.0 * a0 - 2.0 * a1) * t2) / (2.0 * t4);
        c[5] = (12.0 * delta - (6.0 * v0 + 6.0 * v1) * t -
                (a0 - a1) * t2) / (2.0 * t5);
    }

    std::vector<Segment, Eigen::aligned_allocator<Segment>> segments_;
    double duration_{0.0};
    Eigen::Vector3d final_position_{Eigen::Vector3d::Zero()};
};

}  // namespace mission_planner

#endif
