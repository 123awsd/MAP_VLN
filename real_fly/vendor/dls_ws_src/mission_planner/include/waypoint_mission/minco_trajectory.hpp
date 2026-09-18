#ifndef MISSION_PLANNER_MINCO_TRAJECTORY_HPP
#define MISSION_PLANNER_MINCO_TRAJECTORY_HPP

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <data_structure/base/trajectory.h>
#include <traj_opt/minco.h>

namespace mission_planner {

// Fixed-time, global minimum-snap trajectory backed by SUPER's MINCO_S4NU.
// All internal waypoint derivatives are solved together; only the phase head
// and tail P/V/A/J are prescribed.
class MincoTrajectory {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    struct Sample {
        Eigen::Vector3d position{Eigen::Vector3d::Zero()};
        Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
        Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
        Eigen::Vector3d jerk{Eigen::Vector3d::Zero()};
        Eigen::Vector3d snap{Eigen::Vector3d::Zero()};
    };

    void build(
        const Eigen::Vector3d& start,
        const Eigen::Vector3d& start_velocity,
        const Eigen::Vector3d& start_acceleration,
        const Eigen::Vector3d& start_jerk,
        const std::vector<Eigen::Vector3d,
                          Eigen::aligned_allocator<Eigen::Vector3d>>& guides,
        const std::vector<double>& guide_speeds,
        const double minimum_segment_time) {
        if (guides.size() != guide_speeds.size() || minimum_segment_time <= 0.0) {
            throw std::invalid_argument(
                "MINCO guide positions and speeds must have equal size");
        }
        if (std::any_of(guide_speeds.begin(), guide_speeds.end(),
                        [](const double speed) { return speed <= 0.0; })) {
            throw std::invalid_argument("MINCO segment speeds must be positive");
        }

        Points points;
        std::vector<double> segment_speeds;
        points.push_back(start);
        for (std::size_t i = 0; i < guides.size(); ++i) {
            if ((guides[i] - points.back()).norm() > 1.0e-4) {
                points.push_back(guides[i]);
                segment_speeds.push_back(guide_speeds[i]);
            }
        }
        if (points.size() < 2) {
            throw std::invalid_argument("MINCO trajectory needs at least one guide");
        }

        const int piece_count = static_cast<int>(points.size()) - 1;
        Eigen::VectorXd durations(piece_count);
        knot_times_.assign(points.size(), 0.0);
        for (int i = 0; i < piece_count; ++i) {
            durations(i) = std::max((points[i + 1] - points[i]).norm() /
                                        segment_speeds[i],
                                    minimum_segment_time);
            knot_times_[i + 1] = knot_times_[i] + durations(i);
        }

        Eigen::Matrix<double, 3, 4> head = Eigen::Matrix<double, 3, 4>::Zero();
        Eigen::Matrix<double, 3, 4> tail = Eigen::Matrix<double, 3, 4>::Zero();
        head.col(0) = points.front();
        head.col(1) = start_velocity;
        head.col(2) = start_acceleration;
        head.col(3) = start_jerk;
        tail.col(0) = points.back();

        Eigen::MatrixXd internal_points(3, std::max(0, piece_count - 1));
        for (int i = 0; i + 1 < piece_count; ++i) {
            internal_points.col(i) = points[i + 1];
        }

        traj_opt::MINCO_S4NU minco;
        minco.setConditions(head, tail, piece_count);
        minco.setParameters(internal_points, durations);
        minco.getTrajectory(trajectory_);
        duration_ = durations.sum();
        final_position_ = points.back();
        external_trajectory_.clear();
    }

    // The recorder's safe global mode obtains its polynomial from SUPER's
    // corridor-constrained optimizer.  Keep one sampling interface so yaw,
    // validation and command publication are identical for both planners.
    void setCorridorTrajectory(const geometry_utils::Trajectory& trajectory) {
        if (trajectory.empty()) {
            throw std::invalid_argument("cannot assign an empty corridor trajectory");
        }
        external_trajectory_ = trajectory;
        duration_ = external_trajectory_.getTotalDuration();
        final_position_ = external_trajectory_.getPos(duration_);
        knot_times_.clear();
        knot_times_.push_back(0.0);
        double elapsed = 0.0;
        for (int i = 0; i < external_trajectory_.getPieceNum(); ++i) {
            elapsed += external_trajectory_[i].getDuration();
            knot_times_.push_back(elapsed);
        }
    }

    Sample sample(double elapsed) const {
        if (trajectory_.empty() && external_trajectory_.empty()) {
            throw std::runtime_error("MINCO trajectory was sampled before build");
        }
        elapsed = std::max(0.0, std::min(elapsed, duration_));
        Sample result;
        if (!external_trajectory_.empty()) {
            result.position = external_trajectory_.getPos(elapsed);
            result.velocity = external_trajectory_.getVel(elapsed);
            result.acceleration = external_trajectory_.getAcc(elapsed);
            result.jerk = external_trajectory_.getJer(elapsed);
            result.snap = external_trajectory_.getSnap(elapsed);
            return result;
        }
        result.position = trajectory_.getPos(elapsed);
        result.velocity = trajectory_.getVel(elapsed);
        result.acceleration = trajectory_.getAcc(elapsed);
        result.jerk = trajectory_.getJer(elapsed);
        result.snap = trajectory_.getSnap(elapsed);
        return result;
    }

    double maximumJerkDiscontinuity() const {
        double maximum = 0.0;
        if (!external_trajectory_.empty()) {
            for (int i = 0; i + 1 < external_trajectory_.getPieceNum(); ++i) {
                const Eigen::Vector3d before = external_trajectory_[i].getJer(
                    external_trajectory_[i].getDuration());
                const Eigen::Vector3d after = external_trajectory_[i + 1].getJer(0.0);
                maximum = std::max(maximum, (after - before).norm());
            }
            return maximum;
        }
        for (int i = 0; i + 1 < trajectory_.getPieceNum(); ++i) {
            const Eigen::Vector3d before =
                trajectory_[i].getJer(trajectory_[i].getDuration());
            const Eigen::Vector3d after = trajectory_[i + 1].getJer(0.0);
            maximum = std::max(maximum, (after - before).norm());
        }
        return maximum;
    }

    double duration() const { return duration_; }
    const Eigen::Vector3d& finalPosition() const { return final_position_; }
    const std::vector<double>& knotTimes() const { return knot_times_; }

    // Expose the exact polynomial for mission persistence.  Full-smooth
    // execution reuses these coefficients instead of re-running A*, corridor
    // construction, or MINCO after takeoff.
    const geometry_utils::Trajectory& rawTrajectory() const {
        if (trajectory_.empty() && external_trajectory_.empty()) {
            throw std::runtime_error("MINCO trajectory was exported before build");
        }
        return external_trajectory_.empty() ? trajectory_ : external_trajectory_;
    }

private:
    using Points = std::vector<Eigen::Vector3d,
                               Eigen::aligned_allocator<Eigen::Vector3d>>;

    geometry_utils::Trajectory trajectory_;
    geometry_utils::Trajectory external_trajectory_;
    std::vector<double> knot_times_;
    double duration_{0.0};
    Eigen::Vector3d final_position_{Eigen::Vector3d::Zero()};
};

}  // namespace mission_planner

#endif  // MISSION_PLANNER_MINCO_TRAJECTORY_HPP
