#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace {

class CompetitionCloudGuard {
public:
    CompetitionCloudGuard() : private_nh_("~") {
        private_nh_.param("enabled", enabled_, true);
        private_nh_.param("input_topic", input_topic_, std::string("/cloud_registered"));
        private_nh_.param("output_topic", output_topic_, std::string("/competition/planning_cloud"));
        private_nh_.param("constraint_topic", constraint_topic_,
                          std::string("/competition/static_constraint_cloud"));
        private_nh_.param("expected_frame", expected_frame_, std::string("world"));
        private_nh_.param("ceiling_z", ceiling_z_, 2.0);
        private_nh_.param("grid_resolution", grid_resolution_, 0.2);
        private_nh_.param("virtual_point_intensity", virtual_point_intensity_, 0.0);

        private_nh_.getParam("region_x_min", region_x_min_);
        private_nh_.getParam("region_x_max", region_x_max_);
        private_nh_.getParam("region_y_min", region_y_min_);
        private_nh_.getParam("region_y_max", region_y_max_);

        validateConfig();
        buildVirtualCeiling();

        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);
        constraint_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(constraint_topic_, 1, true);
        cloud_sub_ = nh_.subscribe(input_topic_, 1, &CompetitionCloudGuard::cloudCallback, this,
                                   ros::TransportHints().tcpNoDelay());
        publishVirtualCeiling();

        ROS_INFO("[competition_cloud_guard] %s: input=%s output=%s constraint=%s regions=%zu "
                 "ceiling_z=%.2f points=%zu",
                 enabled_ ? "enabled" : "disabled", input_topic_.c_str(), output_topic_.c_str(),
                 constraint_topic_.c_str(), region_x_min_.size(), ceiling_z_, virtual_ceiling_.size());
    }

private:
    void validateConfig() const {
        if (grid_resolution_ <= 0.0) {
            throw std::runtime_error("grid_resolution must be positive");
        }

        const std::size_t region_count = region_x_min_.size();
        if (region_count == 0) {
            throw std::runtime_error("at least one virtual-ceiling region is required");
        }
        if (region_x_max_.size() != region_count || region_y_min_.size() != region_count ||
            region_y_max_.size() != region_count) {
            throw std::runtime_error("region bound arrays must have the same length");
        }

        for (std::size_t i = 0; i < region_count; ++i) {
            if (region_x_min_[i] > region_x_max_[i] || region_y_min_[i] > region_y_max_[i]) {
                throw std::runtime_error("invalid virtual-ceiling region bounds at index " +
                                         std::to_string(i));
            }
        }
    }

    void buildVirtualCeiling() {
        virtual_ceiling_.clear();
        if (!enabled_) {
            return;
        }

        for (std::size_t region = 0; region < region_x_min_.size(); ++region) {
            const int x_steps = static_cast<int>(
                std::ceil((region_x_max_[region] - region_x_min_[region]) / grid_resolution_));
            const int y_steps = static_cast<int>(
                std::ceil((region_y_max_[region] - region_y_min_[region]) / grid_resolution_));

            for (int xi = 0; xi <= x_steps; ++xi) {
                const double x = std::min(region_x_min_[region] + xi * grid_resolution_,
                                          region_x_max_[region]);
                for (int yi = 0; yi <= y_steps; ++yi) {
                    const double y = std::min(region_y_min_[region] + yi * grid_resolution_,
                                              region_y_max_[region]);
                    pcl::PointXYZI point;
                    point.x = static_cast<float>(x);
                    point.y = static_cast<float>(y);
                    point.z = static_cast<float>(ceiling_z_);
                    point.intensity = static_cast<float>(virtual_point_intensity_);
                    virtual_ceiling_.push_back(point);
                }
            }
        }

        virtual_ceiling_.width = static_cast<std::uint32_t>(virtual_ceiling_.size());
        virtual_ceiling_.height = 1;
        virtual_ceiling_.is_dense = true;
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        if (!enabled_) {
            cloud_pub_.publish(msg);
            return;
        }

        if (!expected_frame_.empty() && msg->header.frame_id != expected_frame_) {
            ROS_ERROR_THROTTLE(1.0,
                               "[competition_cloud_guard] input frame '%s' does not match expected frame '%s'; "
                               "drop the cloud to avoid applying an unsafe coordinate constraint",
                               msg->header.frame_id.c_str(), expected_frame_.c_str());
            return;
        }

        // Keep the live scan unmodified so ROG-Map can perform physically valid
        // free-space raycasting from the lidar origin to each real return.
        cloud_pub_.publish(msg);
    }

    void publishVirtualCeiling() {
        if (!enabled_ || virtual_ceiling_.empty()) {
            return;
        }
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(virtual_ceiling_, output);
        output.header.stamp = ros::Time::now();
        output.header.frame_id = expected_frame_;
        constraint_pub_.publish(output);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher cloud_pub_;
    ros::Publisher constraint_pub_;

    bool enabled_{true};
    std::string input_topic_;
    std::string output_topic_;
    std::string constraint_topic_;
    std::string expected_frame_;
    double ceiling_z_{2.0};
    double grid_resolution_{0.2};
    double virtual_point_intensity_{0.0};
    std::vector<double> region_x_min_;
    std::vector<double> region_x_max_;
    std::vector<double> region_y_min_;
    std::vector<double> region_y_max_;
    pcl::PointCloud<pcl::PointXYZI> virtual_ceiling_;
};

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "competition_cloud_guard");
    try {
        CompetitionCloudGuard guard;
        ros::spin();
    } catch (const std::exception& error) {
        ROS_FATAL("[competition_cloud_guard] configuration error: %s", error.what());
        return 1;
    }
    return 0;
}
