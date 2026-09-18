#include <string>

#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

namespace {

class CompetitionCommandMux {
public:
    CompetitionCommandMux() : private_nh_("~") {
        private_nh_.param("super_topic", super_topic_, std::string("/planning/super_pos_cmd"));
        private_nh_.param("gate_topic", gate_topic_, std::string("/planning/gate_pos_cmd"));
        private_nh_.param("output_topic", output_topic_, std::string("/planning/pos_cmd"));
        private_nh_.param("gate_select_topic", gate_select_topic_, std::string("/planning/gate_active"));
        private_nh_.param("gate_command_timeout", gate_command_timeout_, 0.15);

        output_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>(output_topic_, 10);
        super_sub_ = nh_.subscribe(super_topic_, 20, &CompetitionCommandMux::superCallback, this,
                                   ros::TransportHints().tcpNoDelay());
        gate_sub_ = nh_.subscribe(gate_topic_, 20, &CompetitionCommandMux::gateCallback, this,
                                  ros::TransportHints().tcpNoDelay());
        select_sub_ = nh_.subscribe(gate_select_topic_, 2, &CompetitionCommandMux::selectCallback, this);
        watchdog_ = nh_.createTimer(ros::Duration(0.05), &CompetitionCommandMux::watchdogCallback, this);

        ROS_INFO("[competition_command_mux] SUPER=%s gate=%s output=%s select=%s",
                 super_topic_.c_str(), gate_topic_.c_str(), output_topic_.c_str(),
                 gate_select_topic_.c_str());
    }

private:
    void superCallback(const quadrotor_msgs::PositionCommandConstPtr& msg) {
        last_super_command_ = *msg;
        last_super_command_time_ = ros::Time::now();
        have_super_command_ = true;
        if (!gate_active_) {
            output_pub_.publish(msg);
        }
    }

    void gateCallback(const quadrotor_msgs::PositionCommandConstPtr& msg) {
        last_gate_command_time_ = ros::Time::now();
        gate_timeout_reported_ = false;
        if (gate_active_) {
            output_pub_.publish(msg);
        }
    }

    void selectCallback(const std_msgs::BoolConstPtr& msg) {
        gate_active_ = msg->data;
        gate_timeout_reported_ = false;
        if (gate_active_) {
            last_gate_command_time_ = ros::Time::now();
            ROS_WARN("[competition_command_mux] switch command source: SUPER -> smooth gate");
        } else {
            ROS_INFO("[competition_command_mux] switch command source: smooth gate -> SUPER");
            if (have_super_command_ &&
                (ros::Time::now() - last_super_command_time_).toSec() <= gate_command_timeout_) {
                output_pub_.publish(last_super_command_);
            }
        }
    }

    void watchdogCallback(const ros::TimerEvent&) {
        if (!gate_active_ || gate_timeout_reported_) {
            return;
        }
        if ((ros::Time::now() - last_gate_command_time_).toSec() > gate_command_timeout_) {
            gate_timeout_reported_ = true;
            ROS_ERROR("[competition_command_mux] smooth gate command timed out; keep SUPER blocked so PX4Ctrl can fall back to hover");
        }
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher output_pub_;
    ros::Subscriber super_sub_;
    ros::Subscriber gate_sub_;
    ros::Subscriber select_sub_;
    ros::Timer watchdog_;
    std::string super_topic_;
    std::string gate_topic_;
    std::string output_topic_;
    std::string gate_select_topic_;
    double gate_command_timeout_{0.15};
    bool gate_active_{false};
    bool gate_timeout_reported_{false};
    ros::Time last_gate_command_time_;
    ros::Time last_super_command_time_;
    quadrotor_msgs::PositionCommand last_super_command_;
    bool have_super_command_{false};
};

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "competition_command_mux");
    CompetitionCommandMux mux;
    ros::spin();
    return 0;
}
