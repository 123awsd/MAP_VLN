#ifndef _DEBUG_REPLAY_H_
#define _DEBUG_REPLAY_H_

#include <ros/ros.h>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/Px4ctrlDebug.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <cmath>

struct Config
{
    std::string posCmdTopic;
    std::string odomTopic;
    std::string debugTopic;
    std::string frame_id;

    inline void load(const ros::NodeHandle &nh_priv)
    {
        nh_priv.getParam("posCmdTopic", posCmdTopic);
        nh_priv.getParam("odomTopic", odomTopic);
        nh_priv.getParam("debugTopic", debugTopic);
        nh_priv.getParam("frame_id", frame_id);
    }
};

class DebugReplay 
{
private:
    Config cfg;

    ros::NodeHandle nh;
    ros::Subscriber posCmdSub;
    ros::Subscriber odomSub;
    ros::Subscriber debugSub;
    ros::Publisher idealOdomPub;
    ros::Publisher fbOdomPub;
    ros::Publisher execOdomPub;
    ros::Publisher dirPub;
    ros::Publisher ddirPub;

    bool receiveDebug = false, receiveCmd = false;
    nav_msgs::Odometry now_odom, cmd_odom, fb_odom;
    quadrotor_msgs::PositionCommand cmd;
    quadrotor_msgs::Px4ctrlDebug debug;
    visualization_msgs::Marker dirArrow;
    visualization_msgs::Marker ddirArrow;

public:
    inline void poscmdcb(const quadrotor_msgs::PositionCommand::ConstPtr &msg)
    {
        cmd = *msg;
        receiveCmd = true;
        geometry_msgs::Point pt;

        dirArrow.header.stamp = msg->header.stamp;
        dirArrow.points.clear();
        pt.x = msg->position.x;
        pt.y = msg->position.y;
        pt.z = msg->position.z;
        dirArrow.points.push_back(pt);
        const double cos_yaw = std::cos(msg->yaw);
        const double sin_yaw = std::sin(msg->yaw);
        pt.x = msg->position.x + cos_yaw;
        pt.y = msg->position.y + sin_yaw;
        pt.z = msg->position.z;
        dirArrow.points.push_back(pt);
        dirPub.publish(dirArrow);

        ddirArrow.header.stamp = msg->header.stamp;
        ddirArrow.points.clear();
        pt.x = msg->position.x;
        pt.y = msg->position.y;
        pt.z = msg->position.z;
        ddirArrow.points.push_back(pt);
        pt.x = msg->position.x - msg->yaw_dot * sin_yaw;
        pt.y = msg->position.y + msg->yaw_dot * cos_yaw;
        pt.z = msg->position.z;
        ddirArrow.points.push_back(pt);
        ddirPub.publish(ddirArrow);

        if (receiveDebug)
        {
            cmd_odom.header.frame_id = cfg.frame_id;
            cmd_odom.header.stamp = ros::Time::now();
            cmd_odom.pose.pose.position.x = msg->position.x;
            cmd_odom.pose.pose.position.y = msg->position.y;
            cmd_odom.pose.pose.position.z = msg->position.z;
            cmd_odom.pose.pose.orientation.w = debug.ideal_q_w;
            cmd_odom.pose.pose.orientation.x = debug.ideal_q_x;
            cmd_odom.pose.pose.orientation.y = debug.ideal_q_y;
            cmd_odom.pose.pose.orientation.z = debug.ideal_q_z;
            idealOdomPub.publish(cmd_odom);
        }
    }

    inline void odomcb(const nav_msgs::Odometry::ConstPtr &msg)
    {
        now_odom = *msg;
        now_odom.header.frame_id = cfg.frame_id;
        execOdomPub.publish(now_odom);
    }

    inline void debugcb(const quadrotor_msgs::Px4ctrlDebug::ConstPtr &msg)
    {
        debug = *msg;
        receiveDebug = true;

        if (receiveCmd)
        {
            fb_odom.header.frame_id = cfg.frame_id;
            fb_odom.header.stamp = ros::Time::now();
            fb_odom.pose.pose.position.x = cmd.position.x;
            fb_odom.pose.pose.position.y = cmd.position.y;
            fb_odom.pose.pose.position.z = cmd.position.z;
            fb_odom.pose.pose.orientation.w = msg->des_q_w;
            fb_odom.pose.pose.orientation.x = msg->des_q_x;
            fb_odom.pose.pose.orientation.y = msg->des_q_y;
            fb_odom.pose.pose.orientation.z = msg->des_q_z;
            fbOdomPub.publish(fb_odom);
        }
    }

    DebugReplay(Config &conf, ros::NodeHandle &nh_)
    : cfg(conf), nh(nh_)
    {
        posCmdSub = nh.subscribe(cfg.posCmdTopic, 1, 
                                 &DebugReplay::poscmdcb, this,
                                 ros::TransportHints().tcpNoDelay());
        
        odomSub = nh.subscribe(cfg.odomTopic, 1,
                               &DebugReplay::odomcb, this,
                               ros::TransportHints().tcpNoDelay());
        
        debugSub = nh.subscribe(cfg.debugTopic, 1,
                                &DebugReplay::debugcb, this,
                                ros::TransportHints().tcpNoDelay());
        idealOdomPub = nh.advertise<nav_msgs::Odometry>("ideal_odom", 1);
        fbOdomPub = nh.advertise<nav_msgs::Odometry>("feedback_odom", 1);
        execOdomPub = nh.advertise<nav_msgs::Odometry>("real_odom", 1);
        dirPub = nh.advertise<visualization_msgs::Marker>("dir_arrow", 1);
        ddirPub = nh.advertise<visualization_msgs::Marker>("ddir_arrow", 1);

        dirArrow.id = 0;
        dirArrow.header.frame_id = "world";
        dirArrow.type = visualization_msgs::Marker::ARROW;
        dirArrow.action = visualization_msgs::Marker::ADD;
        dirArrow.color.a = 1.0;
        dirArrow.color.r = 0.0;
        dirArrow.color.g = 1.0;
        dirArrow.color.b = 0.0;

        dirArrow.scale.x = 0.05;
        dirArrow.scale.y = 0.1;
        dirArrow.scale.z = 0.1;
        dirArrow.pose.orientation.w = 1.0;

        ddirArrow = dirArrow;
        ddirArrow.color.r = 1.0;
        ddirArrow.color.g = 0.0;
        ddirArrow.color.b = 0.0;
    }

    ~DebugReplay(){}
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "debug_replay_node");

    Config conf;
    conf.load(ros::NodeHandle("~"));

    ros::NodeHandle _nh("~");
    DebugReplay dbg(conf, _nh);

    ros::Rate rate(100);
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
}

#endif
