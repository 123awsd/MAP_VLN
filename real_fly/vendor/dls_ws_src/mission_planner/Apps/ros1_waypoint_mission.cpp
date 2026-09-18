//
// Created by yunfan on 2021/8/29.
//

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
/*
 * Test code:
 *      roslaunch simulator test_env.launch
 * */
#define BACKWARD_HAS_DW 1
#include "utils/backward.hpp"
namespace backward{
    backward::SignalHandling sh;
}

#include "waypoint_mission/ros1_waypoint_planner.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "waypoint_mission");
    ros::NodeHandle nh("~");
    /* Publisher and subcriber */

	mission_planner::WaypointPlanner wpl(nh);

    // Mission state, command-source handoff and collision-cloud callbacks share
    // state. A single callback thread keeps their ordering deterministic.
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Duration(1.0).sleep();
    ros::waitForShutdown();
    return 0;
}
