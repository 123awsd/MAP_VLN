#ifndef __INPUT_H
#define __INPUT_H

#include <ros/ros.h>
#include <Eigen/Dense>

#include <sensor_msgs/Imu.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <sensor_msgs/BatteryState.h>
#include <uav_utils/utils.h>
#include "PX4CtrlParam.h"

class RC_Data_t
{
public:
  double mode;                // 对应5通道模式
  double gear;                // 对应6通道模式
  double reboot_cmd;          // 对应8通道模式
  // 上次通道模式
  double last_mode;
  double last_gear;
  double last_reboot_cmd;
  // 通道初始化标志
  bool have_init_last_mode{false};
  bool have_init_last_gear{false};
  bool have_init_last_reboot_cmd{false};
  double ch[4];               // 归一化通道范围，带死区的[-1,1]

  mavros_msgs::RCIn msg;      // 消息本身
  ros::Time rcv_stamp;        // 接收时间戳

  bool is_command_mode;       // 56通道均在高位
  bool enter_command_mode;    // 5通道高位，且6通道从低到高
  bool is_hover_mode;         // 5通道在高位
  bool enter_hover_mode;      // 5通道从低到高
  bool toggle_reboot;         // 56通道均不在高位，8通道从低到高

  static constexpr double GEAR_SHIFT_VALUE = 0.75;          // 6通道切换判定，0.75
  static constexpr double API_MODE_THRESHOLD_VALUE = 0.75;  // 5通道切换判定，0.75
  static constexpr double REBOOT_THRESHOLD_VALUE = 0.5;     // 8通道切换判定，0.50
  static constexpr double DEAD_ZONE = 0.25;                 // 通道死区范围， 0.25

  RC_Data_t();
  void check_validity();
  bool check_centered();
  void feed(mavros_msgs::RCInConstPtr pMsg);
  bool is_received(const ros::Time &now_time);
};

class Odom_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d p;      // 位置
  Eigen::Vector3d v;      // 速度
  Eigen::Quaterniond q;   // 四元素
  Eigen::Vector3d w;      // 角速度

  nav_msgs::Odometry msg; // 消息本身
  ros::Time rcv_stamp;    // 接收时间戳
  bool recv_new_msg;      // 消息更新位，没啥用

  Odom_Data_t();
  void feed(nav_msgs::OdometryConstPtr pMsg);
};

class Imu_Data_t
{
public:
  Eigen::Quaterniond q;
  Eigen::Vector3d w;
  Eigen::Vector3d a;

  sensor_msgs::Imu msg;
  ros::Time rcv_stamp;

  Imu_Data_t();
  void feed(sensor_msgs::ImuConstPtr pMsg);
};

class State_Data_t
{
public:
  mavros_msgs::State current_state;
  mavros_msgs::State state_before_offboard;

  State_Data_t();
  void feed(mavros_msgs::StateConstPtr pMsg);
};

class ExtendedState_Data_t
{
public:
  mavros_msgs::ExtendedState current_extended_state;

  ExtendedState_Data_t();
  void feed(mavros_msgs::ExtendedStateConstPtr pMsg);
};

class Command_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Vector3d a;
  Eigen::Vector3d j;
  Eigen::Vector3d dir;
  Eigen::Vector3d ddir;
  double yaw;
  double yaw_rate;

  quadrotor_msgs::PositionCommand msg;
  ros::Time rcv_stamp;

  Command_Data_t();
  void feed(quadrotor_msgs::PositionCommandConstPtr pMsg);
};

class Battery_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double volt{0.0};
  double percentage{0.0};

  sensor_msgs::BatteryState msg;
  ros::Time rcv_stamp;

  Battery_Data_t();
  void feed(sensor_msgs::BatteryStateConstPtr pMsg);
};

class Takeoff_Land_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool triggered{false};
  uint8_t takeoff_land_cmd; // see TakeoffLand.msg for its defination

  quadrotor_msgs::TakeoffLand msg;
  ros::Time rcv_stamp;

  Takeoff_Land_Data_t();
  void feed(quadrotor_msgs::TakeoffLandConstPtr pMsg);
};

#endif