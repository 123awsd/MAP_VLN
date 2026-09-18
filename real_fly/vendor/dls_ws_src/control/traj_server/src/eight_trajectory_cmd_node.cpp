#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

namespace
{
constexpr double kPi = 3.14159265358979323846;

double yawFromQuaternion(const geometry_msgs::Quaternion &q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double wrapPi(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double smoothStep5(const double u)
{
  return u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
}

double smoothStep5Dot(const double u, const double ramp_time)
{
  return (30.0 * u * u - 60.0 * u * u * u + 30.0 * u * u * u * u) / ramp_time;
}

double smoothStep5Ddot(const double u, const double ramp_time)
{
  return (60.0 * u - 180.0 * u * u + 120.0 * u * u * u) /
         (ramp_time * ramp_time);
}

double smoothStep5Dddot(const double u, const double ramp_time)
{
  return (60.0 - 360.0 * u + 360.0 * u * u) /
         (ramp_time * ramp_time * ramp_time);
}

std::string expandUserPath(const std::string &path)
{
  if (path.empty() || path[0] != '~')
  {
    return path;
  }

  const char *home = std::getenv("HOME");
  if (!home)
  {
    return path;
  }

  if (path.size() == 1)
  {
    return std::string(home);
  }

  if (path[1] == '/')
  {
    return std::string(home) + path.substr(1);
  }

  return path;
}

bool ensureDirectory(const std::string &dir)
{
  if (dir.empty())
  {
    return false;
  }

  std::string current;
  size_t start = 0;
  if (dir[0] == '/')
  {
    current = "/";
    start = 1;
  }

  while (start <= dir.size())
  {
    const size_t slash = dir.find('/', start);
    const std::string part = dir.substr(start, slash - start);
    if (!part.empty())
    {
      if (!current.empty() && current[current.size() - 1] != '/')
      {
        current += "/";
      }
      current += part;

      if (::mkdir(current.c_str(), 0755) != 0 && errno != EEXIST)
      {
        return false;
      }
    }

    if (slash == std::string::npos)
    {
      break;
    }
    start = slash + 1;
  }

  return true;
}

std::string timestampString()
{
  const std::time_t now = std::time(NULL);
  std::tm tm_now;
  localtime_r(&now, &tm_now);

  char buffer[32];
  std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", &tm_now);
  return std::string(buffer);
}
}  // namespace

class EightTrajectoryCmdNode
{
public:
  EightTrajectoryCmdNode()
    : nh_(),
      pnh_("~"),
      have_odom_(false),
      active_(false),
      trajectory_id_(1),
      rows_written_(0)
  {
    pnh_.param("auto_start", auto_start_, true);
    pnh_.param("wait_for_trigger", wait_for_trigger_, false);
    pnh_.param("center_from_odom", center_from_odom_, true);
    pnh_.param("face_velocity", face_velocity_, false);
    pnh_.param("control_yaw", control_yaw_, true);
    pnh_.param("amplitude_x", amplitude_x_, 0.8);
    pnh_.param("amplitude_y", amplitude_y_, 0.4);
    pnh_.param("period", period_, 8.0);
    pnh_.param("publish_rate", publish_rate_, 100.0);
    pnh_.param("prestart_hold_time", prestart_hold_time_, 1.0);
    pnh_.param("ramp_time", ramp_time_, 2.0);
    pnh_.param("duration", duration_, 0.0);
    pnh_.param("num_periods", num_periods_, 3.0);
    pnh_.param("shutdown_after_finish", shutdown_after_finish_, true);
    pnh_.param("poststop_hold_time", poststop_hold_time_, 0.5);
    pnh_.param("center_x", center_x_, 0.0);
    pnh_.param("center_y", center_y_, 0.0);
    pnh_.param("center_z", center_z_, 1.0);
    pnh_.param("yaw", yaw_, 0.0);
    pnh_.param("write_csv", write_csv_, true);
    pnh_.param("publish_visualization", publish_visualization_, true);
    pnh_.param("max_path_points", max_path_points_, 5000);
    pnh_.param<std::string>("frame_id", frame_id_, "world");
    pnh_.param<std::string>("log_dir", log_dir_,
                            "~/.ros/traj_server_logs/eight_tracking");

    amplitude_x_ = std::max(0.0, amplitude_x_);
    amplitude_y_ = std::max(0.0, amplitude_y_);
    period_ = std::max(0.5, period_);
    publish_rate_ = std::max(20.0, publish_rate_);
    prestart_hold_time_ = std::max(0.0, prestart_hold_time_);
    ramp_time_ = std::max(0.0, ramp_time_);
    duration_ = std::max(0.0, duration_);
    num_periods_ = std::max(0.0, num_periods_);
    poststop_hold_time_ = std::max(0.0, poststop_hold_time_);
    max_path_points_ = std::max(20, max_path_points_);
    log_dir_ = expandUserPath(log_dir_);

    odom_sub_ = pnh_.subscribe("odometry", 50, &EightTrajectoryCmdNode::odomCallback,
                               this, ros::TransportHints().tcpNoDelay());
    trigger_sub_ = nh_.subscribe("/traj_start_trigger", 1,
                                 &EightTrajectoryCmdNode::triggerCallback, this);

    cmd_pub_ = pnh_.advertise<quadrotor_msgs::PositionCommand>("position_command", 50);
    desired_pos_pub_ = pnh_.advertise<geometry_msgs::PoseStamped>("desired_position", 50);
    desired_vel_pub_ = pnh_.advertise<visualization_msgs::Marker>("desired_velocity", 50);
    desired_acc_pub_ = pnh_.advertise<visualization_msgs::Marker>("desired_acceleration", 50);
    trajectory_vis_pub_ = pnh_.advertise<visualization_msgs::Marker>("trajectory_vis", 1, true);
    tracking_error_pub_ = pnh_.advertise<visualization_msgs::Marker>("tracking_error", 50);

    initTrajectoryMarker();
    if (write_csv_)
    {
      openLogFile();
    }

    if (wait_for_trigger_)
    {
      ROS_WARN("[eight_trajectory_cmd] Waiting for /traj_start_trigger.");
    }
    else
    {
      ROS_WARN("[eight_trajectory_cmd] Will start from current odometry once odom is received.");
    }
  }

  ~EightTrajectoryCmdNode()
  {
    if (log_file_.is_open())
    {
      log_file_.flush();
      log_file_.close();
    }
  }

  void spin()
  {
    ros::Rate rate(publish_rate_);
    while (ros::ok())
    {
      ros::spinOnce();

      if (auto_start_ && !active_ && !wait_for_trigger_ && have_odom_)
      {
        startFromOdom();
      }

      if (active_ && have_odom_)
      {
        const ros::Time now = ros::Time::now();
        quadrotor_msgs::PositionCommand cmd = evaluateCommand(now);
        cmd_pub_.publish(cmd);
        publishVisualization(cmd, now);
        writeLog(cmd, now);

        if (shutdown_after_finish_)
        {
          const double hold_duration = holdDuration();
          if (hold_duration > 0.0 &&
              (now - start_time_).toSec() >=
                  prestart_hold_time_ + hold_duration + poststop_hold_time_)
          {
            ROS_WARN("[eight_trajectory_cmd] Finished, shutting down (like the circle node)");
            ros::shutdown();
            return;
          }
        }
      }

      rate.sleep();
    }
  }

private:
  void odomCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_ = *msg;
    have_odom_ = true;
  }

  void triggerCallback(const geometry_msgs::PoseStampedConstPtr &msg)
  {
    if (!wait_for_trigger_)
    {
      return;
    }

    if (center_from_odom_ && have_odom_)
    {
      startFromOdom();
      return;
    }

    center_x_ = msg->pose.position.x;
    center_y_ = msg->pose.position.y;
    center_z_ = msg->pose.position.z;
    yaw_ = yawFromQuaternion(msg->pose.orientation);
    startTrajectory();
  }

  void startFromOdom()
  {
    center_x_ = odom_.pose.pose.position.x;
    center_y_ = odom_.pose.pose.position.y;
    center_z_ = odom_.pose.pose.position.z;
    yaw_ = yawFromQuaternion(odom_.pose.pose.orientation);
    startTrajectory();
  }

  void startTrajectory()
  {
    start_time_ = ros::Time::now();
    active_ = true;
    ++trajectory_id_;
    desired_points_.clear();
    trajectory_marker_.points.clear();
    ROS_WARN("[eight_trajectory_cmd] Start eight trajectory. center=(%.3f, %.3f, %.3f), yaw=%.3f, id=%u",
             center_x_, center_y_, center_z_, yaw_, trajectory_id_);
  }

  // Effective motion duration: explicit duration wins, otherwise N periods.
  double holdDuration() const
  {
    if (duration_ > 0.0)
    {
      return duration_;
    }
    if (num_periods_ > 0.0)
    {
      return num_periods_ * period_;
    }
    return 0.0;
  }

  quadrotor_msgs::PositionCommand evaluateCommand(const ros::Time &now)
  {
    const double elapsed = (now - start_time_).toSec();
    const double traj_t = elapsed - prestart_hold_time_;
    const double omega = 2.0 * kPi / period_;

    double x = 0.0;
    double y = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    double ax = 0.0;
    double ay = 0.0;
    double jx = 0.0;
    double jy = 0.0;

    if (traj_t > 0.0)
    {
      const double hold_duration = holdDuration();
      const double t = hold_duration > 0.0 ? std::min(traj_t, hold_duration) : traj_t;
      const bool hold_final = hold_duration > 0.0 && traj_t >= hold_duration;

      double s = 1.0;
      double sd = 0.0;
      double sdd = 0.0;
      double sddd = 0.0;
      if (ramp_time_ > 0.0 && t < ramp_time_)
      {
        const double u = std::max(0.0, std::min(1.0, t / ramp_time_));
        s = smoothStep5(u);
        sd = smoothStep5Dot(u, ramp_time_);
        sdd = smoothStep5Ddot(u, ramp_time_);
        sddd = smoothStep5Dddot(u, ramp_time_);
      }

      const double sin_wt = std::sin(omega * t);
      const double cos_wt = std::cos(omega * t);
      const double sin_2wt = std::sin(2.0 * omega * t);
      const double cos_2wt = std::cos(2.0 * omega * t);

      const double bx = amplitude_x_ * sin_wt;
      const double by = amplitude_y_ * sin_2wt;
      const double bvx = amplitude_x_ * omega * cos_wt;
      const double bvy = amplitude_y_ * 2.0 * omega * cos_2wt;
      const double bax = -amplitude_x_ * omega * omega * sin_wt;
      const double bay = -amplitude_y_ * 4.0 * omega * omega * sin_2wt;
      const double bjx = -amplitude_x_ * omega * omega * omega * cos_wt;
      const double bjy = -amplitude_y_ * 8.0 * omega * omega * omega * cos_2wt;

      x = s * bx;
      y = s * by;
      vx = sd * bx + s * bvx;
      vy = sd * by + s * bvy;
      ax = sdd * bx + 2.0 * sd * bvx + s * bax;
      ay = sdd * by + 2.0 * sd * bvy + s * bay;
      jx = sddd * bx + 3.0 * sdd * bvx + 3.0 * sd * bax + s * bjx;
      jy = sddd * by + 3.0 * sdd * bvy + 3.0 * sd * bay + s * bjy;

      if (hold_final)
      {
        vx = 0.0;
        vy = 0.0;
        ax = 0.0;
        ay = 0.0;
        jx = 0.0;
        jy = 0.0;
      }
    }

    double cmd_yaw = control_yaw_ ? yaw_ : 0.0;
    double yaw_dot = 0.0;
    if (control_yaw_ && face_velocity_ && std::hypot(vx, vy) > 0.05)
    {
      cmd_yaw = std::atan2(vy, vx);
    }

    quadrotor_msgs::PositionCommand cmd;
    cmd.header.stamp = now;
    cmd.header.frame_id = frame_id_;
    cmd.position.x = center_x_ + x;
    cmd.position.y = center_y_ + y;
    cmd.position.z = center_z_;
    cmd.velocity.x = vx;
    cmd.velocity.y = vy;
    cmd.velocity.z = 0.0;
    cmd.acceleration.x = ax;
    cmd.acceleration.y = ay;
    cmd.acceleration.z = 0.0;
    cmd.jerk.x = jx;
    cmd.jerk.y = jy;
    cmd.jerk.z = 0.0;
    cmd.yaw = wrapPi(cmd_yaw);
    cmd.yaw_dot = yaw_dot;
    cmd.trajectory_id = trajectory_id_;
    cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;

    return cmd;
  }

  void initTrajectoryMarker()
  {
    trajectory_marker_.header.frame_id = frame_id_;
    trajectory_marker_.ns = "eight_trajectory/trajectory";
    trajectory_marker_.id = 0;
    trajectory_marker_.type = visualization_msgs::Marker::LINE_STRIP;
    trajectory_marker_.action = visualization_msgs::Marker::ADD;
    trajectory_marker_.pose.orientation.w = 1.0;
    trajectory_marker_.scale.x = 0.04;
    trajectory_marker_.color.r = 0.0;
    trajectory_marker_.color.g = 0.2;
    trajectory_marker_.color.b = 1.0;
    trajectory_marker_.color.a = 0.9;
  }

  visualization_msgs::Marker makeArrow(const ros::Time &stamp, const std::string &ns,
                                        const int id, const double r,
                                        const double g, const double b) const
  {
    visualization_msgs::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = frame_id_;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.12;
    marker.scale.z = 0.12;
    marker.color.a = 1.0;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    return marker;
  }

  void publishVisualization(const quadrotor_msgs::PositionCommand &cmd, const ros::Time &now)
  {
    if (!publish_visualization_)
    {
      return;
    }

    geometry_msgs::PoseStamped desired_pose;
    desired_pose.header = cmd.header;
    desired_pose.pose.position = cmd.position;
    desired_pose.pose.orientation.w = 1.0;
    desired_pos_pub_.publish(desired_pose);

    geometry_msgs::Point pt;
    pt.x = cmd.position.x;
    pt.y = cmd.position.y;
    pt.z = cmd.position.z;

    trajectory_marker_.header.stamp = now;
    trajectory_marker_.points.push_back(pt);
    if (trajectory_marker_.points.size() > static_cast<size_t>(max_path_points_))
    {
      trajectory_marker_.points.erase(trajectory_marker_.points.begin());
    }
    trajectory_vis_pub_.publish(trajectory_marker_);

    visualization_msgs::Marker vel = makeArrow(now, "eight_trajectory/velocity", 1,
                                               0.0, 1.0, 0.0);
    vel.points.push_back(pt);
    geometry_msgs::Point vel_end = pt;
    vel_end.x += cmd.velocity.x;
    vel_end.y += cmd.velocity.y;
    vel_end.z += cmd.velocity.z;
    vel.points.push_back(vel_end);
    desired_vel_pub_.publish(vel);

    visualization_msgs::Marker acc = makeArrow(now, "eight_trajectory/acceleration", 2,
                                               1.0, 1.0, 0.0);
    acc.points.push_back(pt);
    geometry_msgs::Point acc_end = pt;
    acc_end.x += cmd.acceleration.x / 5.0;
    acc_end.y += cmd.acceleration.y / 5.0;
    acc_end.z += cmd.acceleration.z / 5.0;
    acc.points.push_back(acc_end);
    desired_acc_pub_.publish(acc);

    visualization_msgs::Marker err;
    err.header.stamp = now;
    err.header.frame_id = frame_id_;
    err.ns = "eight_trajectory/tracking_error";
    err.id = 3;
    err.type = visualization_msgs::Marker::LINE_LIST;
    err.action = visualization_msgs::Marker::ADD;
    err.pose.orientation.w = 1.0;
    err.scale.x = 0.02;
    err.color.a = 1.0;
    err.color.r = 1.0;
    err.color.g = 0.0;
    err.color.b = 0.0;
    err.points.push_back(pt);
    geometry_msgs::Point odom_pt;
    odom_pt.x = odom_.pose.pose.position.x;
    odom_pt.y = odom_.pose.pose.position.y;
    odom_pt.z = odom_.pose.pose.position.z;
    err.points.push_back(odom_pt);
    tracking_error_pub_.publish(err);
  }

  void openLogFile()
  {
    if (!ensureDirectory(log_dir_))
    {
      ROS_ERROR("[eight_trajectory_cmd] Failed to create log directory: %s", log_dir_.c_str());
      write_csv_ = false;
      return;
    }

    const std::string log_path = log_dir_ + "/eight_tracking_" + timestampString() + ".csv";
    log_file_.open(log_path.c_str(), std::ios::out);
    if (!log_file_.is_open())
    {
      ROS_ERROR("[eight_trajectory_cmd] Failed to open log file: %s", log_path.c_str());
      write_csv_ = false;
      return;
    }

    log_file_ << "time,traj_t,des_px,des_py,des_pz,des_vx,des_vy,des_vz,"
              << "des_ax,des_ay,des_az,odom_px,odom_py,odom_pz,odom_vx,odom_vy,odom_vz,"
              << "err_px,err_py,err_pz,err_norm,vel_err_x,vel_err_y,vel_err_z,vel_err_norm,"
              << "des_yaw,odom_yaw,yaw_err\n";
    ROS_WARN("[eight_trajectory_cmd] Writing CSV log to %s", log_path.c_str());
  }

  void writeLog(const quadrotor_msgs::PositionCommand &cmd, const ros::Time &now)
  {
    if (!write_csv_ || !log_file_.is_open())
    {
      return;
    }

    const double odom_vx = odom_.twist.twist.linear.x;
    const double odom_vy = odom_.twist.twist.linear.y;
    const double odom_vz = odom_.twist.twist.linear.z;
    const double err_x = odom_.pose.pose.position.x - cmd.position.x;
    const double err_y = odom_.pose.pose.position.y - cmd.position.y;
    const double err_z = odom_.pose.pose.position.z - cmd.position.z;
    const double vel_err_x = odom_vx - cmd.velocity.x;
    const double vel_err_y = odom_vy - cmd.velocity.y;
    const double vel_err_z = odom_vz - cmd.velocity.z;
    const double odom_yaw = yawFromQuaternion(odom_.pose.pose.orientation);

    log_file_ << std::fixed << std::setprecision(6)
              << now.toSec() << ","
              << (now - start_time_).toSec() << ","
              << cmd.position.x << "," << cmd.position.y << "," << cmd.position.z << ","
              << cmd.velocity.x << "," << cmd.velocity.y << "," << cmd.velocity.z << ","
              << cmd.acceleration.x << "," << cmd.acceleration.y << "," << cmd.acceleration.z << ","
              << odom_.pose.pose.position.x << "," << odom_.pose.pose.position.y << ","
              << odom_.pose.pose.position.z << ","
              << odom_vx << "," << odom_vy << "," << odom_vz << ","
              << err_x << "," << err_y << "," << err_z << ","
              << std::sqrt(err_x * err_x + err_y * err_y + err_z * err_z) << ","
              << vel_err_x << "," << vel_err_y << "," << vel_err_z << ","
              << std::sqrt(vel_err_x * vel_err_x + vel_err_y * vel_err_y +
                           vel_err_z * vel_err_z)
              << ","
              << cmd.yaw << "," << odom_yaw << "," << wrapPi(odom_yaw - cmd.yaw)
              << "\n";

    ++rows_written_;
    if (rows_written_ % 20 == 0)
    {
      log_file_.flush();
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber odom_sub_;
  ros::Subscriber trigger_sub_;
  ros::Publisher cmd_pub_;
  ros::Publisher desired_pos_pub_;
  ros::Publisher desired_vel_pub_;
  ros::Publisher desired_acc_pub_;
  ros::Publisher trajectory_vis_pub_;
  ros::Publisher tracking_error_pub_;

  nav_msgs::Odometry odom_;
  bool have_odom_;
  bool active_;
  bool auto_start_;
  bool wait_for_trigger_;
  bool center_from_odom_;
  bool face_velocity_;
  bool control_yaw_;
  bool write_csv_;
  bool publish_visualization_;
  int max_path_points_;
  uint32_t trajectory_id_;
  size_t rows_written_;

  double amplitude_x_;
  double amplitude_y_;
  double period_;
  double publish_rate_;
  double prestart_hold_time_;
  double ramp_time_;
  double duration_;
  double num_periods_;
  bool shutdown_after_finish_;
  double poststop_hold_time_;
  double center_x_;
  double center_y_;
  double center_z_;
  double yaw_;
  ros::Time start_time_;

  std::string frame_id_;
  std::string log_dir_;
  std::ofstream log_file_;
  visualization_msgs::Marker trajectory_marker_;
  std::vector<geometry_msgs::Point> desired_points_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "eight_trajectory_cmd_node");
  EightTrajectoryCmdNode node;
  node.spin();
  return 0;
}
