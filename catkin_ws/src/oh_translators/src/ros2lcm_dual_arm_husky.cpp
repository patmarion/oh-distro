// ### ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <robotiq_force_torque_sensor_msgs/ft_sensor.h>
#include <husky_msgs/HuskyStatus.h>

// ### Standard includes
#include <cstdlib>
#include <sys/time.h>
#include <time.h>
#include <iostream>
#include <map>
#include <string>

#include <lcmtypes/bot_core/ins_t.hpp>
#include <lcmtypes/bot_core/joint_state_t.hpp>
#include <lcmtypes/bot_core/planar_lidar_t.hpp>
#include <lcmtypes/bot_core/pose_t.hpp>
#include <lcmtypes/bot_core/six_axis_force_torque_t.hpp>

#include <lcmtypes/husky/husky_status_t.hpp>
#include <lcm/lcm-cpp.hpp>
#include <bot_lcmgl_client/lcmgl.h>

inline int64_t timestamp_now() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (int64_t)tv.tv_sec * 1e6 + tv.tv_usec;
}

class App {
 public:
  explicit App(ros::NodeHandle node);
  ~App();

 private:
  lcm::LCM lcmPublish_;
  ros::NodeHandle node_;

  ros::Subscriber sick_lidar_sub_, spinning_lidar_sub_, cost_map_sub_;
  void sick_lidar_cb(const sensor_msgs::LaserScanConstPtr &msg);
  void spinning_lidar_cb(const sensor_msgs::LaserScanConstPtr &msg);
  void cost_map_cb(const nav_msgs::OccupancyGridConstPtr &msg);
  void publishLidar(const sensor_msgs::LaserScanConstPtr &msg,
                    std::string channel);

  ros::Subscriber wheel_odom_sub_, ekf_odom_sub_;
  void wheel_odometry_cb(const nav_msgs::OdometryConstPtr &msg);
  void ekf_odometry_cb(const nav_msgs::OdometryConstPtr &msg);

  ros::Subscriber imuSensorSub_;
  void imuSensorCallback(const sensor_msgs::ImuConstPtr &msg);

  ros::Subscriber jointStatesSub_;
  void jointStatesCallback(const sensor_msgs::JointStateConstPtr &msg);

  ros::Subscriber left_robotiq_sub_, right_robotiq_sub_;
  void leftRobotiqStatesCallback(const sensor_msgs::JointStateConstPtr &msg);
  void rightRobotiqStatesCallback(const sensor_msgs::JointStateConstPtr &msg);

  ros::Subscriber left_ft_sub_, right_ft_sub_;
  void leftRobotiqForceTorqueCallback(
      const robotiq_force_torque_sensor_msgs::ft_sensor &msg);
  void rightRobotiqForceTorqueCallback(
      const robotiq_force_torque_sensor_msgs::ft_sensor &msg);

  ros::Subscriber husky_status_sub_;
  void huskyStatusCallback(const husky_msgs::HuskyStatusConstPtr &msg);

  void PublishJointState(int64_t utime, std::string channel,
                         const sensor_msgs::JointStateConstPtr &ros_msg,
                         std::string prefix);
  void PublishJointState(int64_t utime, std::string channel,
                         const sensor_msgs::JointStateConstPtr &ros_msg);

};

App::App(ros::NodeHandle node) : node_(node) {
  if (!lcmPublish_.good()) std::cerr << "ERROR: lcm is not good()" << std::endl;

  left_robotiq_sub_ = node_.subscribe("/husky_gripper_left/joint_states", 100,
                                      &App::leftRobotiqStatesCallback, this);

  right_robotiq_sub_ = node_.subscribe("/husky_gripper_right/joint_states", 100,
                                       &App::rightRobotiqStatesCallback, this);

  left_ft_sub_ =
      node_.subscribe("/husky_left_gripper/robotiq_force_torque_sensor", 100,
                      &App::leftRobotiqForceTorqueCallback, this);
  right_ft_sub_ =
      node_.subscribe("/husky_right_gripper/robotiq_force_torque_sensor", 100,
                      &App::rightRobotiqForceTorqueCallback, this);

  husky_status_sub_ =
      node_.subscribe("/status", 100, &App::huskyStatusCallback, this);

  sick_lidar_sub_ =
      node_.subscribe(std::string("/scan"), 100, &App::sick_lidar_cb, this);
  spinning_lidar_sub_ = node_.subscribe(std::string("/lidar_scan"), 100,
                                        &App::spinning_lidar_cb, this);

  ekf_odom_sub_ = node_.subscribe(std::string("/odometry/filtered"), 100,
                                  &App::ekf_odometry_cb, this);

  wheel_odom_sub_ =
      node_.subscribe(std::string("/husky_velocity_controller/odom"), 100,
                      &App::wheel_odometry_cb, this);

  imuSensorSub_ = node_.subscribe(std::string("/imu/data"), 100,
                                  &App::imuSensorCallback, this);

  jointStatesSub_ = node_.subscribe(std::string("/joint_states"), 100,
                                    &App::jointStatesCallback, this);

  cost_map_sub_ = node_.subscribe(
      std::string("/move_base/global_costmap/costmap"), 10, &App::cost_map_cb,
      this);
}

App::~App() {}

void App::imuSensorCallback(const sensor_msgs::ImuConstPtr &msg) {
  int64_t utime =
      static_cast<int64_t>(floor(msg->header.stamp.toNSec() / 1000));

  bot_core::ins_t imu;
  imu.utime = utime;
  imu.device_time = imu.utime;
  imu.gyro[0] = msg->angular_velocity.x;
  imu.gyro[1] = msg->angular_velocity.y;
  imu.gyro[2] = msg->angular_velocity.z;
  imu.mag[0] = 0;
  imu.mag[1] = 0;
  imu.mag[2] = 0;
  imu.accel[0] = msg->linear_acceleration.x;
  imu.accel[1] = msg->linear_acceleration.y;
  imu.accel[2] = msg->linear_acceleration.z;
  imu.quat[0] = msg->orientation.w;
  imu.quat[1] = msg->orientation.x;
  imu.quat[2] = msg->orientation.y;
  imu.quat[3] = msg->orientation.z;
  imu.pressure = 0;
  imu.rel_alt = 0;

  lcmPublish_.publish("IMU_MICROSTRAIN", &imu);
}

void App::leftRobotiqForceTorqueCallback(
    const robotiq_force_torque_sensor_msgs::ft_sensor &msg) {
  bot_core::six_axis_force_torque_t lcm_msg;
  lcm_msg.utime = timestamp_now();

  lcm_msg.force[0] = msg.Fx;
  lcm_msg.force[1] = msg.Fy;
  lcm_msg.force[2] = msg.Fz;

  lcm_msg.moment[0] = msg.Mx;
  lcm_msg.moment[1] = msg.My;
  lcm_msg.moment[2] = msg.Mz;

  lcmPublish_.publish("LEFT_FORCE_TORQUE", &lcm_msg);
}

void App::rightRobotiqForceTorqueCallback(
    const robotiq_force_torque_sensor_msgs::ft_sensor &msg) {
  bot_core::six_axis_force_torque_t lcm_msg;
  lcm_msg.utime = timestamp_now();

  lcm_msg.force[0] = msg.Fx;
  lcm_msg.force[1] = msg.Fy;
  lcm_msg.force[2] = msg.Fz;

  lcm_msg.moment[0] = msg.Mx;
  lcm_msg.moment[1] = msg.My;
  lcm_msg.moment[2] = msg.Mz;

  lcmPublish_.publish("RIGHT_FORCE_TORQUE", &lcm_msg);
}

void App::huskyStatusCallback(const husky_msgs::HuskyStatusConstPtr &msg) {
  int64_t utime =
      static_cast<int64_t>(floor(msg->header.stamp.toNSec() / 1000));

  husky::husky_status_t lcm_msg;
  lcm_msg.utime = utime;

  lcm_msg.uptime = msg->uptime;

  lcm_msg.ros_control_loop_freq = msg->ros_control_loop_freq;

  lcm_msg.mcu_and_user_port_current = msg->mcu_and_user_port_current;
  lcm_msg.left_driver_current = msg->left_driver_current;
  lcm_msg.right_driver_current = msg->right_driver_current;

  lcm_msg.battery_voltage = msg->battery_voltage;
  lcm_msg.left_driver_voltage = msg->left_driver_voltage;
  lcm_msg.right_driver_voltage = msg->right_driver_voltage;

  lcm_msg.left_driver_temp = msg->left_driver_temp;
  lcm_msg.right_driver_temp = msg->right_driver_temp;
  lcm_msg.left_motor_temp = msg->left_motor_temp;
  lcm_msg.right_motor_temp = msg->right_motor_temp;

  lcm_msg.capacity_estimate = msg->capacity_estimate;
  lcm_msg.charge_estimate = msg->charge_estimate;

  lcm_msg.timeout = msg->timeout;
  lcm_msg.lockout = msg->lockout;
  lcm_msg.e_stop = msg->e_stop;
  lcm_msg.ros_pause = msg->ros_pause;
  lcm_msg.no_battery = msg->no_battery;
  lcm_msg.current_limit = msg->current_limit;

  lcmPublish_.publish("HUSKY_STATUS", &lcm_msg);
}

void App::wheel_odometry_cb(const nav_msgs::OdometryConstPtr &msg) {
  int64_t utime =
      static_cast<int64_t>(floor(msg->header.stamp.toNSec() / 1000));

  bot_core::pose_t lcm_pose_msg;
  lcm_pose_msg.utime = utime;
  lcm_pose_msg.pos[0] = msg->pose.pose.position.x;
  lcm_pose_msg.pos[1] = msg->pose.pose.position.y;
  lcm_pose_msg.pos[2] = msg->pose.pose.position.z;
  lcm_pose_msg.orientation[0] = msg->pose.pose.orientation.w;
  lcm_pose_msg.orientation[1] = msg->pose.pose.orientation.x;
  lcm_pose_msg.orientation[2] = msg->pose.pose.orientation.y;
  lcm_pose_msg.orientation[3] = msg->pose.pose.orientation.z;
  lcmPublish_.publish("POSE_BODY_WHEEL_ODOMETRY", &lcm_pose_msg);
}

void App::ekf_odometry_cb(const nav_msgs::OdometryConstPtr &msg) {
  int64_t utime =
      static_cast<int64_t>(floor(msg->header.stamp.toNSec() / 1000));

  bot_core::pose_t lcm_pose_msg;
  lcm_pose_msg.utime = utime;
  lcm_pose_msg.pos[0] = msg->pose.pose.position.x;
  lcm_pose_msg.pos[1] = msg->pose.pose.position.y;
  lcm_pose_msg.pos[2] = msg->pose.pose.position.z;
  lcm_pose_msg.orientation[0] = msg->pose.pose.orientation.w;
  lcm_pose_msg.orientation[1] = msg->pose.pose.orientation.x;
  lcm_pose_msg.orientation[2] = msg->pose.pose.orientation.y;
  lcm_pose_msg.orientation[3] = msg->pose.pose.orientation.z;
  lcmPublish_.publish("POSE_BODY_EKF", &lcm_pose_msg);
}

void App::sick_lidar_cb(const sensor_msgs::LaserScanConstPtr &msg) {
  publishLidar(msg, "SICK_SCAN");
}

void App::spinning_lidar_cb(const sensor_msgs::LaserScanConstPtr &msg) {
  publishLidar(msg, "SCAN");
}

void App::publishLidar(const sensor_msgs::LaserScanConstPtr &msg,
                       std::string channel) {
  bot_core::planar_lidar_t scan_out;
  scan_out.ranges = msg->ranges;
  scan_out.intensities = msg->intensities;
  scan_out.utime = (int64_t)floor(msg->header.stamp.toNSec() / 1000);
  scan_out.nranges = msg->ranges.size();
  scan_out.nintensities = msg->intensities.size();
  scan_out.rad0 = msg->angle_min;
  scan_out.radstep = msg->angle_increment;
  lcmPublish_.publish(channel.c_str(), &scan_out);
}

void App::jointStatesCallback(const sensor_msgs::JointStateConstPtr &msg) {
  int64_t utime =
      static_cast<int64_t>(floor(msg->header.stamp.toNSec() / 1000));

  // Publish UR5
  std::string left_ur5_prefix = "l_ur5_arm";
  std::string right_ur5_prefix = "r_ur5_arm";

  if (msg->name[0].substr(0, left_ur5_prefix.size()) == left_ur5_prefix)
    PublishJointState(utime, "LEFT_UR5_STATE", msg);

  if (msg->name[0].substr(0, right_ur5_prefix.size()) == right_ur5_prefix)
    PublishJointState(utime, "RIGHT_UR5_STATE", msg);

  // Publish PTU State
  std::string ptu_prefix = "husky_ptu";
  if (msg->name[0].substr(0, ptu_prefix.size()) == ptu_prefix)
    PublishJointState(utime, "PTU_STATE", msg);

  // Publish Wheel State
  std::string wheel_suffix = "wheel";
  if (msg->name[0].substr(msg->name[0].size() - wheel_suffix.size(),
                          wheel_suffix.size()) == wheel_suffix)
    PublishJointState(utime, "WHEEL_STATE", msg);
}

void App::leftRobotiqStatesCallback(
    const sensor_msgs::JointStateConstPtr &msg) {
  int64_t utime =
      static_cast<int64_t>(floor(msg->header.stamp.toNSec() / 1000));

  // Publish Robotiq States
  PublishJointState(utime, "ROBOTIQ_LEFT_STATE", msg, "l_");
}

void App::rightRobotiqStatesCallback(
    const sensor_msgs::JointStateConstPtr &msg) {
  int64_t utime =
      static_cast<int64_t>(floor(msg->header.stamp.toNSec() / 1000));

  // Publish Robotiq States
  PublishJointState(utime, "ROBOTIQ_RIGHT_STATE", msg, "r_");
}

void App::PublishJointState(int64_t utime, std::string channel,
                            const sensor_msgs::JointStateConstPtr &ros_msg) {
  PublishJointState(utime, channel, ros_msg, "");
}
void App::PublishJointState(int64_t utime, std::string channel,
                            const sensor_msgs::JointStateConstPtr &ros_msg,
                            std::string prefix = "") {
  bot_core::joint_state_t msg;
  msg.utime = utime;
  msg.num_joints = ros_msg->name.size();

  msg.joint_name.assign(msg.num_joints, "");
  msg.joint_position.assign(msg.num_joints, (const float &)0.);
  msg.joint_velocity.assign(msg.num_joints, (const float &)0.);
  msg.joint_effort.assign(msg.num_joints, (const float &)0.);

  // Iterate over joint_states_ map
  for (int joint_number = 0; joint_number < msg.num_joints; joint_number++) {
    // NB: Some joint states (PTU) do not contain velocity or effort parts, this
    // catches exceptions
    std::string name = prefix + ros_msg->name[joint_number];
    double position =
        (ros_msg->position.size() > 0) ? ros_msg->position[joint_number] : 0.0;
    double velocity =
        (ros_msg->velocity.size() > 0) ? ros_msg->velocity[joint_number] : 0.0;
    double effort =
        (ros_msg->effort.size() > 0) ? ros_msg->effort[joint_number] : 0.0;

    msg.joint_name[joint_number] = name;
    msg.joint_position[joint_number] = position;
    msg.joint_velocity[joint_number] = velocity;
    msg.joint_effort[joint_number] = effort;
  }

  lcmPublish_.publish(channel, &msg);
}

void App::cost_map_cb(const nav_msgs::OccupancyGridConstPtr &msg)
{
  static bot_lcmgl_t* lcmgl = bot_lcmgl_init(lcmPublish_.getUnderlyingLCM(),
      "NavigationCostMap");
  int size = msg->info.width*msg->info.height;
  int cnt = 0;
  int reduce_size = 3, reduce_w = 0, reduce_h = 0;
  for(int w=0;w<msg->info.width;w++)
  {
    if (reduce_w < reduce_size)
    {
      reduce_w++;
      cnt+=msg->info.height;
    }
    else
    {
      reduce_w = 0;
      for (int h = 0; h < msg->info.height; h++)
      {
        if(reduce_h<reduce_size)
        {
          reduce_h++;
        }
        else
        {
          reduce_h = 0;
          if (msg->data[cnt] > 0)
          {
            bot_lcmgl_color4f(lcmgl, msg->data[cnt] / 100.0,
                (100.0 - msg->data[cnt]) / 100.0,
                (100.0 - msg->data[cnt]) / 100.0,
                msg->data[cnt] > 50 ? 1.0 : 0.5);
            double xyz[3] = { msg->info.origin.position.x
                + (double) h * msg->info.resolution, msg->info.origin.position.y
                + (double) w * msg->info.resolution, msg->data[cnt] / 400.0 };
            float size[3] = { msg->info.resolution * reduce_size,
                msg->info.resolution * reduce_size, msg->info.resolution *reduce_size };
            bot_lcmgl_box(lcmgl, xyz, size);
          }
        }
        cnt++;
      }
    }
  }
  bot_lcmgl_switch_buffer(lcmgl);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ros2lcm_dual_arm_husky");
  ros::NodeHandle nh;

  new App(nh);
  ROS_INFO_STREAM("ros2lcm_dual_arm_husky translator ready");
  ros::spin();
  return 0;
}
