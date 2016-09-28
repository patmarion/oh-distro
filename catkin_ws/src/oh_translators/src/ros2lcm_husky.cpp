// ### ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Bool.h>

// ### Standard includes
#include <cstdlib>
#include <sys/time.h>
#include <time.h>
#include <iostream>
#include <map>
#include <string>

#include <lcmtypes/bot_core.hpp>
#include <lcm/lcm-cpp.hpp>

struct JointState {
  double position, velocity, effort;

  JointState() {
    position = 0.0;
    velocity = 0.0;
    effort = 0.0;
  };
  JointState(double position_in, double velocity_in, double effort_in) {
    position = position_in;
    velocity = velocity_in;
    effort = effort_in;
  }
};

struct Pose {
  double position[3];
  double orientation[4];  // w, x, y, z

  Pose() {
    for (int i = 0; i < 3; i++)
      position[i] = 0.0;

    orientation[0] = 1.0;
    orientation[1] = 0.0;
    orientation[2] = 0.0;
    orientation[3] = 0.0;
  }
};

class App
{
public:
  explicit App(ros::NodeHandle node, std::string husky_type);
  ~App();

private:
  lcm::LCM lcmPublish_;
  ros::NodeHandle node_;

  ros::Subscriber sick_lidar_sub_, spinning_lidar_sub_;
  void sick_lidar_cb(const sensor_msgs::LaserScanConstPtr& msg);
  void spinning_lidar_cb(const sensor_msgs::LaserScanConstPtr& msg);
  void publishLidar(const sensor_msgs::LaserScanConstPtr& msg, std::string channel);

  ros::Subscriber ekf_odom_sub_;
  void ekf_odom_cb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

  ros::Subscriber imuSensorSub_;
  void imuSensorCallback(const sensor_msgs::ImuConstPtr& msg);


  ros::Subscriber jointStatesSub_;
  void publishMultisenseState(int64_t utime, float position, float velocity);
  void jointStatesCallback(const sensor_msgs::JointStateConstPtr& msg);

  ros::Subscriber left_robotiq_sub_, right_robotiq_sub_;
  void leftRobotiqStatesCallback(const sensor_msgs::JointStateConstPtr& msg);
  void rightRobotiqStatesCallback(const sensor_msgs::JointStateConstPtr& msg);

  // Switch between Multisense and Dual Arm Huskies
  std::string husky_type_;
  bool publish_est_robot_state_from_ekf_;

  // Store joint states internally and compose EST_ROBOT_STATE
  void UpdateInternalStateFromJointStatesMsg(
      const sensor_msgs::JointStateConstPtr& msg, std::string prefix);
  void UpdateInternalStateFromJointStatesMsg(
      const sensor_msgs::JointStateConstPtr& msg);
  void PublishEstRobotStateFromInternalState(int64_t utime);
  void PublishJointState(int64_t utime, std::string channel,
                         const sensor_msgs::JointStateConstPtr& ros_msg,
                         std::string prefix);
  void PublishJointState(int64_t utime, std::string channel,
                         const sensor_msgs::JointStateConstPtr& ros_msg);

  // Joint names
  std::vector<std::string> dual_arm_husky_joints_ = {
      "front_left_wheel", "front_right_wheel", "rear_left_wheel",
      "rear_right_wheel", "husky_ptu_pan", "husky_ptu_tilt",
      "l_ur5_arm_shoulder_pan_joint", "l_ur5_arm_shoulder_lift_joint",
      "l_ur5_arm_elbow_joint", "l_ur5_arm_wrist_1_joint",
      "l_ur5_arm_wrist_2_joint", "l_ur5_arm_wrist_3_joint",
      "r_ur5_arm_shoulder_pan_joint", "r_ur5_arm_shoulder_lift_joint",
      "r_ur5_arm_elbow_joint", "r_ur5_arm_wrist_1_joint",
      "r_ur5_arm_wrist_2_joint", "r_ur5_arm_wrist_3_joint",
      "l_palm_finger_1_joint", "l_finger_1_joint_1", "l_finger_1_joint_2",
      "l_finger_1_joint_3", "l_finger_1_joint_proximal_actuating_hinge",
      "l_finger_1_joint_paraproximal_actuating_hinge",
      "l_finger_1_joint_proximal_actuating_bar",
      "l_finger_1_joint_paraproximal_bar",
      "l_finger_1_joint_median_actuating_hinge",
      "l_finger_1_joint_median_actuating_hinge_median_bar",
      "l_finger_1_joint_paramedian_hinge",
      "l_finger_1_joint_paramedian_hinge_median_bar_underactuated",
      "l_finger_1_joint_paradistal_hinge", "l_palm_finger_2_joint",
      "l_finger_2_joint_1", "l_finger_2_joint_2", "l_finger_2_joint_3",
      "l_finger_2_joint_proximal_actuating_hinge",
      "l_finger_2_joint_paraproximal_actuating_hinge",
      "l_finger_2_joint_proximal_actuating_bar",
      "l_finger_2_joint_paraproximal_bar",
      "l_finger_2_joint_median_actuating_hinge",
      "l_finger_2_joint_median_actuating_hinge_median_bar",
      "l_finger_2_joint_paramedian_hinge",
      "l_finger_2_joint_paramedian_hinge_median_bar_underactuated",
      "l_finger_2_joint_paradistal_hinge", "l_palm_finger_middle_joint",
      "l_finger_middle_joint_1", "l_finger_middle_joint_2",
      "l_finger_middle_joint_3",
      "l_finger_middle_joint_proximal_actuating_hinge",
      "l_finger_middle_joint_paraproximal_actuating_hinge",
      "l_finger_middle_joint_proximal_actuating_bar",
      "l_finger_middle_joint_paraproximal_bar",
      "l_finger_middle_joint_median_actuating_hinge",
      "l_finger_middle_joint_median_actuating_hinge_median_bar",
      "l_finger_middle_joint_paramedian_hinge",
      "l_finger_middle_joint_paramedian_hinge_median_bar_underactuated",
      "l_finger_middle_joint_paradistal_hinge", "r_palm_finger_1_joint",
      "r_finger_1_joint_1", "r_finger_1_joint_2", "r_finger_1_joint_3",
      "r_finger_1_joint_proximal_actuating_hinge",
      "r_finger_1_joint_paraproximal_actuating_hinge",
      "r_finger_1_joint_proximal_actuating_bar",
      "r_finger_1_joint_paraproximal_bar",
      "r_finger_1_joint_median_actuating_hinge",
      "r_finger_1_joint_median_actuating_hinge_median_bar",
      "r_finger_1_joint_paramedian_hinge",
      "r_finger_1_joint_paramedian_hinge_median_bar_underactuated",
      "r_finger_1_joint_paradistal_hinge", "r_palm_finger_2_joint",
      "r_finger_2_joint_1", "r_finger_2_joint_2", "r_finger_2_joint_3",
      "r_finger_2_joint_proximal_actuating_hinge",
      "r_finger_2_joint_paraproximal_actuating_hinge",
      "r_finger_2_joint_proximal_actuating_bar",
      "r_finger_2_joint_paraproximal_bar",
      "r_finger_2_joint_median_actuating_hinge",
      "r_finger_2_joint_median_actuating_hinge_median_bar",
      "r_finger_2_joint_paramedian_hinge",
      "r_finger_2_joint_paramedian_hinge_median_bar_underactuated",
      "r_finger_2_joint_paradistal_hinge", "r_palm_finger_middle_joint",
      "r_finger_middle_joint_1", "r_finger_middle_joint_2",
      "r_finger_middle_joint_3",
      "r_finger_middle_joint_proximal_actuating_hinge",
      "r_finger_middle_joint_paraproximal_actuating_hinge",
      "r_finger_middle_joint_proximal_actuating_bar",
      "r_finger_middle_joint_paraproximal_bar",
      "r_finger_middle_joint_median_actuating_hinge",
      "r_finger_middle_joint_median_actuating_hinge_median_bar",
      "r_finger_middle_joint_paramedian_hinge",
      "r_finger_middle_joint_paramedian_hinge_median_bar_underactuated",
      "r_finger_middle_joint_paradistal_hinge"};
  std::map<std::string, JointState> joint_states_;
  Pose pose_;
};

App::App(ros::NodeHandle node, std::string husky_type)
    : node_(node), husky_type_(husky_type), publish_est_robot_state_from_ekf_(false) {
  if (!lcmPublish_.good()) std::cerr << "ERROR: lcm is not good()" << std::endl;

  if (husky_type != "multisense" && husky_type != "dual_arm")
    throw std::runtime_error(
        "Unsupported mode: Only multisense and dual_arm modes supported.");

  // Multisense Husky
  if (husky_type_ == "multisense") {
    publish_est_robot_state_from_ekf_ = true;
  } else if (husky_type_ == "dual_arm") {
    for (auto& joint : dual_arm_husky_joints_)
      joint_states_.insert(std::make_pair(joint, JointState()));

    left_robotiq_sub_ = node_.subscribe("/husky_gripper_left/joint_states", 100,
                                        &App::leftRobotiqStatesCallback, this);

    right_robotiq_sub_ =
        node_.subscribe("/husky_gripper_right/joint_states", 100,
                        &App::rightRobotiqStatesCallback, this);
  }

  sick_lidar_sub_ = node_.subscribe(std::string("/sick_scan"), 100,
                                    &App::sick_lidar_cb, this);
  spinning_lidar_sub_ = node_.subscribe(std::string("/lidar_scan"), 100,
                                        &App::spinning_lidar_cb, this);
  ekf_odom_sub_ = node_.subscribe(std::string("/robot_pose_ekf/odom_combined"),
                                  100, &App::ekf_odom_cb, this);

  imuSensorSub_ = node_.subscribe(std::string("/imu/data"), 100,
                                  &App::imuSensorCallback, this);

  jointStatesSub_ = node_.subscribe(std::string("/joint_states"), 100,
                                        &App::jointStatesCallback, this);
}

App::~App() {}

void App::imuSensorCallback(const sensor_msgs::ImuConstPtr& msg)
{
  bot_core::ins_t imu;
  imu.utime = (int64_t)floor(msg->header.stamp.toNSec() / 1000);
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


void App::ekf_odom_cb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  ROS_ERROR_STREAM("ekfcp");
/*
    position: 
      x: -0.473553686959
      y: 59.9936536045
      z: 0.0
    orientation: 
      x: 0.0138799797993
      y: -0.014872795039
      z: 0.607424085735
      w: 0.794117199283
*/
  bot_core::pose_t lcm_pose_msg;
  lcm_pose_msg.utime = (int64_t)msg->header.stamp.toNSec() / 1000;  // from nsec to usec
  lcm_pose_msg.pos[0] = msg->pose.pose.position.x;
  lcm_pose_msg.pos[1] = msg->pose.pose.position.y;
  lcm_pose_msg.pos[2] = msg->pose.pose.position.z;
  lcm_pose_msg.orientation[0] = msg->pose.pose.orientation.w;
  lcm_pose_msg.orientation[1] = msg->pose.pose.orientation.x;
  lcm_pose_msg.orientation[2] = msg->pose.pose.orientation.y;
  lcm_pose_msg.orientation[3] = msg->pose.pose.orientation.z;
  lcmPublish_.publish("POSE_BODY", &lcm_pose_msg);

  // Update internal pose state
  for (int i = 0; i < 3; i++)
    pose_.position[i] = lcm_pose_msg.pos[i];
  for (int i = 0; i < 4; i++)
    pose_.orientation[i] = lcm_pose_msg.orientation[i];

  if (publish_est_robot_state_from_ekf_) { // publish EST_ROBOT_STATE (used to draw the robot)
    bot_core::robot_state_t est_robot_state_;
    est_robot_state_.utime = (int64_t)msg->header.stamp.toNSec() / 1000;  // from nsec to usec
    est_robot_state_.num_joints = 4;
    est_robot_state_.pose.translation.x = msg->pose.pose.position.x;
    est_robot_state_.pose.translation.y = msg->pose.pose.position.y;
    est_robot_state_.pose.translation.z = msg->pose.pose.position.z;
    est_robot_state_.pose.rotation.w = msg->pose.pose.orientation.w;
    est_robot_state_.pose.rotation.x = msg->pose.pose.orientation.x;
    est_robot_state_.pose.rotation.y = msg->pose.pose.orientation.y;
    est_robot_state_.pose.rotation.z = msg->pose.pose.orientation.z;
    est_robot_state_.twist.linear_velocity.x = 0.0;
    est_robot_state_.twist.linear_velocity.y = 0.0;
    est_robot_state_.twist.linear_velocity.z = 0.0;
    est_robot_state_.twist.angular_velocity.x = 0.0;
    est_robot_state_.twist.angular_velocity.y = 0.0;
    est_robot_state_.twist.angular_velocity.z = 0.0;


    est_robot_state_.joint_name.assign(est_robot_state_.num_joints, "");
    est_robot_state_.joint_position.assign(est_robot_state_.num_joints, (const float &) 0.);
    est_robot_state_.joint_velocity.assign(est_robot_state_.num_joints, (const float &) 0.);
    est_robot_state_.joint_effort.assign(est_robot_state_.num_joints, (const float &) 0.);
    est_robot_state_.joint_name[0] = "front_left_wheel";
    est_robot_state_.joint_name[1] = "front_right_wheel";
    est_robot_state_.joint_name[2] = "rear_left_wheel";
    est_robot_state_.joint_name[3] = "rear_right_wheel";

    // Initialise F/T sensors in est_robot_state_
    est_robot_state_.force_torque.l_foot_force_z = 0.0;
    est_robot_state_.force_torque.l_foot_torque_x = 0.0;
    est_robot_state_.force_torque.l_foot_torque_y = 0.0;
    est_robot_state_.force_torque.r_foot_force_z = 0.0;
    est_robot_state_.force_torque.r_foot_torque_x = 0.0;
    est_robot_state_.force_torque.r_foot_torque_y = 0.0;

    for (unsigned int i = 0; i < 3; i++) {
        est_robot_state_.force_torque.l_hand_force[i] = 0.0;
        est_robot_state_.force_torque.l_hand_torque[i] = 0.0;
        est_robot_state_.force_torque.r_hand_force[i] = 0.0;
        est_robot_state_.force_torque.r_hand_torque[i] = 0.0;
    }

    lcmPublish_.publish("EST_ROBOT_STATE", &est_robot_state_);
  }
}


void App::sick_lidar_cb(const sensor_msgs::LaserScanConstPtr& msg)
{
  publishLidar(msg, "SICK_SCAN");
}

void App::spinning_lidar_cb(const sensor_msgs::LaserScanConstPtr& msg)
{
  publishLidar(msg, "SCAN");
}


void App::publishLidar(const sensor_msgs::LaserScanConstPtr& msg, std::string channel)
{
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

void App::jointStatesCallback(const sensor_msgs::JointStateConstPtr& msg) {
  int64_t utime =
      static_cast<int64_t>(floor(msg->header.stamp.toNSec() / 1000));

  // Multisense Husky Mode: Publish Multisense Hokuyo state
  if (husky_type_ == "multisense" && msg->name.size() == 1 &&
      msg->name[0] == "hokuyo_joint")
    return publishMultisenseState(utime, msg->position[0], msg->velocity[0]);

  // Store incoming joints in internal state
  UpdateInternalStateFromJointStatesMsg(msg);

  if (husky_type_ == "dual_arm") {
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

    // We got the wheels, so publish joint state
    // NB: We use the wheels, even though at lower frequency, to trigger the
    // publishing of the generated message as these will always be available.
    // Hands or arms, which might be powered down, are not generally available
    // at
    // all times.
    // TODO: create a dedicated state sync
    if (msg->name.size() == 4) PublishEstRobotStateFromInternalState(utime);
  }
}

void App::leftRobotiqStatesCallback(
    const sensor_msgs::JointStateConstPtr& msg) {
  int64_t utime =
      static_cast<int64_t>(floor(msg->header.stamp.toNSec() / 1000));

  // Store incoming joints in internal state
  UpdateInternalStateFromJointStatesMsg(msg, "l_");

  // Publish Robotiq States
  PublishJointState(utime, "ROBOTIQ_LEFT_STATE", msg, "l_");
}

void App::rightRobotiqStatesCallback(
    const sensor_msgs::JointStateConstPtr& msg) {
  int64_t utime =
      static_cast<int64_t>(floor(msg->header.stamp.toNSec() / 1000));

  // Store incoming joints in internal state
  UpdateInternalStateFromJointStatesMsg(msg, "r_");

  // Publish Robotiq States
  PublishJointState(utime, "ROBOTIQ_RIGHT_STATE", msg, "r_");
}

void App::UpdateInternalStateFromJointStatesMsg(
    const sensor_msgs::JointStateConstPtr& msg) {
  UpdateInternalStateFromJointStatesMsg(msg, "");
}
void App::UpdateInternalStateFromJointStatesMsg(
    const sensor_msgs::JointStateConstPtr& msg, std::string prefix = "") {
  size_t num_joints = msg->name.size();

  for (size_t i = 0; i < num_joints; i++) {
    // NB: Some joint states (PTU) do not contain velocity or effort parts, this
    // catches exceptions
    std::string name = prefix + msg->name[i];
    double position = (msg->position.size() > 0) ? msg->position[i] : 0.0;
    double velocity = (msg->velocity.size() > 0) ? msg->velocity[i] : 0.0;
    double effort = (msg->effort.size() > 0) ? msg->effort[i] : 0.0;

    // Find in map, if not create
    if (joint_states_.find(name) == joint_states_.end()) {
      ROS_WARN_STREAM("Joint " << name
                               << " not found in internal state - this should "
                                  "not happen due to pre-allocation -- check "
                                  "correct pre-allocation");
      JointState new_joint_state(position, velocity, effort);
      joint_states_.insert(std::make_pair(name, new_joint_state));
    } else {
      JointState& joint_state = joint_states_[name];
      joint_state.position = position;
      joint_state.velocity = velocity;
      joint_state.effort = effort;
    }
  }
}

void App::PublishEstRobotStateFromInternalState(int64_t utime) {
  bot_core::robot_state_t msg;
  msg.utime = utime;
  msg.num_joints = joint_states_.size();
  msg.pose.translation.x = pose_.position[0];
  msg.pose.translation.y = pose_.position[1];
  msg.pose.translation.z = pose_.position[2];
  msg.pose.rotation.w = pose_.orientation[0];
  msg.pose.rotation.x = pose_.orientation[1];
  msg.pose.rotation.y = pose_.orientation[2];
  msg.pose.rotation.z = pose_.orientation[3];
  msg.twist.linear_velocity.x = 0.0;
  msg.twist.linear_velocity.y = 0.0;
  msg.twist.linear_velocity.z = 0.0;
  msg.twist.angular_velocity.x = 0.0;
  msg.twist.angular_velocity.y = 0.0;
  msg.twist.angular_velocity.z = 0.0;

  // Initialise F/T sensors in msg
  msg.force_torque.l_foot_force_z = 0.0;
  msg.force_torque.l_foot_torque_x = 0.0;
  msg.force_torque.l_foot_torque_y = 0.0;
  msg.force_torque.r_foot_force_z = 0.0;
  msg.force_torque.r_foot_torque_x = 0.0;
  msg.force_torque.r_foot_torque_y = 0.0;

  for (unsigned int i = 0; i < 3; i++) {
      msg.force_torque.l_hand_force[i] = 0.0;
      msg.force_torque.l_hand_torque[i] = 0.0;
      msg.force_torque.r_hand_force[i] = 0.0;
      msg.force_torque.r_hand_torque[i] = 0.0;
  }

  msg.joint_name.assign(msg.num_joints, "");
  msg.joint_position.assign(msg.num_joints, (const float &) 0.);
  msg.joint_velocity.assign(msg.num_joints, (const float &) 0.);
  msg.joint_effort.assign(msg.num_joints, (const float &) 0.);
  
  // Iterate over joint_states_ map
  int joint_number = 0;
  for (auto& joint : joint_states_) {
    msg.joint_name[joint_number] = joint.first;
    msg.joint_position[joint_number] = joint.second.position;
    msg.joint_velocity[joint_number] = joint.second.velocity;
    msg.joint_effort[joint_number] = joint.second.effort;
    joint_number++;
  }

  lcmPublish_.publish("EST_ROBOT_STATE", &msg);
}

void App::PublishJointState(int64_t utime, std::string channel,
                            const sensor_msgs::JointStateConstPtr& ros_msg) {
  PublishJointState(utime, channel, ros_msg, "");
}
void App::PublishJointState(int64_t utime, std::string channel,
                            const sensor_msgs::JointStateConstPtr& ros_msg,
                            std::string prefix = "") {
  bot_core::joint_state_t msg;
  msg.utime = utime;
  msg.num_joints = ros_msg->name.size();

  msg.joint_name.assign(msg.num_joints, "");
  msg.joint_position.assign(msg.num_joints, (const float&)0.);
  msg.joint_velocity.assign(msg.num_joints, (const float&)0.);
  msg.joint_effort.assign(msg.num_joints, (const float&)0.);

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

void App::publishMultisenseState(int64_t utime, float position, float velocity)
{
  bot_core::joint_state_t msg_out;
  msg_out.utime = utime;
  msg_out.joint_position.push_back(position);
  msg_out.joint_velocity.push_back(velocity);
  msg_out.joint_effort.push_back(0);
  msg_out.joint_name.push_back("hokuyo_joint");
  msg_out.num_joints = 1;
  lcmPublish_.publish("MULTISENSE_STATE", &msg_out);

  // publish message for bot_frames
  bot_core::rigid_transform_t preToPostFrame;
  preToPostFrame.utime = utime;
  preToPostFrame.trans[0] = 0;
  preToPostFrame.trans[1] = 0;
  preToPostFrame.trans[2] = 0;
  preToPostFrame.quat[0] = std::cos(position / 2);
  preToPostFrame.quat[1] = 0;
  preToPostFrame.quat[2] = 0;
  preToPostFrame.quat[3] = std::sin(position / 2);
  lcmPublish_.publish("PRE_SPINDLE_TO_POST_SPINDLE", &preToPostFrame);
}

int main(int argc, char** argv) {
  std::string husky_type;

  if (argc > 1) {
    husky_type = argv[1];
  } else {
    ROS_ERROR(
        "Need to have one additional argument: husky type (dual_arm or "
        "multisense)");
    exit(-1);
  }

  if (husky_type != "dual_arm" && husky_type != "multisense") {
    ROS_ERROR("Mode not understood: use dual_arm or multisense");
    exit(-1);
  }

  ros::init(argc, argv, "ros2lcm_husky");
  ros::NodeHandle nh;

  new App(nh, husky_type);
  ROS_INFO_STREAM("ros2lcm_husky translator ready: " << husky_type);
  ROS_ERROR_STREAM("ros2lcm_husky translator ready: " << husky_type);
  ros::spin();
  return 0;
}
