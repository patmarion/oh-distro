/*
 * lcm2ros_dual_arm_husky.cpp
 *
 *  Created on: 12 Oct 2016
 *      Author: yiming, theo, wolfgang
 */

#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <lcm/lcm-cpp.hpp>

// DRC LCM-Types
#include <lcmtypes/drc/plan_control_t.hpp>
#include <lcmtypes/drc/plan_status_t.hpp>
#include <lcmtypes/drc/robot_plan_t.hpp>

// Bot-Core LCM-Types
#include <lcmtypes/bot_core/joint_state_t.hpp>
#include <lcmtypes/bot_core/pose_t.hpp>
#include <lcmtypes/bot_core/robot_state_t.hpp>
#include <lcmtypes/bot_core/twist_t.hpp>
#include <lcmtypes/bot_core/utime_t.hpp>

// ROS-Messages
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>

inline int64_t timestamp_now() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (int64_t)tv.tv_sec * 1e6 + tv.tv_usec;
}

enum side_t { LEFT, RIGHT };

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
    TrajectoryActionClient;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseActionClient;

class LCM2ROS {
 public:
  LCM2ROS(std::shared_ptr<lcm::LCM>& lcm_in, ros::NodeHandle& nh_in);
  ~LCM2ROS() {}

 private:
  std::shared_ptr<lcm::LCM> lcm_;
  ros::NodeHandle nh_;

  void robotPlanHandler(const lcm::ReceiveBuffer* rbuf,
                        const std::string& channel,
                        const drc::robot_plan_t* msg);
  void robotPlanPauseHandler(const lcm::ReceiveBuffer* rbuf,
                             const std::string& channel,
                             const drc::plan_control_t* msg);
  bool findIdx(const drc::robot_plan_t* msg);

  void ptuCommandHandler(const lcm::ReceiveBuffer* rbuf,
                         const std::string& channel,
                         const bot_core::joint_state_t* msg);
  ros::Publisher ptu_command_pub_;

  void moveBaseGoalHandler(const lcm::ReceiveBuffer* rbuf,
                           const std::string& channel,
                           const bot_core::pose_t* msg);
  void moveBaseCancelGoalHandler(const lcm::ReceiveBuffer* rbuf,
                                 const std::string& channel,
                                 const bot_core::utime_t* msg);
  MoveBaseActionClient mb_ac_;
  int64_t move_base_last_plan_msg_utime_,
      move_base_last_plan_execution_start_utime_;
  void moveBaseExecutionGoalCallback();
  void moveBaseExecutionActiveCallback();
  void PublishMoveBasePlanStatus(int8_t plan_type, int8_t execution_status);

  void cmdVelHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                     const bot_core::twist_t* msg);
  ros::Publisher cmd_vel_pub_;

  void PublishArmJointState(int64_t utime, std::string channel,
                            const control_msgs::FollowJointTrajectoryGoal msg,
                            const drc::robot_plan_t* robot_msg);

  TrajectoryActionClient larm_ac_;
  TrajectoryActionClient rarm_ac_;
  std::map<std::string, int> larm_joints_map_;
  std::map<std::string, int> rarm_joints_map_;
  std::vector<std::string> larm_joints_;
  std::vector<std::string> rarm_joints_;
  bool hasIdx_;

  void leftArmTrajectoryExecutionGoalCallback();
  void rightArmTrajectoryExecutionGoalCallback();
  void leftArmTrajectoryExecutionActiveCallback();
  void rightArmTrajectoryExecutionActiveCallback();
  void leftArmTrajectoryExecutionPreemptCallback();
  void rightArmTrajectoryExecutionPreemptCallback();
  int64_t larm_last_plan_msg_utime_, rarm_last_plan_msg_utime_;
  int64_t larm_last_plan_execution_start_utime_,
      rarm_last_plan_execution_start_utime_;
  void PublishUR5PlanExecutionStatus(int8_t side, int8_t execution_status);
};

LCM2ROS::LCM2ROS(std::shared_ptr<lcm::LCM>& lcm_in, ros::NodeHandle& nh_in)
    : lcm_(lcm_in),
      nh_(nh_in),
      larm_ac_("/husky_left_ur5/follow_joint_trajectory", true),
      rarm_ac_("/husky_right_ur5/follow_joint_trajectory", true),
      hasIdx_(false),
      larm_last_plan_msg_utime_(0),
      rarm_last_plan_msg_utime_(0),
      larm_last_plan_execution_start_utime_(0),
      rarm_last_plan_execution_start_utime_(0),
      mb_ac_("/move_base", true),
      move_base_last_plan_msg_utime_(0),
      move_base_last_plan_execution_start_utime_(0) {
  lcm_->subscribe("COMMITTED_ROBOT_PLAN", &LCM2ROS::robotPlanHandler, this);

  lcm_->subscribe("COMMITTED_PLAN_PAUSE", &LCM2ROS::robotPlanPauseHandler,
                  this);

  lcm_->subscribe("PTU_COMMAND", &LCM2ROS::ptuCommandHandler, this);
  ptu_command_pub_ = nh_.advertise<sensor_msgs::JointState>("/cmd", 1);

  lcm_->subscribe("HUSKY_CMD_VEL", &LCM2ROS::cmdVelHandler, this);
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  lcm_->subscribe("HUSKY_MOVE_BASE_CANCEL", &LCM2ROS::moveBaseCancelGoalHandler,
                  this);

  lcm_->subscribe("HUSKY_MOVE_BASE_GOAL", &LCM2ROS::moveBaseGoalHandler, this);

  larm_joints_ = {
      "l_ur5_arm_shoulder_pan_joint", "l_ur5_arm_shoulder_lift_joint",
      "l_ur5_arm_elbow_joint",        "l_ur5_arm_wrist_1_joint",
      "l_ur5_arm_wrist_2_joint",      "l_ur5_arm_wrist_3_joint"};
  rarm_joints_ = {
      "r_ur5_arm_shoulder_pan_joint", "r_ur5_arm_shoulder_lift_joint",
      "r_ur5_arm_elbow_joint",        "r_ur5_arm_wrist_1_joint",
      "r_ur5_arm_wrist_2_joint",      "r_ur5_arm_wrist_3_joint"};
  for (int i = 0; i < larm_joints_.size(); i++)
    larm_joints_map_[larm_joints_[i]] = -1;
  for (int i = 0; i < rarm_joints_.size(); i++)
    rarm_joints_map_[rarm_joints_[i]] = -1;
}

/**
 * @brief      Receives bot_core::pose_t base goals in map-frame, translates
 *them into move_base_msgs::MoveBaseGoal ROS messages and sets them as a goal
 *using the action client.
 *
 * @param[in]  rbuf     LCM Receive Buffer
 * @param[in]  channel  LCM Channel
 * @param[in]  msg      bot_core::pose_t message
 */
void LCM2ROS::moveBaseGoalHandler(const lcm::ReceiveBuffer* rbuf,
                                  const std::string& channel,
                                  const bot_core::pose_t* msg) {
  move_base_msgs::MoveBaseGoal goal_msg;
  goal_msg.target_pose.header.stamp = ros::Time().fromSec(msg->utime * 1e-6);
  goal_msg.target_pose.header.frame_id = "map";

  goal_msg.target_pose.pose.position.x = msg->pos[0];
  goal_msg.target_pose.pose.position.y = msg->pos[1];
  goal_msg.target_pose.pose.position.z = msg->pos[2];

  goal_msg.target_pose.pose.orientation.w = msg->orientation[0];
  goal_msg.target_pose.pose.orientation.x = msg->orientation[1];
  goal_msg.target_pose.pose.orientation.y = msg->orientation[2];
  goal_msg.target_pose.pose.orientation.z = msg->orientation[3];

  move_base_last_plan_msg_utime_ = msg->utime;
  mb_ac_.sendGoal(goal_msg,
                  boost::bind(&LCM2ROS::moveBaseExecutionGoalCallback, this),
                  boost::bind(&LCM2ROS::moveBaseExecutionActiveCallback, this));
  ros::spinOnce();
}

/**
 * @brief      Callback invoked upon completion of the move base action goal.
 */
void LCM2ROS::moveBaseExecutionGoalCallback() {
  PublishMoveBasePlanStatus(drc::plan_status_t::WALKING,
                            drc::plan_status_t::EXECUTION_STATUS_FINISHED);

  move_base_last_plan_msg_utime_ = 0;
  move_base_last_plan_execution_start_utime_ = 0;
}

/**
 * @brief      Callback invoked upon start of the move base action goal.
 */
void LCM2ROS::moveBaseExecutionActiveCallback() {
  move_base_last_plan_execution_start_utime_ = timestamp_now();

  PublishMoveBasePlanStatus(drc::plan_status_t::WALKING,
                            drc::plan_status_t::EXECUTION_STATUS_EXECUTING);
}

/**
 * @brief      Receives bot_core::utime_t trigger to cancel current move_base
 *goal via the action client.
 *
 * @param[in]  rbuf     LCM Receive Buffer
 * @param[in]  channel  LCM Channel (should be "HUSKY_MOVE_BASE_CANCEL")
 * @param[in]  msg      bot_core::utime_t message used as trigger
 */
void LCM2ROS::moveBaseCancelGoalHandler(const lcm::ReceiveBuffer* rbuf,
                                        const std::string& channel,
                                        const bot_core::utime_t* msg) {
  ROS_WARN_STREAM("Cancelling move_base goal");
  mb_ac_.cancelAllGoals();
}

/**
 * @brief      Publish a drc::plan_status_t message for the base with the
 * current execution status (to be combined in another node)
 *
 * @param[in]  plan_type         The plan type (drc::plan_status_t enum)
 * @param[in]  execution_status  The execution status (drc::plan_status_t enum)
 */
void LCM2ROS::PublishMoveBasePlanStatus(int8_t plan_type,
                                        int8_t execution_status) {
  drc::plan_status_t msg_out;
  msg_out.utime = timestamp_now();
  msg_out.plan_type = plan_type;
  msg_out.execution_status = execution_status;
  msg_out.last_plan_msg_utime = move_base_last_plan_msg_utime_;
  msg_out.last_plan_start_utime = move_base_last_plan_execution_start_utime_;
  lcm_->publish("HUSKY_MOVE_BASE_PLAN_STATUS", &msg_out);
}

/**
 * @brief      Receives bot_core::twist_t LCM messages, translates them to
 *geometry_msgs::Twist ROS messages and publishes them on the /cmd_vel topic.
 *
 * @param[in]  rbuf     LCM Receive Buffer
 * @param[in]  channel  LCM Channel (should be "HUSKY_CMD_VEL")
 * @param[in]  msg      bot_core::twist_t message
 */
void LCM2ROS::cmdVelHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& channel,
                            const bot_core::twist_t* msg) {
  if (channel != "HUSKY_CMD_VEL") return;

  geometry_msgs::Twist ros_msg;
  ros_msg.linear.x = msg->linear_velocity.x;
  ros_msg.linear.y = msg->linear_velocity.y;
  ros_msg.linear.z = msg->linear_velocity.z;
  ros_msg.angular.x = msg->angular_velocity.x;
  ros_msg.angular.y = msg->angular_velocity.y;
  ros_msg.angular.z = msg->angular_velocity.z;

  ROS_INFO_STREAM("Base: Commanding linear velocity ("
                  << ros_msg.linear.x << ", " << ros_msg.linear.y << ", "
                  << ros_msg.linear.z << "), angular velocity ("
                  << ros_msg.angular.x << ", " << ros_msg.angular.y << ", "
                  << ros_msg.angular.z << ")");

  cmd_vel_pub_.publish(ros_msg);
}

void LCM2ROS::ptuCommandHandler(const lcm::ReceiveBuffer* rbuf,
                                const std::string& channel,
                                const bot_core::joint_state_t* msg) {
  sensor_msgs::JointState ros_msg;
  ros_msg.header.stamp = ros::Time().fromSec(msg->utime * 1e-6);

  ros_msg.name.resize(msg->num_joints);
  ros_msg.position.resize(msg->num_joints);
  ros_msg.velocity.resize(msg->num_joints);

  for (int i = 0; i < msg->num_joints; i++) {
    ros_msg.name[i] = msg->joint_name[i];
    ros_msg.position[i] = msg->joint_position[i];
    ros_msg.velocity[i] = msg->joint_velocity[i];
  }

  ROS_INFO_STREAM("Commanding PTU to [" << ros_msg.position[0] << ", "
                                        << ros_msg.position[1] << "]");
  ptu_command_pub_.publish(ros_msg);
}

void LCM2ROS::robotPlanHandler(const lcm::ReceiveBuffer* rbuf,
                               const std::string& channel,
                               const drc::robot_plan_t* msg) {
  ROS_INFO_STREAM("LCM2ROS received COMMITTED_ROBOT_PLAN");

  if (!hasIdx_ && !findIdx(msg)) return;

  control_msgs::FollowJointTrajectoryGoal larm_goal, rarm_goal;
  larm_goal.trajectory.header.frame_id = "base_link";
  rarm_goal.trajectory.header.frame_id = "base_link";
  larm_goal.trajectory.joint_names = larm_joints_;
  rarm_goal.trajectory.joint_names = rarm_joints_;

  larm_goal.trajectory.points.resize(msg->num_states);
  rarm_goal.trajectory.points.resize(msg->num_states);
  for (int i = 0; i < msg->num_states; i++) {
    bot_core::robot_state_t state = msg->plan[i];
    larm_goal.trajectory.points[i].positions.resize(larm_joints_.size());
    larm_goal.trajectory.points[i].velocities.resize(larm_joints_.size());
    for (int j = 0; j < larm_joints_.size(); j++) {
      larm_goal.trajectory.points[i].positions[j] =
          (double)state.joint_position[larm_joints_map_[larm_joints_[j]]];

      int i1 = (i > 0) ? (i - 1) : 0;
      int i2 = i;
      int i3 = (i < msg->num_states - 1) ? (i + 1) : (msg->num_states - 1);

      double dt1 = (msg->plan[i2].utime - msg->plan[i1].utime) * 1e-6;
      double dt2 = (msg->plan[i3].utime - msg->plan[i2].utime) * 1e-6;
      double dq1 =
          msg->plan[i2].joint_position[larm_joints_map_[larm_joints_[j]]] -
          msg->plan[i1].joint_position[larm_joints_map_[larm_joints_[j]]];
      double dq2 =
          msg->plan[i3].joint_position[larm_joints_map_[larm_joints_[j]]] -
          msg->plan[i2].joint_position[larm_joints_map_[larm_joints_[j]]];
      larm_goal.trajectory.points[i].velocities[j] =
          (dt1 * dt2 != 0) ? (dq1 / dt1 * 0.5 + dq2 / dt2 * 0.5) : 0.0;
      (double)state.joint_velocity[larm_joints_map_[larm_joints_[j]]];

      larm_goal.trajectory.points[i].time_from_start =
          ros::Duration().fromSec(state.utime * 1E-6);
    }

    rarm_goal.trajectory.points[i].positions.resize(rarm_joints_.size());
    rarm_goal.trajectory.points[i].velocities.resize(rarm_joints_.size());
    for (int j = 0; j < rarm_joints_.size(); j++) {
      rarm_goal.trajectory.points[i].positions[j] =
          (double)state.joint_position[rarm_joints_map_[rarm_joints_[j]]];
      int i1 = (i > 0) ? (i - 1) : 0;
      int i2 = i;
      int i3 = (i < msg->num_states - 1) ? (i + 1) : (msg->num_states - 1);

      double dt1 = (msg->plan[i2].utime - msg->plan[i1].utime) * 1e-6;
      double dt2 = (msg->plan[i3].utime - msg->plan[i2].utime) * 1e-6;
      double dq1 =
          msg->plan[i2].joint_position[rarm_joints_map_[rarm_joints_[j]]] -
          msg->plan[i1].joint_position[rarm_joints_map_[rarm_joints_[j]]];
      double dq2 =
          msg->plan[i3].joint_position[rarm_joints_map_[rarm_joints_[j]]] -
          msg->plan[i2].joint_position[rarm_joints_map_[rarm_joints_[j]]];

      rarm_goal.trajectory.points[i].velocities[j] =
          (dt1 * dt2 != 0) ? (dq1 / dt1 * 0.5 + dq2 / dt2 * 0.5) : 0.0;
      (double)state.joint_velocity[rarm_joints_map_[rarm_joints_[j]]];

      rarm_goal.trajectory.points[i].time_from_start =
          ros::Duration().fromSec(state.utime * 1E-6);
    }
  }

  larm_goal.trajectory.header.stamp = ros::Time::now();
  rarm_goal.trajectory.header.stamp = ros::Time::now();

  int64_t lutime = static_cast<int64_t>(
      floor(larm_goal.trajectory.header.stamp.toNSec() / 1000));
  int64_t rutime = static_cast<int64_t>(
      floor(rarm_goal.trajectory.header.stamp.toNSec() / 1000));

  // Theo's velocity debug feedback
  // Publish larm_goal and rarm_goal (position and velocities) so
  // we can analyse them
  PublishArmJointState(lutime, "LEFT_UR5_EXECUTE", larm_goal, msg);
  PublishArmJointState(rutime, "RIGHT_UR5_EXECUTE", rarm_goal, msg);

  // Callbacks: &doneCb, &activeCb, &feedbackCb)
  // TODO: Add parameters for feedback
  larm_ac_.sendGoal(
      larm_goal, boost::bind(&LCM2ROS::leftArmTrajectoryExecutionGoalCallback,
                             this),  //, _1, _2),
      boost::bind(&LCM2ROS::leftArmTrajectoryExecutionActiveCallback,
                  this));  //, _1,_2));
  rarm_ac_.sendGoal(
      rarm_goal, boost::bind(&LCM2ROS::rightArmTrajectoryExecutionGoalCallback,
                             this),  //, _1, _2),
      boost::bind(&LCM2ROS::rightArmTrajectoryExecutionActiveCallback,
                  this));  //, _1,_2));
  ROS_INFO_STREAM("Sending left arm plan with "
                  << larm_goal.trajectory.points.size() << " waypoints");
  ROS_INFO_STREAM("Sending right arm plan with "
                  << rarm_goal.trajectory.points.size() << " waypoints");

  // Set global variables for plan execution status
  larm_last_plan_msg_utime_ = msg->utime;
  rarm_last_plan_msg_utime_ = msg->utime;

  ros::spinOnce();
}

/**
 * @brief      Handles "Stop" commands as executed from Director to interrupt
 * the current trajectory execution (Soft-E-Stop)
 *
 * @param[in]  rbuf     The rbuf
 * @param[in]  channel  LCM Channel (should be "COMMITTED_PLAN_PAUSE")
 * @param[in]  msg      The message
 */
void LCM2ROS::robotPlanPauseHandler(const lcm::ReceiveBuffer* rbuf,
                                    const std::string& channel,
                                    const drc::plan_control_t* msg) {
  ROS_WARN("Cancelling left and right arm goals as well as move base goal.");
  larm_ac_.cancelAllGoals();
  rarm_ac_.cancelAllGoals();
  mb_ac_.cancelAllGoals();
}

/**
 * @brief      Actionlib-Callback called upon completion of the commanded
 * trajectory for the left arm
 */
// TODO: Add parameters as in
// http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Callback%20Based%20Simple%20Action%20Client
void LCM2ROS::leftArmTrajectoryExecutionGoalCallback() {
  ROS_INFO_STREAM("Left arm trajectory execution completed");

  PublishUR5PlanExecutionStatus(side_t::LEFT,
                                drc::plan_status_t::EXECUTION_STATUS_FINISHED);
  larm_last_plan_msg_utime_ = 0;
  larm_last_plan_execution_start_utime_ = 0;
}

/**
 * @brief      Actionlib-Callback called upon completion of the commanded
 * trajectory for the right arm
 */
void LCM2ROS::rightArmTrajectoryExecutionGoalCallback() {
  ROS_INFO_STREAM("Right arm trajectory execution completed");

  PublishUR5PlanExecutionStatus(side_t::RIGHT,
                                drc::plan_status_t::EXECUTION_STATUS_FINISHED);
  rarm_last_plan_msg_utime_ = 0;
  rarm_last_plan_execution_start_utime_ = 0;
}

/**
 * @brief      Actionlib-Callback called upon start of the commanded
 * trajectory for the left arm
 */
void LCM2ROS::leftArmTrajectoryExecutionActiveCallback() {
  ROS_INFO_STREAM("Left arm trajectory execution started");

  larm_last_plan_execution_start_utime_ = timestamp_now();
  PublishUR5PlanExecutionStatus(side_t::LEFT,
                                drc::plan_status_t::EXECUTION_STATUS_EXECUTING);
}

/**
 * @brief      Actionlib-Callback called upon start of the commanded
 * trajectory for the right arm
 */
void LCM2ROS::rightArmTrajectoryExecutionActiveCallback() {
  ROS_INFO_STREAM("Right arm trajectory execution started");

  rarm_last_plan_execution_start_utime_ = timestamp_now();
  PublishUR5PlanExecutionStatus(side_t::RIGHT,
                                drc::plan_status_t::EXECUTION_STATUS_EXECUTING);
}

/**
 * @brief      Actionlib-Callback called upon preemption of the commanded
 * trajectory for the left arm
 */
void LCM2ROS::leftArmTrajectoryExecutionPreemptCallback() {
  ROS_INFO_STREAM("Left arm trajectory execution preempted");

  PublishUR5PlanExecutionStatus(side_t::LEFT,
                                drc::plan_status_t::EXECUTION_STATUS_NO_PLAN);
}

/**
 * @brief      Actionlib-Callback called upon preemption of the commanded
 * trajectory for the right arm
 */
void LCM2ROS::rightArmTrajectoryExecutionPreemptCallback() {
  ROS_INFO_STREAM("Right arm trajectory execution preempted");

  PublishUR5PlanExecutionStatus(side_t::RIGHT,
                                drc::plan_status_t::EXECUTION_STATUS_NO_PLAN);
}

/**
 * @brief      Publish a drc::plan_status_t message for the arms with the
 * current execution status (to be combined in another node)
 *
 * @param[in]  side              The side (side_t enum - left or right)
 * @param[in]  execution_status  The execution status (drc::plan_status_t enum)
 */
void LCM2ROS::PublishUR5PlanExecutionStatus(int8_t side,
                                            int8_t execution_status) {
  drc::plan_status_t msg_out;
  msg_out.utime = timestamp_now();
  msg_out.plan_type = drc::plan_status_t::MANIPULATING;
  msg_out.execution_status = execution_status;

  if (side == side_t::LEFT) {
    msg_out.last_plan_msg_utime = larm_last_plan_msg_utime_;
    msg_out.last_plan_start_utime = larm_last_plan_execution_start_utime_;
    lcm_->publish("LEFT_UR5_PLAN_STATUS", &msg_out);
  } else if (side == side_t::RIGHT) {
    msg_out.last_plan_msg_utime = rarm_last_plan_msg_utime_;
    msg_out.last_plan_start_utime = rarm_last_plan_execution_start_utime_;
    lcm_->publish("RIGHT_UR5_PLAN_STATUS", &msg_out);
  }
}

bool LCM2ROS::findIdx(const drc::robot_plan_t* msg) {
  for (int i = 0; i < msg->plan[0].joint_name.size(); i++) {
    if (larm_joints_map_.find(msg->plan[0].joint_name[i]) !=
        larm_joints_map_.end())
      larm_joints_map_[msg->plan[0].joint_name[i]] = i;
    else if (rarm_joints_map_.find(msg->plan[0].joint_name[i]) !=
             rarm_joints_map_.end())
      rarm_joints_map_[msg->plan[0].joint_name[i]] = i;
  }
  for (auto& it : larm_joints_map_)
    if (it.second == -1) {
      ROS_ERROR_STREAM("Joint " << it.first << " does not exist in LCM plan");
      return false;
    }
  for (auto& it : rarm_joints_map_)
    if (it.second == -1) {
      ROS_ERROR_STREAM("Joint " << it.first << " does not exist in LCM plan");
      return false;
    }
  hasIdx_ = true;
  return hasIdx_;
}

void LCM2ROS::PublishArmJointState(
    int64_t utime, std::string channel,
    const control_msgs::FollowJointTrajectoryGoal msg,
    const drc::robot_plan_t* robot_msg) {
  // arm
  size_t num_waypoints = msg.trajectory.points.size();

  bot_core::joint_state_t lcm_msg;

  int64_t secs =
      static_cast<int64_t>(floor(msg.trajectory.header.stamp.toNSec() / 1000));

  // waypoints in the trajectory
  for (int i = 0; i < num_waypoints; i++) {
    // debugging

    // ROS_INFO_STREAM(
    //  "Publisher right arm plan with
    // "<<floor(msg.trajectory.points[i].time_from_start.toSec())<<"
    // waypoints");
    // ROS_INFO_STREAM(
    //  "Utime "<< utime << " ros time "<< secs );
    //  ROS_INFO_STREAM(
    //  "Utime plan "<<robot_msg->plan[i].utime);

    // time of the message exactly the same as the time-indexing of the planned
    // motion
    lcm_msg.utime = secs + robot_msg->plan[i].utime;

    // set the number of joints for the message
    lcm_msg.num_joints = msg.trajectory.joint_names.size();

    // initialisations
    lcm_msg.joint_name.assign(lcm_msg.num_joints, "");
    lcm_msg.joint_position.assign(lcm_msg.num_joints, (const float&)0.);
    lcm_msg.joint_velocity.assign(lcm_msg.num_joints, (const float&)0.);
    lcm_msg.joint_effort.assign(lcm_msg.num_joints, (const float&)0.);

    // Iterate over joints and set positions and velocities
    for (int joint_number = 0; joint_number < lcm_msg.num_joints;
         joint_number++) {
      std::string name = msg.trajectory.joint_names[joint_number];

      double position = (msg.trajectory.points.size() > 0)
                            ? msg.trajectory.points[i].positions[joint_number]
                            : 0.0;
      double velocity = (msg.trajectory.points.size() > 0)
                            ? msg.trajectory.points[i].velocities[joint_number]
                            : 0.0;
      double effort = (msg.trajectory.points.size() > 0) ? 0.0 : 0.0;

      lcm_msg.joint_name[joint_number] = name;
      lcm_msg.joint_position[joint_number] = position;
      lcm_msg.joint_velocity[joint_number] = velocity;
      lcm_msg.joint_effort[joint_number] = effort;
    }

    // publish the message
    lcm_->publish(channel, &lcm_msg);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "lcm2ros_dual_arm_husky",
            ros::init_options::NoSigintHandler);
  std::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if (!lcm->good()) {
    std::cerr << "ERROR: lcm is not good()" << std::endl;
  }
  ros::NodeHandle nh;

  LCM2ROS handlerObject(lcm, nh);
  ROS_INFO_STREAM("LCM2ROS Dual Arm Husky Translator Ready");

  while (0 == lcm->handle()) {
  }

  return 0;
}
