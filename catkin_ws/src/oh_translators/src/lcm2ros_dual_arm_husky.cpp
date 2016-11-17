/*
 * lcm2ros_dual_arm_husky.cpp
 *
 *  Created on: 12 Oct 2016
 *      Author: yiming
 */

#include <cstdlib>
#include <memory>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <lcm/lcm-cpp.hpp>

#include "lcmtypes/drc/robot_plan_t.hpp"
#include "lcmtypes/drc/plan_control_t.hpp"
#include "lcmtypes/bot_core/robot_state_t.hpp"
#include <trajectory_msgs/JointTrajectory.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

class LCM2ROS
{
  public:
    LCM2ROS(std::shared_ptr<lcm::LCM> &lcm_in, ros::NodeHandle &nh_in);
    ~LCM2ROS()
    {
    }

  private:
    std::shared_ptr<lcm::LCM> lcm_;
    ros::NodeHandle nh_;

    void robotPlanHandler(const lcm::ReceiveBuffer* rbuf,
        const std::string &channel, const drc::robot_plan_t* msg);
    void robotPlanPauseHandler(const lcm::ReceiveBuffer* rbuf,
        const std::string &channel, const drc::plan_control_t* msg);
    bool findIdx(const drc::robot_plan_t* msg);

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> larm_ac_;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> rarm_ac_;
    std::map<std::string, int> larm_joints_map_;
    std::map<std::string, int> rarm_joints_map_;
    std::vector<std::string> larm_joints_;
    std::vector<std::string> rarm_joints_;
    bool hasIdx_;
};

LCM2ROS::LCM2ROS(std::shared_ptr<lcm::LCM> &lcm_in, ros::NodeHandle &nh_in)
    : lcm_(lcm_in), nh_(nh_in), larm_ac_(
        "/husky_left_ur5/follow_joint_trajectory", true), rarm_ac_(
        "/husky_right_ur5/follow_joint_trajectory", true), hasIdx_(false)
{
  lcm_->subscribe("COMMITTED_ROBOT_PLAN", &LCM2ROS::robotPlanHandler, this);

  lcm_->subscribe("COMMITTED_PLAN_PAUSE", &LCM2ROS::robotPlanPauseHandler,
      this);

  larm_joints_ =
  { "l_ur5_arm_shoulder_pan_joint",
    "l_ur5_arm_shoulder_lift_joint", "l_ur5_arm_elbow_joint",
    "l_ur5_arm_wrist_1_joint", "l_ur5_arm_wrist_2_joint",
    "l_ur5_arm_wrist_3_joint"};
  rarm_joints_ =
  { "r_ur5_arm_shoulder_pan_joint",
    "r_ur5_arm_shoulder_lift_joint", "r_ur5_arm_elbow_joint",
    "r_ur5_arm_wrist_1_joint", "r_ur5_arm_wrist_2_joint",
    "r_ur5_arm_wrist_3_joint"};
  for (int i = 0; i < larm_joints_.size(); i++)
    larm_joints_map_[larm_joints_[i]] = -1;
  for (int i = 0; i < rarm_joints_.size(); i++)
    rarm_joints_map_[rarm_joints_[i]] = -1;
}

void LCM2ROS::robotPlanHandler(const lcm::ReceiveBuffer* rbuf,
    const std::string &channel, const drc::robot_plan_t* msg)
{
  ROS_ERROR("LCM2ROS got plan");

  if (!hasIdx_ && !findIdx(msg)) return;

  control_msgs::FollowJointTrajectoryGoal larm_goal, rarm_goal;
  larm_goal.trajectory.header.frame_id = "base_link";
  rarm_goal.trajectory.header.frame_id = "base_link";
  larm_goal.trajectory.joint_names = larm_joints_;
  rarm_goal.trajectory.joint_names = rarm_joints_;

  larm_goal.trajectory.points.resize(msg->num_states);
  rarm_goal.trajectory.points.resize(msg->num_states);
  for (int i = 0; i < msg->num_states; i++)
  {
    bot_core::robot_state_t state = msg->plan[i];
    larm_goal.trajectory.points[i].positions.resize(larm_joints_.size());
    larm_goal.trajectory.points[i].velocities.resize(larm_joints_.size());
    for (int j = 0; j < larm_joints_.size(); j++)
    {
      larm_goal.trajectory.points[i].positions[j] =
          (double) state.joint_position[larm_joints_map_[larm_joints_[j]]];

      int i1 = (i > 0) ? (i - 1) : 0;
      int i2 = i;
      int i3 = (i < msg->num_states - 1) ? (i + 1) : (msg->num_states - 1);

      double dt1 = (msg->plan[i2].utime - msg->plan[i1].utime) * 1e-6;
      double dt2 = (msg->plan[i3].utime - msg->plan[i2].utime) * 1e-6;
      double dq1 =
          msg->plan[i2].joint_position[larm_joints_map_[larm_joints_[j]]]
              - msg->plan[i1].joint_position[larm_joints_map_[larm_joints_[j]]];
      double dq2 =
          msg->plan[i3].joint_position[larm_joints_map_[larm_joints_[j]]]
              - msg->plan[i2].joint_position[larm_joints_map_[larm_joints_[j]]];
      larm_goal.trajectory.points[i].velocities[j] =
          (dt1 * dt2 != 0) ? (dq1 / dt1 * 0.5 + dq2 / dt2 * 0.5) : 0.0;
      (double) state.joint_velocity[larm_joints_map_[larm_joints_[j]]];

      larm_goal.trajectory.points[i].time_from_start = ros::Duration().fromSec(
          state.utime * 1E-6);

    }

    rarm_goal.trajectory.points[i].positions.resize(rarm_joints_.size());
    rarm_goal.trajectory.points[i].velocities.resize(rarm_joints_.size());
    for (int j = 0; j < rarm_joints_.size(); j++)
    {
      rarm_goal.trajectory.points[i].positions[j] =
          (double) state.joint_position[rarm_joints_map_[rarm_joints_[j]]];
      int i1 = (i > 0) ? (i - 1) : 0;
      int i2 = i;
      int i3 = (i < msg->num_states - 1) ? (i + 1) : (msg->num_states - 1);

      double dt1 = (msg->plan[i2].utime - msg->plan[i1].utime) * 1e-6;
      double dt2 = (msg->plan[i3].utime - msg->plan[i2].utime) * 1e-6;
      double dq1 =
          msg->plan[i2].joint_position[larm_joints_map_[rarm_joints_[j]]]
              - msg->plan[i1].joint_position[larm_joints_map_[rarm_joints_[j]]];
      double dq2 =
          msg->plan[i3].joint_position[larm_joints_map_[rarm_joints_[j]]]
              - msg->plan[i2].joint_position[larm_joints_map_[rarm_joints_[j]]];
      rarm_goal.trajectory.points[i].velocities[j] =
          (dt1 * dt2 != 0) ? (dq1 / dt1 * 0.5 + dq2 / dt2 * 0.5) : 0.0;
      (double) state.joint_velocity[larm_joints_map_[rarm_joints_[j]]];

      rarm_goal.trajectory.points[i].time_from_start = ros::Duration().fromSec(
          state.utime * 1E-6);
    }
  }

  larm_goal.trajectory.header.stamp = ros::Time::now();
  rarm_goal.trajectory.header.stamp = ros::Time::now();

  larm_ac_.sendGoal(larm_goal);
  rarm_ac_.sendGoal(rarm_goal);
  ROS_INFO_STREAM(
      "Sending left arm plan with "<<larm_goal.trajectory.points.size()<<" waypoints");
  ROS_INFO_STREAM(
      "Sending right arm plan with "<<rarm_goal.trajectory.points.size()<<" waypoints");
  ros::spinOnce();
}

void LCM2ROS::robotPlanPauseHandler(const lcm::ReceiveBuffer* rbuf,
    const std::string &channel, const drc::plan_control_t* msg)
{
  ROS_WARN("Cancelling left and right arm goals.");
  larm_ac_.cancelGoal();
  rarm_ac_.cancelGoal();
}

bool LCM2ROS::findIdx(const drc::robot_plan_t* msg)
{
  for (int i = 0; i < msg->plan[0].joint_name.size(); i++)
  {
    if (larm_joints_map_.find(msg->plan[0].joint_name[i])
        != larm_joints_map_.end())
      larm_joints_map_[msg->plan[0].joint_name[i]] = i;
    else if (rarm_joints_map_.find(msg->plan[0].joint_name[i])
        != rarm_joints_map_.end())
      rarm_joints_map_[msg->plan[0].joint_name[i]] = i;
  }
  for (auto &it : larm_joints_map_)
    if (it.second == -1)
    {
      ROS_ERROR_STREAM("Joint "<<it.first<<" does not exist in LCM plan");
      return false;
    }
  for (auto &it : rarm_joints_map_)
    if (it.second == -1)
    {
      ROS_ERROR_STREAM("Joint "<<it.first<<" does not exist in LCM plan");
      return false;
    }
  hasIdx_ = true;
  return hasIdx_;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lcm2ros_dual_arm_husky", ros::init_options::NoSigintHandler);
  std::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if (!lcm->good())
  {
    std::cerr << "ERROR: lcm is not good()" << std::endl;
  }
  ros::NodeHandle nh;

  LCM2ROS handlerObject(lcm, nh);
  ROS_INFO_STREAM("LCM2ROS Dual Arm Husky Translator Ready");

  while (0 == lcm->handle())
  {
  }

  return 0;
}
