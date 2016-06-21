#include "lcm2ros_ihmc.hpp"


ihmc_msgs::FootstepDataRosMessage LCM2ROS::convertFootStepToIHMC(const drc::footstep_t &drc_step) {
  ihmc_msgs::FootstepDataRosMessage ihmc_step;
  ihmc_step.robot_side = drc_step.is_right_foot;
  ihmc_step.location.x = drc_step.pos.translation.x;
  ihmc_step.location.y = drc_step.pos.translation.y;
  ihmc_step.location.z = drc_step.pos.translation.z;
  ihmc_step.orientation.w = drc_step.pos.rotation.w;
  ihmc_step.orientation.x = drc_step.pos.rotation.x;
  ihmc_step.orientation.y = drc_step.pos.rotation.y;
  ihmc_step.orientation.z = drc_step.pos.rotation.z;

  // Used values from footstepsdriver.py for Valkyrie version 2
  // foot_length = 0.21, foot_width = 0.11 (dimension of the sole))
  if (robotName_.compare("valkyrie") == 0)
  {
    double footsizeReduction = 0.04;
    double foot_length = 0.25 - footsizeReduction;
    double foot_width = 0.15 - footsizeReduction;
    // if (support_contact_groups == 0) we do not set the contact points because
    // a value of null will default to use the entire foot
    if (drc_step.params.support_contact_groups == 1)
    {
      ihmc_msgs::Point2dRosMessage point;
      point.x = 0.5 * foot_length;
      point.y = -0.5 * foot_width;
      ihmc_step.predicted_contact_points.push_back(point);
      point.x = 0.5 * foot_length;
      point.y = 0.5 * foot_width;
      ihmc_step.predicted_contact_points.push_back(point);
      point.x = -0.166666667 * foot_length;
      point.y = -0.5 * foot_width;
      ihmc_step.predicted_contact_points.push_back(point);
      point.x = -0.166666667 * foot_length;
      point.y = 0.5 * foot_width;
      ihmc_step.predicted_contact_points.push_back(point);
    }
    else if (drc_step.params.support_contact_groups == 2)
    {
      ihmc_msgs::Point2dRosMessage point;
      point.x = 0.166666667 * foot_length;
      point.y = -0.5 * foot_width;
      ihmc_step.predicted_contact_points.push_back(point);
      point.x = 0.166666667 * foot_length;
      point.y = 0.5 * foot_width;
      ihmc_step.predicted_contact_points.push_back(point);
      point.x = -0.5 * foot_length;
      point.y = -0.5 * foot_width;
      ihmc_step.predicted_contact_points.push_back(point);
      point.x = -0.5 * foot_length;
      point.y = 0.5 * foot_width;
      ihmc_step.predicted_contact_points.push_back(point);
    }
  }
  // else if atlas, always use the entire foot


  double max_relative_terrain_height = 0.0;
  ihmc_step.trajectory_type = ihmc_step.BASIC;

  if (drc_step.terrain_height.size() > 0) {
    assert(drc_step.terrain_height.size() == drc_step.terrain_path_dist.size());
    double starting_terrain_height = drc_step.terrain_height[0];
    max_relative_terrain_height = *std::max_element(drc_step.terrain_height.begin(), drc_step.terrain_height.end()) - starting_terrain_height;
    double swing_distance_in_plane = drc_step.terrain_path_dist[drc_step.terrain_path_dist.size() - 1] - drc_step.terrain_path_dist[0];
    if (swing_distance_in_plane > 0) {
      // Does the terrain rise significantly above a triangle that starts at the initial foot pose, rises to the maximum terrain height halfway through the swing, and finishes at the final foot pose? If so, do a high clearance step. 
      for (size_t i = 0; i < drc_step.terrain_height.size(); i++) {
        double expected_relative_terrain_height = max_relative_terrain_height * (1 - std::abs(drc_step.terrain_path_dist[i] - swing_distance_in_plane / 2.0) / (swing_distance_in_plane / 2.0));
        if (drc_step.terrain_height[i] > expected_relative_terrain_height + starting_terrain_height + 0.02) {
          ihmc_step.trajectory_type = ihmc_step.OBSTACLE_CLEARANCE;
          break;
        }
      }
    }
  }

  double swing_height = max_relative_terrain_height + drc_step.params.step_height;
  ihmc_step.swing_height = std::min(std::max(swing_height, MIN_SWING_HEIGHT),
                                    MAX_SWING_HEIGHT);

  return ihmc_step;
}



void LCM2ROS::footstepPlanHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
                                  const drc::walking_plan_request_t* msg)
{
  ROS_ERROR("LCM2ROS got WALKING_CONTROLLER_PLAN_REQUEST (non-pronto and drake mode)");

  ihmc_msgs::FootstepDataListRosMessage mout;
  mout.transfer_time = msg->footstep_plan.footsteps[0].params.ihmc_transfer_time;
  mout.swing_time = msg->footstep_plan.footsteps[0].params.ihmc_swing_time;
  mout.unique_id = msg->utime; // TODO: needs a better ID, but without unique_id the message gets ignored
  for (int i = 2; i < msg->footstep_plan.num_steps; i++)  // skip the first two standing steps
  {
    mout.footstep_data_list.push_back(convertFootStepToIHMC(msg->footstep_plan.footsteps[i]));
  }
  walking_plan_pub_.publish(mout);
}


void LCM2ROS::footstepPlanBDIModeHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
                                         const drc::footstep_plan_t* msg)
{
  ROS_ERROR("LCM2ROS got BDI_ADJUSTED_FOOTSTEP_PLAN or COMMITTED_FOOTSTEP_PLAN (pronto and bdi mode)");

  ihmc_msgs::FootstepDataListRosMessage mout;
  mout.transfer_time = msg->footsteps[0].params.ihmc_transfer_time;
  mout.swing_time = msg->footsteps[0].params.ihmc_swing_time;
  mout.unique_id = msg->utime; // TODO: needs a better ID, but without unique_id the message gets ignored 
  for (int i = 2; i < msg->num_steps; i++)  // skip the first two standing steps
  {
    mout.footstep_data_list.push_back(convertFootStepToIHMC(msg->footsteps[i]));
  }
  walking_plan_pub_.publish(mout);
}

void LCM2ROS::lFootPoseHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const ihmc::foot_pose_packet_message_t* msg)
{
  ROS_ERROR("LCM2ROS got desired left foot pose");
  if (msg->robot_side != LEFT){ // left foot check
    ROS_ERROR("LCM2ROS foot side not correct, not sending");
    return;
  }
  sendFootPose(msg);
}

void LCM2ROS::rFootPoseHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const ihmc::foot_pose_packet_message_t* msg)
{
  ROS_ERROR("LCM2ROS got desired right foot pose");
  if (msg->robot_side != RIGHT){ // right foot check
    ROS_ERROR("LCM2ROS foot side not correct, not sending");
    return;
  }
  sendFootPose(msg);
}

void LCM2ROS::sendFootPose(const ihmc::foot_pose_packet_message_t* msg)
{
  ihmc_msgs::SE3TrajectoryPointRosMessage point;
  point.time = msg->trajectory_time;

  point.position.x = msg->position[0];
  point.position.y = msg->position[1];
  point.position.z = msg->position[2];
  point.orientation.w = msg->orientation[0];
  point.orientation.x = msg->orientation[1];
  point.orientation.y = msg->orientation[2];
  point.orientation.z = msg->orientation[3];

  point.linear_velocity.x = 0;
  point.linear_velocity.y = 0;
  point.linear_velocity.z = 0;
  point.angular_velocity.x = 0;
  point.angular_velocity.y = 0;
  point.angular_velocity.z = 0;
  point.unique_id = -1;

  std::vector<ihmc_msgs::SE3TrajectoryPointRosMessage> points;
  points.push_back(point);


  ihmc_msgs::FootTrajectoryRosMessage mout;
  mout.robot_side = msg->robot_side;
  mout.taskspace_trajectory_points = points;
  mout.execution_mode = 0;//OVERRIDE;
  mout.previous_message_id = -1;
  mout.unique_id = msg->utime;
  foot_pose_pub_.publish(mout);

}