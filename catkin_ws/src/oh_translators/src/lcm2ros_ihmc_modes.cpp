// This cpp is a kind of 'left overs' file for boiler plate passthrough handers

#include "lcm2ros_ihmc.hpp"
#include "lcm2ros_ihmc_conversions.hpp"



void LCM2ROS::scsAPIHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
                            const drc::scs_api_command_t* msg)
{
  std_msgs::String rmsg;
  rmsg.data = msg->command;
  scs_api_pub_.publish(rmsg);
}

/*
void LCM2ROS::comHeightHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
                               const ihmc::com_height_packet_message_t* msg)
{
  ROS_ERROR("LCM2ROS got com height");
  ihmc_msgs::ComHeightPacketMessage mout;
  mout.height_offset = msg->height_offset;
  com_height_pub_.publish(mout);
}


void LCM2ROS::pauseHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
                           const ihmc::pause_command_message_t* msg)
{
  ROS_ERROR("LCM2ROS got pause %d", static_cast<int>(msg->pause));
  ihmc_msgs::PauseCommandMessage mout;
  mout.pause = msg->pause;
  pause_pub_.publish(mout);
}

void LCM2ROS::stopHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::plan_control_t* msg)
{
  ROS_ERROR("LCM2ROS got STOP_WALKING - sending pause=true");
  ihmc_msgs::PauseCommandMessage mout;
  mout.pause = true;
  pause_pub_.publish(mout);
}

void LCM2ROS::stopManipHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
                               const drc::plan_control_t* msg)
{
  ROS_ERROR("LCM2ROS got COMMITTED_PLAN_PAUSE - sending manipulate stop");
  ihmc_msgs::StopMotionPacketMessage mout;
  mout.unique_id = msg->utime;
  stop_manip_pub_.publish(mout);
}

void LCM2ROS::neckPitchHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const bot_core::joint_angles_t* msg)
{
  ROS_ERROR("LCM2ROS got desired neck pitch");

  int lower_neck_pitch_id = -1;
  for (int i = 0; i < msg->num_joints; i++) {
    if (msg->joint_name[i] == "lowerNeckPitch") {
      lower_neck_pitch_id = i;
      break;
    }
  }

  if (lower_neck_pitch_id == -1) {
    ROS_WARN("lowerNeckPitch not in DESIRED_NECK_ANGLES message, ignoring");
    return;
  }

  ihmc_msgs::HeadOrientationPacketMessage mout;
  Eigen::Quaterniond quat = euler_to_quat(0, msg->joint_position[lower_neck_pitch_id], 0);
  mout.trajectory_time = 1;
  mout.orientation.w = quat.w();
  mout.orientation.x = quat.x();
  mout.orientation.y = quat.y();
  mout.orientation.z = quat.z();
  mout.unique_id = msg->utime;
  neck_orientation_pub_.publish(mout);
}

void LCM2ROS::headOrientationHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const bot_core::pose_t* msg)
{
  ROS_ERROR("LCM2ROS got desired desired head orientation");
  ihmc_msgs::HeadOrientationPacketMessage mout;
  mout.trajectory_time = 1;
  mout.orientation.w = msg->orientation[0];
  mout.orientation.x = msg->orientation[1];
  mout.orientation.y = msg->orientation[2];
  mout.orientation.z = msg->orientation[3];
  mout.unique_id = msg->utime;
  neck_orientation_pub_.publish(mout);
}
*/
