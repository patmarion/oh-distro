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

void LCM2ROS::stopHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::plan_control_t* msg)
{
  ROS_ERROR("LCM2ROS got STOP_WALKING - sending pause=true");
  ihmc_msgs::PauseWalkingRosMessage mout;
  mout.pause = true;
  mout.unique_id = msg->utime;
  pause_pub_.publish(mout);
}

void LCM2ROS::stopManipHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
                               const drc::plan_control_t* msg)
{
  ROS_ERROR("LCM2ROS got COMMITTED_PLAN_PAUSE - sending manipulate stop");
  ihmc_msgs::StopAllTrajectoryRosMessage mout;
  mout.unique_id = msg->utime;
  stop_manip_pub_.publish(mout);
}

std::vector<int> LCM2ROS::findJointIndices(std::vector<std::string>& input_joint_names, std::vector<std::string>& output_joint_names)
{
    std::vector<int> indices;
    for (size_t i = 0; i < output_joint_names.size(); i++)
    {
      std::vector<std::string>::iterator it;
      int index = find(input_joint_names.begin(), input_joint_names.end(), output_joint_names[i]) - input_joint_names.begin();
      if (index < input_joint_names.size())
      {
        indices.push_back(index);
      }
      else
      {
        ROS_ERROR("%s not found in input_joint_names, not sending plan", output_joint_names[i].c_str());
        std::cout << output_joint_names[i] << " not found in input_joint_names, not sending plan\n";
      }
    }
    return indices;
}


std::vector<ihmc_msgs::OneDoFJointTrajectoryRosMessage> LCM2ROS::getJointTrajectories(const bot_core::joint_angles_t* msg, std::vector<int>& indices)
{
    std::vector<ihmc_msgs::OneDoFJointTrajectoryRosMessage> ret;
    for (int j = 0; j < indices.size(); j++)
    {
        ihmc_msgs::OneDoFJointTrajectoryRosMessage joint_trajectory_message;
        ihmc_msgs::TrajectoryPoint1DRosMessage point;
        point.position = msg->joint_position[indices[j]];
        point.velocity = 0.0;

        point.time = 2.0;
        point.unique_id = 1;
        joint_trajectory_message.unique_id = 1;

        joint_trajectory_message.trajectory_points.push_back(point);
        ret.push_back( joint_trajectory_message );
    }
    return ret;
}

bool LCM2ROS::getNeckPlan(const bot_core::joint_angles_t* msg, std::vector<std::string> output_joint_names_neck,
                               std::vector<std::string> input_joint_names, ihmc_msgs::NeckTrajectoryRosMessage &m)
{
    // Find the indices of the arm joints which we want to extract
    std::vector<int> neck_indices = findJointIndices(input_joint_names, output_joint_names_neck);
    if(neck_indices.size()!=output_joint_names_neck.size()) return false;

    m.joint_trajectory_messages = getJointTrajectories(msg, neck_indices);
    m.unique_id = msg->utime;
    return true;
}

void LCM2ROS::sendNeckPlan(const bot_core::joint_angles_t* msg, std::vector<std::string> output_joint_names_neck,
                                std::vector<std::string> input_joint_names)
{
  ihmc_msgs::NeckTrajectoryRosMessage m;
  bool status = getNeckPlan(msg, output_joint_names_neck, input_joint_names, m);
  if (status)
  {
    neck_joint_traj_pub_.publish(m);
  }
  else
  {
    ROS_ERROR("LCM2ROS: problem with neck plan, not sending");
  }
}

void LCM2ROS::neckPitchHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const bot_core::joint_angles_t* msg)
{
  ROS_ERROR("LCM2ROS got desired neck pitch");

  std::vector<std::string> neck_strings = {"lowerNeckPitch", "neckYaw", "upperNeckPitch"};

  sendNeckPlan(msg, neck_strings, msg->joint_name);
}

/*
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
