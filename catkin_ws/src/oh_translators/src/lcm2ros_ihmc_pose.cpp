#include "lcm2ros_ihmc.hpp"

void LCM2ROS::robotPoseCorrectionHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
                                         const bot_core::pose_t* msg)
{
  nav_msgs::Odometry mout;

  float secs = msg->utime/1000000.0;
  mout.header.stamp = ros::Time(secs);
  //mout.header.frame_id = "world";
  //mout.child_frame_id = "pelvis";
  mout.pose.pose.position.x = msg->pos[0];
  mout.pose.pose.position.y = msg->pos[1];
  mout.pose.pose.position.z = msg->pos[2];
  mout.pose.pose.orientation.w = msg->orientation[0];
  mout.pose.pose.orientation.x = msg->orientation[1];
  mout.pose.pose.orientation.y = msg->orientation[2];
  mout.pose.pose.orientation.z = msg->orientation[3];
  correction_pub_.publish(mout);
}