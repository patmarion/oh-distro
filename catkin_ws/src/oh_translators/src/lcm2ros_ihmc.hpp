#ifndef LCM2ROS_IHMC_HPP_
#define LCM2ROS_IHMC_HPP_


#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <lcm/lcm-cpp.hpp>
#include <ros/ros.h>

#include <model-client/model-client.hpp>
#include "kdl_parser/kdl_parser.hpp"
#include "forward_kinematics/treefksolverposfull_recursive.hpp"


#include "lcmtypes/bot_core/pose_t.hpp"
#include "lcmtypes/bot_core/joint_angles_t.hpp"
#include "lcmtypes/drc/walking_plan_t.hpp"
#include "lcmtypes/drc/walking_plan_request_t.hpp"
#include "lcmtypes/drc/footstep_plan_t.hpp"
#include "lcmtypes/drc/plan_control_t.hpp"
#include "lcmtypes/drc/robot_plan_t.hpp"
#include "lcmtypes/drc/scs_api_command_t.hpp"
#include "lcmtypes/drc/int64_stamped_t.hpp"
#include "lcmtypes/ihmc/com_height_packet_message_t.hpp"
#include "lcmtypes/ihmc/hand_pose_packet_message_t.hpp"
#include "lcmtypes/ihmc/foot_pose_packet_message_t.hpp"


#include <std_msgs/String.h>
#include <ihmc_msgs/FootstepDataListRosMessage.h>
#include <ihmc_msgs/PauseWalkingRosMessage.h>
#include <ihmc_msgs/StopAllTrajectoryRosMessage.h>
#include <ihmc_msgs/WholeBodyTrajectoryRosMessage.h>
//#include <ihmc_msgs/ArmTrajectoryRosMessage.h>
//#include <ihmc_msgs/HeadTrajectoryRosMessage.h>
//#include <ihmc_msgs/FootTrajectoryRosMessage.h>

#define LEFT 0
#define RIGHT 1
#define MIN_SWING_HEIGHT 0.05
#define MAX_SWING_HEIGHT 0.1 

enum class TrajectoryMode {wholeBody, leftArm, rightArm, bothArms}; // 0,1,2,3
static const char * TrajectoryNames[] = {"Whole Body", "Left Arm", "Right Arm", "Both Arms"};


class LCM2ROS
{
public:
  LCM2ROS(boost::shared_ptr<lcm::LCM> &lcm_, ros::NodeHandle &nh_, std::string robotName_);
  ~LCM2ROS()
  {
  }

private:

const char * getTrajectoryName( int enumVal )
{
  return TrajectoryNames[enumVal];
}

  boost::shared_ptr<lcm::LCM> lcm_;
  ros::NodeHandle nh_;
  ros::NodeHandle* node_;
  std::string robotName_;
  boost::shared_ptr<ModelClient> model_;
  boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive> fksolver_;
  std::string chestLinkName_;

  // Parameters and Variables:
  // Seconds to offset the plan so that the controller
  // can blend from the current desired joint position to the plan joint position
  // this was added to avoid controller jerks when starting short plans.
  double planDesiredOffset_;
  TrajectoryMode outputTrajectoryMode_;

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Walking Messages
  void footstepPlanHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
                           const drc::walking_plan_request_t* msg);
  void footstepPlanBDIModeHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
                                  const drc::footstep_plan_t* msg);
  ros::Publisher walking_plan_pub_;
  ihmc_msgs::FootstepDataRosMessage convertFootStepToIHMC(const drc::footstep_t & drc_step);

  // Safety Messages
  void stopHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::plan_control_t* msg);
  void stopManipHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::plan_control_t* msg);
  ros::Publisher pause_pub_, stop_manip_pub_;

  // Whole Body and Arm Plan Messages
  void ihmcControlModeCommandHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::int64_stamped_t* msg);
  void robotPlanHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::robot_plan_t* msg);
  ros::Publisher arm_joint_traj2_pub_, whole_body_trajectory_pub_;

  double getPlanTimeAtWaypoint(int64_t planUtime);
  void sendSingleArmPlan(const drc::robot_plan_t* msg, std::vector<std::string> output_joint_names_arm,
                         std::vector<std::string> input_joint_names, bool is_right);
  bool getSingleArmPlan(const drc::robot_plan_t* msg, std::vector<std::string> output_joint_names_arm,
                        std::vector<std::string> input_joint_names, bool is_right,
                        ihmc_msgs::ArmTrajectoryRosMessage &m);
  bool getChestTrajectoryPlan(const drc::robot_plan_t* msg, std::vector<geometry_msgs::Quaternion> &m);


  void handPoseCommandHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const bot_core::pose_t* msg);
  ros::Publisher hand_pose_command_pub_;


  // Neck Control Messages
  void neckPitchHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const bot_core::joint_angles_t* msg);
  void headOrientationHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const bot_core::pose_t* msg);
  ros::Publisher neck_orientation_pub_;

  // Feet Control Messages
  void lFootPoseHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const ihmc::foot_pose_packet_message_t* msg);
  void rFootPoseHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const ihmc::foot_pose_packet_message_t* msg);
  ros::Publisher foot_pose_pub_;
  void sendFootPose(const ihmc::foot_pose_packet_message_t* msg);

  // Misc Messages
  void comHeightHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
                        const ihmc::com_height_packet_message_t* msg);
  ros::Publisher com_height_pub_;

  void scsAPIHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::scs_api_command_t* msg);
  ros::Publisher scs_api_pub_;

};

#endif /* LCM2ROS_IHMC_HPP_ */
