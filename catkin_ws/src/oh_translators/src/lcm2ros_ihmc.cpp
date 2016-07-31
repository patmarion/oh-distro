// Copyright 2015 Maurice Fallon, Vladimir Ivan

#include "lcm2ros_ihmc.hpp"
#include "lcm2ros_ihmc_conversions.hpp"




LCM2ROS::LCM2ROS(boost::shared_ptr<lcm::LCM> &lcm_in, ros::NodeHandle &nh_in, std::string robotName_in):
    lcm_(lcm_in), nh_(nh_in), robotName_(robotName_in)
{
  model_ = boost::shared_ptr<ModelClient>(new ModelClient(lcm_->getUnderlyingLCM(), 0));
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(model_->getURDFString(), tree))
  {
    std::cerr << "ERROR: Failed to extract kdl tree from xml robot description" << std::endl;
    exit(-1);
  }
  fksolver_ = boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive>(new KDL::TreeFkSolverPosFull_recursive(tree));

  if (robotName_.compare("atlas") == 0)  // Atlas has utorso
    chestLinkName_ = "utorso";
  else  // valkyrie has torso
    chestLinkName_ = "torso";
  std::cout << "Using "<< robotName_ << " so expecting chest link called " << chestLinkName_ << "\n";

  // Hard Coded Parameters:
  // Conservative values for real Valkyrie, using defaults used by IHMC
  planDesiredOffset_ = 1.0;
  // Variable to set what part of a whole body plan gets passed through to Val:
  outputTrajectoryMode_ = TrajectoryMode::wholeBody;
  ROS_ERROR("LCM2ROS Controller TrajectoryMode: %s", getTrajectoryName( (int) outputTrajectoryMode_) );

  ////////////////// Subscriptions and Adverts //////////////////////
  // If pronto is running never send plans like this:
  lcm_->subscribe("WALKING_CONTROLLER_PLAN_REQUEST", &LCM2ROS::footstepPlanHandler, this);
  // COMMITTED_FOOTSTEP_PLAN is creating in Pronto frame and transformed into 
  // BDI/IHMC coordinate frame using BDI_ADJUSTED_FOOTSTEP_PLAN
  lcm_->subscribe("BDI_ADJUSTED_FOOTSTEP_PLAN", &LCM2ROS::footstepPlanBDIModeHandler, this);
  walking_plan_pub_ = nh_.advertise<ihmc_msgs::FootstepDataListRosMessage>(
      "/ihmc_ros/" + robotName_ + "/control/footstep_list", 10);

  // Subscribe correction from localization module
  lcm_->subscribe("POSE_BODY_CORRECTED", &LCM2ROS::robotPoseCorrectionHandler, this);
  correction_pub_ = nh_.advertise<nav_msgs::Odometry>(
      "/ihmc_ros/localization/pelvis_odom_pose_correction", 10);

  lcm_->subscribe("STOP_WALKING", &LCM2ROS::stopHandler, this);  // from Director
  pause_pub_ = nh_.advertise<ihmc_msgs::PauseWalkingRosMessage>("/ihmc_ros/" + robotName_ + "/control/pause_walking",
                                                             10);
  lcm_->subscribe("COMMITTED_PLAN_PAUSE", &LCM2ROS::stopManipHandler, this);  // from Director to stop manipulation plans
  stop_manip_pub_ = nh_.advertise<ihmc_msgs::StopAllTrajectoryRosMessage>(
      "/ihmc_ros/" + robotName_ + "/control/stop_all_trajectories", 10);

  // robot plan messages now used, untested
  lcm_->subscribe("COMMITTED_ROBOT_PLAN", &LCM2ROS::robotPlanHandler, this);
  arm_joint_traj2_pub_ = nh_.advertise<ihmc_msgs::ArmTrajectoryRosMessage>(
      "/ihmc_ros/" + robotName_ + "/control/arm_trajectory", 10);
  whole_body_trajectory_pub_ = nh_.advertise<ihmc_msgs::WholeBodyTrajectoryRosMessage>(
      "/ihmc_ros/" + robotName_ + "/control/whole_body_trajectory", 10);
  neck_joint_traj_pub_ = nh_.advertise<ihmc_msgs::NeckTrajectoryRosMessage>(
              "/ihmc_ros/" + robotName_ + "/control/neck_trajectory", 10);


  // These messages work with new IHMC API, but are not recommended:
  lcm_->subscribe("HAND_POSE_COMMAND_LEFT", &LCM2ROS::handPoseCommandHandler, this);
  lcm_->subscribe("HAND_POSE_COMMAND_RIGHT", &LCM2ROS::handPoseCommandHandler, this);
  hand_pose_command_pub_ = nh_.advertise<ihmc_msgs::HandTrajectoryRosMessage>(
      "/ihmc_ros/" + robotName_ + "/control/hand_trajectory", 10);



  lcm_->subscribe("DESIRED_NECK_ANGLES", &LCM2ROS::neckPitchHandler, this);
//  lcm_->subscribe("DESIRED_HEAD_ORIENTATION", &LCM2ROS::headOrientationHandler, this);
//  neck_orientation_pub_ = nh_.advertise<ihmc_msgs::HeadTrajectoryRosMessage>(
//      "/ihmc_ros/" + robotName_ + "/control/head_orientation", 10);

  lcm_->subscribe("DESIRED_LEFT_FOOT_POSE", &LCM2ROS::lFootPoseHandler, this);
  lcm_->subscribe("DESIRED_RIGHT_FOOT_POSE", &LCM2ROS::rFootPoseHandler, this);
  // Only one topic for both feet
  foot_pose_pub_ = nh_.advertise<ihmc_msgs::FootTrajectoryRosMessage>(
      "/ihmc_ros/" + robotName_ + "/control/foot_trajectory", 10);


  lcm_->subscribe("SCS_API_CONTROL", &LCM2ROS::scsAPIHandler, this);
  scs_api_pub_ = nh_.advertise<std_msgs::String>("/ihmc_ros/" + robotName_ + "/api_command", 10);


  // Subscriptions that handle local variables:
  lcm_->subscribe("IHMC_CONTROL_MODE_COMMAND", &LCM2ROS::ihmcControlModeCommandHandler, this);

  node_ = new ros::NodeHandle();
}


int main(int argc, char** argv)
{
  std::string robotName;  // = "valkyrie";  // "atlas"

  if (argc >= 2)
  {
    robotName = argv[1];

    if (!((robotName == "atlas") || robotName == "valkyrie")) {
      ROS_ERROR("Robot name needs to be either atlas or valkyrie");
      return 1;
    }

    ROS_ERROR("Robot Name: %s", argv[1]);
  }
  else
  {
    ROS_ERROR("Need to have an argument: robot name");
    exit(-1);
  }

  ros::init(argc, argv, "lcm2ros_ihmc", ros::init_options::NoSigintHandler);
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if (!lcm->good())
  {
    std::cerr << "ERROR: lcm is not good()" << std::endl;
  }
  ros::NodeHandle nh;

  LCM2ROS handlerObject(lcm, nh, robotName);
  ROS_ERROR("LCM2ROS IHMC Translator Ready [robotName: %s]", robotName.c_str());

  while (0 == lcm->handle())
  {
  }

  return 0;
}
