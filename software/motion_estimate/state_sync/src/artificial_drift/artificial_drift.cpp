// Script which generates artificial drift and subscribes to a correction.
// To be used in simulation.

// Input: POSE_BODY_ALT Output: POSE_BODY_ALT_WITH_DRIFT
// Applies incremental drift while the robot is walking

// Problem: to our system will appear to have drifted
// but any goals we give to IHMC will be interpreted from their (true)
// coordinate frame and acted upon. 
// Really this drift should be applied to the signals they are seeing.
// so that we and they see the same drifting error

#include <stdio.h>
#include <inttypes.h>
#include <iostream>
#include <limits>
#include <vector>
#include <fstream>

#include "artificial_drift.hpp"
#include <pronto_utils/pronto_math.hpp>
#include <pronto_utils/pronto_conversions_lcm.hpp>

#include <ConciseArgs>
#include <sys/time.h>
#include <algorithm>

/////////////////////////////////////
App::App(std::shared_ptr<lcm::LCM> &lcm_,
                       std::shared_ptr<CommandLineConfig> &cl_cfg_):
                       lcm_(lcm_), cl_cfg_(cl_cfg_), driftingPose_(0,Eigen::Isometry3d::Identity()),
                       previousCorrectPose_(0,Eigen::Isometry3d::Identity()) {

  // Subscribe to required signals
  lcm_->subscribe("POSE_BODY_ALT",&App::poseIHMCHandler,this); // Always provided by the IHMC Driver
  lcm_->subscribe("ROBOT_BEHAVIOR", &App::behaviorHandler, this);

  last_behavior_ = -1; // uninitialised

  restart_corrupting_ = true;
}


void App::behaviorHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::behavior_t* msg){

  last_behavior_ = msg->behavior;
}


void App::poseIHMCHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg){
  if (restart_corrupting_){
    //std::cout << "initializing drifting pose\n";
    driftingPose_ = pronto::getPoseAsIsometry3dTime(msg);

    // TODO: publish inital pose to system unmodified
    previousCorrectPose_ = driftingPose_;
    restart_corrupting_ = false;
    return;
  }

  Isometry3dTime currentCorrectPose = pronto::getPoseAsIsometry3dTime(msg);

  // Calculate the most recentl differential motion and apply as transform:
  Eigen::Isometry3d deltaCorrectPose = previousCorrectPose_.pose.inverse() * currentCorrectPose.pose ;  
  double dt = (currentCorrectPose.utime - previousCorrectPose_.utime)*1E-6; // typically 0.002
  driftingPose_.pose = driftingPose_.pose * deltaCorrectPose;
  driftingPose_.utime = msg->utime;

  // If walking: rotate the pose in yaw (world aligned) and add translations
  if (last_behavior_ == 4){

    Eigen::Quaterniond quat_original(driftingPose_.pose.rotation());
    double rpy_original[3];
    quat_to_euler(quat_original, rpy_original[0], rpy_original[1], rpy_original[2]);
    rpy_original[2] += cl_cfg_->yaw_drift_rate*dt;
    Eigen::Quaterniond quat_yaw_drift =  euler_to_quat(rpy_original[0], rpy_original[1], rpy_original[2]);  

    Eigen::Isometry3d driftingPoseNew;
    driftingPoseNew.setIdentity();
    driftingPoseNew.translation()  << driftingPose_.pose.translation() 
        + Eigen::Vector3d(cl_cfg_->x_drift_rate*dt, cl_cfg_->y_drift_rate*dt, cl_cfg_->z_drift_rate*dt);
    driftingPoseNew.rotate(quat_yaw_drift);
    driftingPose_.pose = driftingPoseNew;
  }else{
    std::cout << msg->utime <<": standing, not corrupting.\n";
    restart_corrupting_ = true;
  }

  bot_core::pose_t msg_out = pronto::getIsometry3dAsBotPose(driftingPose_.pose, driftingPose_.utime);
  lcm_->publish("POSE_BODY_ALT_WITH_DRIFT",&msg_out);

  previousCorrectPose_ = currentCorrectPose;
}

int main(int argc, char ** argv)
{
  std::shared_ptr<CommandLineConfig> cl_cfg(new CommandLineConfig());
  ConciseArgs opt(argc, (char**)argv);
  opt.add(cl_cfg->output_channel, "o", "output_channel","Output Channel for robot state msg");
  opt.parse();

  std::shared_ptr<lcm::LCM> lcm(new lcm::LCM());
  if (!lcm->good()) return 1;

  std::cout << "ready\n";
  App app(lcm, cl_cfg);


  while (0 == lcm->handle());
  return 0;
}