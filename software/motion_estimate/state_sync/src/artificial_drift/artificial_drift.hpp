#ifndef artificialDrift_HPP_
#define artificialDrift_HPP_

#include <lcm/lcm-cpp.hpp>

#include <map>
#include <memory>

#include "lcmtypes/bot_core.hpp"
#include "lcmtypes/drc/behavior_t.hpp"
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <pronto_utils/pronto_math.hpp>


/////////////////////////////////////
class CommandLineConfig{
  public:
    CommandLineConfig(){
      // Read from command line:
      output_channel = "EST_ROBOT_STATE";

      // drift rates (in si) per second (reasonable rates)
      x_drift_rate = 0.0005;
      y_drift_rate = 0.00025;
      z_drift_rate = 0.0005;
      yaw_drift_rate = 0.005;

      // high rate drift - for demo
      /*
      x_drift_rate = 0.005;
      y_drift_rate = 0.0025;
      z_drift_rate = 0.005;
      yaw_drift_rate = 0.005;
      */


    }
    ~CommandLineConfig(){};

    std::string output_channel;

  // drift rates (in si) per second
  double x_drift_rate;
  double y_drift_rate;
  double z_drift_rate;
  double yaw_drift_rate;


};

///////////////////////////////////////////////////////////////
class App{
  public:
    App(std::shared_ptr<lcm::LCM> &lcm_, std::shared_ptr<CommandLineConfig> &cl_cfg_);
    
    ~App(){
    }
    void Identity();
    
   
  private:
    std::shared_ptr<CommandLineConfig> cl_cfg_;
    std::shared_ptr<lcm::LCM> lcm_;
        
    void poseIHMCHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg);
    void poseCorrHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg);
    void behaviorHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::behavior_t* msg);
    
    //Eigen::Isometry3d
    Isometry3dTime driftingPose_;
    Isometry3dTime previousCorrection_;
    Isometry3dTime currentCorrection_;
    Eigen::Isometry3d correction_;
    Isometry3dTime previousCorrectPose_;    

    int last_behavior_;
    bool updatedCorrection_;
};    

#endif
