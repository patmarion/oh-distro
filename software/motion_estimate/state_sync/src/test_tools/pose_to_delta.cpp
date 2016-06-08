#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>

#include <path_util/path_util.h>
#include <lcmtypes/bot_core.hpp>

#include <bot_param/param_client.h>
#include <bot_param/param_util.h>
#include <lcmtypes/bot_core/pose_t.hpp>

#include <bot_frames_cpp/bot_frames_cpp.hpp>
#include <ConciseArgs>

////////////////////////////////////////
struct CommandLineConfig
{
    int mode;
};

class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& ca_cfg_);
    
    ~App(){
    }    
    
    void sendUpdate();
    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    const CommandLineConfig& ca_cfg_;
    
    void pbdih(const lcm::ReceiveBuffer* rbuf, 
                      const std::string& channel, const  bot_core::pose_t* msg);   
    
    BotParam* botparam_;
    BotFrames* botframes_;
    bot::frames* botframes_cpp_;   
    
//    Isometry3dTime previousPose_;

    int counter_; // used for terminal feedback
    int verbose_;
};

/*


bot_core::pose_t getIsometry3dAsBotPose(Eigen::Isometry3d pose, int64_t utime){
  bot_core::pose_t tf;
  tf.utime = utime;
  tf.pos[0] = pose.translation().x();
  tf.pos[1] = pose.translation().y();
  tf.pos[2] = pose.translation().z();
  Eigen::Quaterniond quat(pose.rotation());
  tf.orientation[0] = quat.w();
  tf.orientation[1] = quat.x();
  tf.orientation[2] = quat.y();
  tf.orientation[3] = quat.z();
  return tf;
}



// Difference the transform and scale by elapsed time:
// [duplicated in rbis_fovis_common.cpp]
BotTrans LegOdoCommon::getTransAsVelocityTrans(BotTrans msgT, int64_t utime, int64_t prev_utime){
  BotTrans msgT_vel;
  memset(&msgT_vel, 0, sizeof(msgT_vel));
  
  double rpy[3];
  bot_quat_to_roll_pitch_yaw(msgT.rot_quat,rpy);
  double elapsed_time = ( (double) utime -  prev_utime)/1000000;
  double rpy_rate[3];
  rpy_rate[0] = rpy[0]/elapsed_time;
  rpy_rate[1] = rpy[1]/elapsed_time;
  rpy_rate[2] = rpy[2]/elapsed_time;
  
  if (verbose){
    std::stringstream ss;
    ss << utime << " msgT: ";
    printTrans(msgT, ss.str() );  
    std::cout << "Elapsed Time: " << elapsed_time  << " sec\n";
    std::cout << "RPY: " << rpy[0] << ", "<<rpy[1] << ", "<<rpy[2] <<" rad\n";
    std::cout << "RPY: " << rpy[0]*180/M_PI << ", "<<rpy[1]*180/M_PI << ", "<<rpy[2]*180/M_PI <<" deg\n";
    std::cout << "RPY: " << rpy_rate[0] << ", "<<rpy_rate[1] << ", "<<rpy_rate[2] <<" rad/s | velocity scaled\n";
    std::cout << "RPY: " << rpy_rate[0]*180/M_PI << ", "<<rpy_rate[1]*180/M_PI << ", "<<rpy_rate[2]*180/M_PI <<" deg/s | velocity scaled\n";
    std::cout << "XYZ: " << msgT.trans_vec[0] << ", "  << msgT.trans_vec[1] << ", "  << msgT.trans_vec[2] << "\n";
  }
  
  msgT_vel.trans_vec[0] = msgT.trans_vec[0]/elapsed_time;
  msgT_vel.trans_vec[1] = msgT.trans_vec[1]/elapsed_time;
  msgT_vel.trans_vec[2] = msgT.trans_vec[2]/elapsed_time;
  bot_roll_pitch_yaw_to_quat (rpy_rate, msgT_vel.rot_quat);
  
  if (verbose){
    std::stringstream ss2;
    ss2 << " msgT_vel: ";
    printTrans(msgT_vel, ss2.str() );
    std::cout << "\n\n";
  }  
  
  return msgT_vel;
}

void LegOdoHandler::sendTransAsVelocityPose(BotTrans msgT, int64_t utime, int64_t prev_utime, std::string channel){
  BotTrans msgT_vel = leg_odo_common_->getTransAsVelocityTrans(msgT, utime, prev_utime);
  bot_core::pose_t vel_pose = getBotTransAsBotPoseVelocity(msgT_vel, utime)  ;
  lcm_pub->publish(channel, &vel_pose );
}

*/
App::App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& ca_cfg_):
    lcm_(lcm_), ca_cfg_(ca_cfg_){


  
  counter_ =0;  
  verbose_=3; // 1 important, 2 useful 3, lots
  
  std::cout << "POSE_BODY body_linear_rate_to_local_linear_rate\n";
  lcm_->subscribe("POSE_BODY",&App::pbdih,this);  
}

void App::pbdih(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg){
  std::cout << channel <<  "\n" << msg->utime << "\n";
  //lcm_->publish("POSE_VICON", &m);  
/*
  if (previousCorrectPose_.utime == 0){
    std::cout << "initializing pose\n";
    previousPose_ = getPoseAsIsometry3dTime(msg);
    return;
  }

  Isometry3dTime currentPose = getPoseAsIsometry3dTime(msg);

  Eigen::Isometry3d deltaCorrectPose = previousPose_.pose.inverse() * currentPose.pose ;  
  double dt = (currentPose.utime - previousPose_.utime)*1E-6; // typically 0.002


  bot_core::pose_t msg_out = getIsometry3dAsBotPose(deltaCorrectPose.pose, deltaCorrectPose.utime);
  lcm_->publish("POSE_BODY_ALT_WITH_DRIFT",&msg_out);
*/
}


int main(int argc, char ** argv) {
  CommandLineConfig ca_cfg;
  ca_cfg.mode = 0;
 
  ConciseArgs opt(argc, (char**)argv);
  opt.add(ca_cfg.mode , "m", "type","Mode: 0 spit params. 1 push");
  opt.parse();  
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  App accu(lcm, ca_cfg);
  
  while(0 == lcm->handle());  

  return 0;
}
