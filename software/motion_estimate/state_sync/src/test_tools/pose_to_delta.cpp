#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>

#include <bot_param/param_client.h>
#include <bot_param/param_util.h>
#include <lcmtypes/bot_core/pose_t.hpp>
#include <lcmtypes/bot_core/images_t.hpp>

#include <bot_frames_cpp/bot_frames_cpp.hpp>
#include <pronto_utils/pronto_math.hpp>
#include <pronto_utils/pronto_conversions_lcm.hpp>
#include <ConciseArgs>

////////////////////////////////////////
struct CommandLineConfig
{
    int mode;
    std::string channel;
};

class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& ca_cfg_);
    
    ~App(){
    }    
    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    const CommandLineConfig& ca_cfg_;
    
    void poseHandler(const lcm::ReceiveBuffer* rbuf, 
                      const std::string& channel, const  bot_core::pose_t* msg);   
    
    void cameraHandler(const lcm::ReceiveBuffer* rbuf, 
                      const std::string& channel, const  bot_core::images_t* msg);   

    void publishOutput(Isometry3dTime currentPoseT, std::string channel_root);

    BotParam* botparam_;
    BotFrames* botframes_;
    bot::frames* botframes_cpp_;   
    
    Isometry3dTime previousPoseT_;
};


App::App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& ca_cfg_):
    lcm_(lcm_), ca_cfg_(ca_cfg_), previousPoseT_(0,Eigen::Isometry3d::Identity()){

  botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
  botframes_= bot_frames_get_global(lcm_->getUnderlyingLCM(), botparam_);
  botframes_cpp_ = new bot::frames(botframes_);

//  std::cout << "POSE_BODY body_linear_rate_to_local_linear_rate\n";
//  lcm_->subscribe( ca_cfg_.channel,&App::poseHandler,this);  

  std::cout << "MULTISENSE_CAMERA\n";
  lcm_->subscribe( "MULTISENSE_CAMERA",&App::cameraHandler,this);  
}


void App::publishOutput(Isometry3dTime currentPoseT, std::string channel_root)
{
  // Get the delta distance travelled:
  Eigen::Isometry3d deltaPose = previousPoseT_.pose.inverse() * currentPoseT.pose ;  
  Isometry3dTime deltaPoseT = Isometry3dTime(currentPoseT.utime, deltaPose);
  bot_core::pose_t msg_out = pronto::getIsometry3dAsBotPose(deltaPoseT.pose, deltaPoseT.utime);
  lcm_->publish(std::string(channel_root + "_DELTA"),&msg_out);

  // Convert delta distance into a rate
  int64_t dt = (currentPoseT.utime - previousPoseT_.utime);
  Eigen::Isometry3d ratePose = pronto::getTransAsVelocityTrans(deltaPoseT.pose, dt);
  bot_core::pose_t msg_out_rate = pronto::getIsometry3dAsBotPoseVelocity(ratePose, deltaPoseT.utime);
  lcm_->publish(std::string(channel_root + "_VELOCITY"),&msg_out_rate);
}

void App::poseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg){
  // std::cout << channel <<  " " << msg->utime << "\n";

  if (previousPoseT_.utime == 0){
    std::cout << "initializing pose\n";
    previousPoseT_ = pronto::getPoseAsIsometry3dTime(msg);
    return;
  }
  Isometry3dTime currentPoseT = pronto::getPoseAsIsometry3dTime(msg);

  publishOutput(currentPoseT, channel);

  previousPoseT_ = currentPoseT;
}


void App::cameraHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::images_t* msg){
  std::cout << channel <<  " " << msg->utime << "\n";

  if (previousPoseT_.utime == 0){
    std::cout << "initializing pose\n";
    previousPoseT_.utime = msg->utime;
    int status = botframes_cpp_->get_trans_with_utime( botframes_ ,  "CAMERA_LEFT", "local"  , msg->utime, previousPoseT_.pose);
    return;
  }
  Isometry3dTime currentPoseT = Isometry3dTime(msg->utime,Eigen::Isometry3d::Identity());
  int status = botframes_cpp_->get_trans_with_utime( botframes_ ,  "CAMERA_LEFT", "local"  , msg->utime, currentPoseT.pose);

  /*
  bot_core::pose_t msg_out = pronto::getIsometry3dAsBotPose(currentPoseT, msg->utime);
  lcm_->publish(std::string("POSE_CAMERA_LEFT_ALT"),&msg_out);*/

  publishOutput(currentPoseT, "POSE_CAMERA_LEFT");

  previousPoseT_ = currentPoseT;
}


int main(int argc, char ** argv) {
  CommandLineConfig ca_cfg;
  ca_cfg.mode = 0;
  ca_cfg.channel = "POSE_BODY";
 
  ConciseArgs opt(argc, (char**)argv);
  opt.add(ca_cfg.mode , "m", "type","Mode: 0 spit params. 1 push");
  opt.add(ca_cfg.channel , "c", "channel","LCM Channel");
  opt.parse();  
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  App accu(lcm, ca_cfg);
  
  while(0 == lcm->handle());  

  return 0;
}
