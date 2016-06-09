#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>

#include <lcmtypes/bot_core.hpp>

#include <bot_param/param_client.h>
#include <bot_param/param_util.h>
#include <lcmtypes/bot_core/pose_t.hpp>

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
    
    BotParam* botparam_;
    BotFrames* botframes_;
    bot::frames* botframes_cpp_;   
    
    Isometry3dTime previousPoseT_;
};


App::App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& ca_cfg_):
    lcm_(lcm_), ca_cfg_(ca_cfg_), previousPoseT_(0,Eigen::Isometry3d::Identity()){

  std::cout << "POSE_BODY body_linear_rate_to_local_linear_rate\n";
  lcm_->subscribe( ca_cfg_.channel,&App::pbdih,this);  
}

void App::poseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg){
  std::cout << channel <<  "\n" << msg->utime << "\n";

  if (previousPoseT_.utime == 0){
    std::cout << "initializing pose\n";
    previousPoseT_ = pronto::getPoseAsIsometry3dTime(msg);
    return;
  }
  Isometry3dTime currentPoseT = pronto::getPoseAsIsometry3dTime(msg);

  // Get the delta distance travelled:
  Eigen::Isometry3d deltaPose = previousPoseT_.pose.inverse() * currentPoseT.pose ;  
  double dt = (currentPoseT.utime - previousPoseT_.utime)*1E-6;
  Isometry3dTime deltaPoseT = Isometry3dTime(dt, deltaPose);
  bot_core::pose_t msg_out = pronto::getIsometry3dAsBotPose(deltaPoseT.pose, deltaPoseT.utime);
  lcm_->publish(std::string(ca_cfg_.channel + "_DELTA"),&msg_out);

  // Convert delta distance into a rate
  Eigen::Isometry3d ratePose = pronto::getTransAsVelocityTrans(deltaPoseT.pose, deltaPoseT.utime);
  bot_core::pose_t msg_out_rate = pronto::getIsometry3dAsBotPose(ratePose, deltaPoseT.utime);
  lcm_->publish(std::string(ca_cfg_.channel + "_VELOCITY"),&msg_out_rate);

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
