#ifndef CLOUD_ACCUMULATE_HPP_
#define CLOUD_ACCUMULATE_HPP_

#include <deque>
#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core/planar_lidar_t.hpp>
#include <lcmtypes/bot_core/pointcloud2_t.hpp>

#include <laser_utils/laser_util.h>
#include <path_util/path_util.h>

//#include <bot_frames_cpp/bot_frames_cpp.hpp>

#include <pronto_utils/pronto_vis.hpp>
#include <pronto_utils/pronto_lcm.hpp> // decode perception lcm messages
#include <pronto_utils/pronto_frame_check_tools.hpp>
////////////////////////////////////////
struct CloudAccumulateConfig
{
    int batch_size;
    std::string lidar_channel;
    double max_range;
    double min_range;
};

class CloudAccumulate{
  public:
    CloudAccumulate(boost::shared_ptr<lcm::LCM> &lcm_, const CloudAccumulateConfig& ca_cfg_);
    CloudAccumulate(boost::shared_ptr<lcm::LCM> &lcm_, const CloudAccumulateConfig& ca_cfg_,
                    BotParam* botparam, BotFrames* botframes);

    ~CloudAccumulate(){
      delete laser_projector_;
      delete projected_laser_scan_;
    }    
    
    int getCounter(){ return counter_; }
    bool getFinished(){ return finished_; }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud(){ return combined_cloud_; }
    Laser_projector* getLaserProjector(){ return laser_projector_; }
    
    void clearCloud(){ 
      combined_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB> ());
      counter_ = 0;
      finished_ = false;
      std::cout << "Empty previous map\n";
    }
    
    void publishCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);
    void processLidar(const  bot_core::planar_lidar_t* msg);
    void processLidar(std::shared_ptr<bot_core::planar_lidar_t> this_msg);

  private:
    void init(boost::shared_ptr<lcm::LCM> &lcm_, const CloudAccumulateConfig& ca_cfg_,
                          BotParam* botparam, BotFrames* botframes);

    boost::shared_ptr<lcm::LCM> lcm_;
    const CloudAccumulateConfig& ca_cfg_;    
    
    pronto_vis* pc_vis_ ;
    BotParam* botparam_;
    BotFrames* botframes_;
    FrameCheckTools frame_check_tools_;
    
    int counter_; 
    int verbose_;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr combined_cloud_;
    
    bool finished_;
    
    Laser_projector * laser_projector_;
    laser_projected_scan * projected_laser_scan_;  
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertPlanarScanToCloud(std::shared_ptr<bot_core::planar_lidar_t> this_msg);
    
};


#endif
