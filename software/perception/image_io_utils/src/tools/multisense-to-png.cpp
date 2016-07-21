// Tool to take a multisense stereo image (images_t)
// - write the rgb images to png files

#include <stdio.h>
#include <memory>
#include <lcm/lcm.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <ConciseArgs>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <lcmtypes/bot_core.hpp>

#include <image_io_utils/image_io_utils.hpp>   // to simplify jpeg/zlib compression and decompression

using namespace cv;
using namespace std;

class image_tool{
  public:
    image_tool(boost::shared_ptr<lcm::LCM> &lcm_, std::string camera_in_,
      int period_between_files_);
    
    ~image_tool(){}
    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    std::string camera_in_;
    
    void disparityHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::images_t* msg);   
   
    int64_t last_utime_out_;

    image_io_utils*  imgutils_;
    bot_core::image_t rgb_img_;

    int img_buf_size_;
    uint8_t* img_buf_;

    int width_;
    int height_;
    int period_between_files_;
};    

image_tool::image_tool(boost::shared_ptr<lcm::LCM> &lcm_, std::string camera_in_,
      int period_between_files_):
      lcm_(lcm_), camera_in_(camera_in_), period_between_files_(period_between_files_) {
  lcm_->subscribe( camera_in_.c_str(),&image_tool::disparityHandler,this);
  width_ = 1024;
  height_ = 1024;
  imgutils_ = new image_io_utils( lcm_, width_, height_);
  last_utime_out_=0;  
  img_buf_= (uint8_t*) malloc(10* width_  * height_);
}

void image_tool::disparityHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const bot_core::images_t* msg){

  // Determine if we should write this image:
  int msg_time_seconds =  (int) floor(msg->utime *1E-6);
  if (msg_time_seconds < last_utime_out_ + period_between_files_){
    //std::cout << msg_time_seconds << " return, don't write\n";
    return;
  }
  last_utime_out_ = msg_time_seconds;

  // Check the dimensions match
  int w = msg->images[0].width;
  int h = msg->images[0].height; 
  if (w!= width_){
    std::cout << w << " does not match " << width_ << "\n";
    return;
  }
  if (h!= height_){
    std::cout << h << " does not match " << height_ << "\n";    
    return;
  }

  // Decompress images and convert to open CV
  rgb_img_= msg->images[0];
  Mat src= Mat::zeros( h,w  ,CV_8UC3);
  if (rgb_img_.pixelformat == bot_core::image_t::PIXEL_FORMAT_MJPEG){
    imgutils_->decodeImageToRGB( &(rgb_img_), img_buf_);
    src.data = img_buf_;
  }else{
    src.data = rgb_img_.data.data();
  }

  // Flip BGR to RGB
  Mat src_rgb_cv;
  cvtColor( src, src_rgb_cv, CV_BGR2RGB );

  std::stringstream img_name;
  img_name << msg_time_seconds << ".png";
  //std::cout << img_name.str() << "\n";
  imwrite(  img_name.str(), src_rgb_cv );
  cout << img_name.str() << " written with "<< msg->utime << " | "<< msg->images[0].width <<" x "<< msg->images[0].height <<"\n";

}

int main(int argc, char ** argv) {
  ConciseArgs parser(argc, argv, "registeration-app");
  string camera_in="CAMERA";
  int period_between_files = 1; // 1 = 1Hz. 5 = 0.2Hz etc
  parser.add(camera_in, "i", "in", "Incoming Multisense channel");
  parser.add(period_between_files, "p","period_between_files", "period between png output files");
  parser.parse();
  cout << camera_in << " is camera_in\n"; 

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM); 
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  image_tool app(lcm,camera_in,period_between_files);
  cout << "Ready image tool" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}
