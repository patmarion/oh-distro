#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <math.h>

#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_io_utils/image_io_utils.hpp> // to simplify jpeg/zlib compression and decompression
#include <ConciseArgs>

using namespace cv;
using namespace std;

class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_, std::string image_channel_,
      int jpeg_quality_, int mode , int downsample_);
    
    ~Pass(){
    }    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    void imageHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::image_t* msg);  

    void sendOutput();

    int jpeg_quality_;

    int width_;
    int height_;
    int counter_;
    std::string image_channel_;
    int downsample_;

    int mode_;

    image_io_utils*  imgutils_;
  
    bot_core::image_t last_img_;  

    int img_buf_size_;
    uint8_t* img_buf_;
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_, std::string image_channel_, 
           int jpeg_quality_, int mode_ , int downsample_):
    lcm_(lcm_), jpeg_quality_(jpeg_quality_), mode_(mode_),
    image_channel_(image_channel_), downsample_(downsample_){

  lcm_->subscribe( image_channel_ ,&Pass::imageHandler,this);

  width_ = 1024;
  height_ = 1024;
  imgutils_ = new image_io_utils( lcm_, width_, height_ );
  counter_=0;
  last_img_.utime=0; // used to indicate no message recieved yet

  img_buf_= (uint8_t*) malloc(10* width_  * height_);
}

void Pass::sendOutput(){
  if (last_img_.utime==0){     return;   } // if no msg recieved then ignore output command

  if (mode_==0){ // jpeg and resend 
    Mat src= Mat::zeros( last_img_.height,last_img_.width  ,CV_8UC3);
    src.data = last_img_.data.data();
    imgutils_->sendImageJpeg(src.data, last_img_.utime, 
                last_img_.width,last_img_.height, jpeg_quality_, string(image_channel_ + "_COMPRESSED"), 3);
    
  }else if(mode_==1){ // unzip and resend
    uint8_t* buf = imgutils_->unzipImage( &(last_img_) );// , image_channel_);
    imgutils_->sendImage(buf, last_img_.utime, last_img_.width, 
                         last_img_.height, 1, string(image_channel_ + "_UNZIPPED")  );
  }else if(mode_==2){ // rotate 180 and resend
    Mat src= Mat::zeros( last_img_.height,last_img_.width  ,CV_8UC3);
    src.data = last_img_.data.data();

    Mat img = Mat::zeros( last_img_.height,last_img_.width ,CV_8UC3);
    cv::flip(src, img, -1);

    imgutils_->sendImageJpeg(img.data, last_img_.utime, 
                last_img_.width,last_img_.height, jpeg_quality_, string(image_channel_ + "_ROTATED"), 3);
  }else if(mode_==3){ // save to png

    Mat src= Mat::zeros( last_img_.height,last_img_.width  ,CV_8UC3);
    if (last_img_.pixelformat == bot_core::image_t::PIXEL_FORMAT_MJPEG){
      imgutils_->decodeImageToRGB( &(last_img_), img_buf_);
      src.data = img_buf_;
    }else{
      src.data = last_img_.data.data();
    }

    Mat src_rgb_cv;

    cvtColor( src, src_rgb_cv, CV_BGR2RGB );

    std::stringstream img_name;
    img_name << "images/" << counter_ << ".png";
    std::cout << img_name.str() << "\n";

    imwrite(  img_name.str(), src_rgb_cv );
  }
}


void Pass::imageHandler(const lcm::ReceiveBuffer* rbuf, 
                        const std::string& channel, const  bot_core::image_t* msg){
  counter_++;
  if (counter_%30 ==0){ cout << counter_ << " | " << msg->utime << "\n";   }  
  if (width_ != msg->width){
    cout << "incoming width " << msg->width << " doesn't match assumed width " << width_ << "\n";
    cout << "returning cowardly\n";
    return;
  }

  last_img_= *msg;  
  if(downsample_==-1){   return;   }
  if (counter_% downsample_ ==0){ sendOutput();  }    
}



int main(int argc, char ** argv) {
  cout << "============= QUICK MODES ===================\n";
  cout << "drc-image-tool  -m 0 -c CAMERALEFT\n";
  cout << "drc-image-tool  -m 1 -c CAMERALEFT_MASKZIPPED\n";
  cout << "=============================================\n";

  int jpeg_quality = 50;
  string channel = "CAMERALEFT";
  int mode=0;
  int downsample = -1;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(jpeg_quality, "j", "jpeg_quality","jpeg_quality");
  opt.add(channel, "c", "channel","channel");
  opt.add(mode, "m", "mode","0=rgbinJPEGOUT 1=zipinGRAYOUT, 2=rotate180, 3=write to file");
  opt.add(downsample, "d", "downsample","Downsample Factor");
  opt.parse();
  std::cout << "jpeg_quality: " << jpeg_quality << "\n";  
  std::cout << "channel: " << channel << "\n";
  std::cout << "mode: " << mode << "\n";    
  std::cout << "downsample: output 1 in every [" << downsample << "] frames\n";
  std::cout << "            (set as -1 to output none regularly)\n";    
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm,channel, jpeg_quality, mode, downsample);
  cout << "Tool ready" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}
