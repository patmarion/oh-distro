// example usage, tested wit valkyrie data:
// drc-image-passthrough -o 1
// june 2016

#include <image-passthrough/image-passthrough-app.hpp>

#include <ConciseArgs>
using namespace std;


class Main{
  public:
    Main(int argc, char** argv, boost::shared_ptr<lcm::LCM> &publish_lcm, 
         std::string camera_channel, int output_color_mode_, 
         bool use_convex_hulls, string camera_frame,
         bool verbose, bool use_mono);
    
    ~Main(){
    }

  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    void multisenseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, 
                           const  bot_core::images_t* msg);    
    void cameraHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, 
                             const  bot_core::image_t* msg);

    void sendMasked(const  bot_core::images_t* msg);
    Pass::Ptr pass;
    
    image_io_utils*  imgutils_; 
    BotParam* botparam_; 
    CameraParams camera_params_;   
    uint8_t* img_buf_;


  int depth_compress_buf_size_;
  uint8_t* depth_compress_buf_;
};
    
    
Main::Main(int argc, char** argv, boost::shared_ptr<lcm::LCM> &lcm_, 
           std::string camera_channel, int output_color_mode, 
           bool use_convex_hulls, std::string camera_frame,
           bool verbose, bool use_mono): lcm_(lcm_){

  // Get Camera Parameters:
  botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
  camera_params_.setParams(botparam_, string("cameras." + camera_channel) );
      
  pass = Pass::Ptr (new Pass (argc, argv, lcm_, 
                              camera_channel, output_color_mode, 
                              use_convex_hulls, camera_frame, 
                              camera_params_, verbose));
  if (use_mono){
    lcm::Subscription* sub = lcm_->subscribe("CAMERA_LEFT",&Main::cameraHandler,this);  
    sub->setQueueCapacity(1);
  }else{
    lcm::Subscription* sub = lcm_->subscribe("MULTISENSE_CAMERA",&Main::multisenseHandler,this);
    sub->setQueueCapacity(1);
  }
  
  img_buf_= (uint8_t*) malloc(3* camera_params_.width  * camera_params_.height);
  imgutils_ = new image_io_utils( lcm_, 
                                  camera_params_.width, 
                                  3*camera_params_.height );

  
}


void Main::sendMasked(const  bot_core::images_t* msg){
  int64_t msg_time = msg->utime;

  // Uncompress the RGB image, apply a mask and re compress  
  imgutils_->decodeImageToRGB(&(msg->images[0]),  img_buf_ );
  pass->applyMask(msg_time, img_buf_, 0, 0);
  bot_core::image_t msgout = imgutils_->jpegImage(img_buf_, 
    msg_time, camera_params_.width, camera_params_.height, 95, 3);


  // Uncompress the Disparity image, apply a mask and recompress
  uint8_t* buf = imgutils_->unzipImage( &(msg->images[1]) );
  pass->applyMask(msg_time, (uint16_t*) buf, 0, 0);
  bot_core::image_t image = imgutils_->zipImage(buf, msg_time, camera_params_.width, camera_params_.height, 2);

  // Transmitt the CAMERA images with a mask
  bot_core::images_t ms;
  ms.images.push_back( (msgout) );
  ms.images.push_back(image);
  ms.image_types.push_back( msg->image_types[0]);// left
  ms.image_types.push_back( msg->image_types[1]);// disparity
  ms.n_images = ms.images.size();
  ms.utime = msg->utime;
  lcm_->publish("CAMERA_MASKED", &ms);
}


void Main::multisenseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, 
                             const  bot_core::images_t* msg){

  if (!pass->createMask(msg->utime) )
    return;

  pass->sendOutput(msg->utime);  

  sendMasked(msg);
}


void Main::cameraHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, 
                             const  bot_core::image_t* msg){
  int64_t msg_time = msg->utime;
  if (!pass->createMask(msg_time) )
    return;

  pass->sendOutput(msg_time);  
  
  imgutils_->decodeImageToRGB( msg,  img_buf_ );
  
  pass->overlayMask(msg_time, img_buf_);

  imgutils_->sendImage( img_buf_, msg_time,
                        camera_params_.width, camera_params_.height, 3,
                        string( channel +  "_OVERLAY") );    
  
}

int main( int argc, char** argv ){
  ConciseArgs parser(argc, argv, "lidar-passthrough");
  string camera_channel="CAMERA_LEFT";
  int output_color_mode=1; // 0 =rgb, 1=grayscale mask, 2=binary black/white grayscale mask
  bool use_convex_hulls=false;
  string camera_frame = "left_camera_optical_frame";
  bool verbose = false;
  bool use_mono = false;
  parser.add(camera_channel, "c", "camera_channel", "Camera channel");
  parser.add(camera_frame, "f", "camera_frame", "Camera frame");
  parser.add(output_color_mode, "o", "output_color_mode", "0rgb |1grayscale |2b/w");
  parser.add(use_convex_hulls, "u", "use_convex_hulls", "Use convex hull models");
  parser.add(verbose, "v", "verbose", "Verbose");
  parser.add(use_mono, "m", "use_mono", "Key off of the left monocularimage");  
  parser.parse();
  cout << camera_channel << " is camera_channel\n"; 
  cout << camera_frame << " is camera_frame\n"; 
  cout << output_color_mode << " is output_color_mode\n"; 
  cout << use_convex_hulls << " is use_convex_hulls\n"; 
  cout << verbose << " is verbose\n";
  cout << use_mono << " is use_mono\n";
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Main main(argc,argv, lcm, 
            camera_channel,output_color_mode, 
            use_convex_hulls, camera_frame, verbose,
           use_mono);
  cout << "image-passthrough ready" << endl << endl;
  while(0 == lcm->handle());
  return 0;
}
