#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/bot_core.hpp"

#include <image-passthrough/image-passthrough-app.hpp>
#include <multisense_image_utils/multisense_image_utils.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"

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
        float computeIntensity(unsigned char * rgb, int row, int col, int width);
        void filterLowTexture(cv::Mat& disparity, unsigned char * rgb, int width, int height, int windowSize, double threshold);
        void sobel(cv::Mat& disparity, unsigned char * rgb, int width, int height, int windowSize, double threshold);
        Pass::Ptr pass;

        image_io_utils*  imgutils_;
        BotParam* botparam_;
        CameraParams camera_params_;
        uint8_t* img_buf_;
        multisense_image_utils miu_;
        float thresh;
        float depthThreshold = 100;
        float sizeThreshold = 6000;
        float windowSize = 5;
        float gradientSize = 25;
        float sobelWindowSize = 5;
        float sobelGradientSize = 60;
        float parallelLineCos = 0.99;
        int rows;
        int cols;
};


float Main::computeIntensity(unsigned char * rgb, int row, int col, int width) {

    return 0.2126 * rgb[3 * (row * width + col) + 0] + 0.7152 * rgb[3 * (row * width + col) + 1] + 0.0722 * rgb[3 * (row * width + col) + 2];
}

void Main::filterLowTexture(cv::Mat& disparity, unsigned char * rgb, int width, int height, int windowSize, double threshold) {

    for (int i=windowSize; i<height-windowSize; i++) {
       for (int j=windowSize; j<width-windowSize; j++) {

          float tl = computeIntensity(rgb, i-windowSize, j-windowSize, width);
          float br = computeIntensity(rgb, i+windowSize, j+windowSize, width);

          float tr = computeIntensity(rgb, i-windowSize, j+windowSize, width);
          float bl = computeIntensity(rgb, i+windowSize, j-windowSize, width);

          float tv = computeIntensity(rgb, i-windowSize, j, width);
          float bv = computeIntensity(rgb, i+windowSize, j, width);

          float lh = computeIntensity(rgb, i, j-windowSize, width);
          float rh = computeIntensity(rgb, i, j+windowSize, width);


          float d1 = fabs(tl - br);
          float d2 = fabs(tr - bl);
          float d3 = fabs(tv - bv);
          float d4 = fabs(lh - rh);

          float cos = d3 / sqrt(d4*d4 + d3*d3);

          if (cos >= parallelLineCos) {
              disparity.at<short>(i,j) = 0;
          }

          if (d1 < threshold && d2 < threshold && d3 < threshold && d4 < threshold) {
             disparity.at<short>(i,j) = 0;
          }
       }
    }
}

void Main::sobel(cv::Mat& disparity, unsigned char * rgb, int width, int height, int windowSize, double threshold) {

    for (int i=windowSize; i<height-windowSize; i++) {
       for (int j=windowSize; j<width-windowSize; j++) {

          float tl = computeIntensity(rgb, i-windowSize, j-windowSize, width);
          float br = computeIntensity(rgb, i+windowSize, j+windowSize, width);

          float tr = computeIntensity(rgb, i-windowSize, j+windowSize, width);
          float bl = computeIntensity(rgb, i+windowSize, j-windowSize, width);

          float tv = computeIntensity(rgb, i-windowSize, j, width);
          float bv = computeIntensity(rgb, i+windowSize, j, width);

          float lh = computeIntensity(rgb, i, j-windowSize, width);
          float rh = computeIntensity(rgb, i, j+windowSize, width);

          float gx = -1 * tl + 1 * tr - 2 * lh + 2 * rh - 1 * bl + 1 * br;
          float gy = -1 * tl + 1 * bl - 2 * tv + 2 * bv - 1 * tr + 1 * br;

          float g = sqrt(gx*gx + gy*gy);

          float d3 = fabs(tv - bv);
          float d4 = fabs(lh - rh);

          float cos = d3 / sqrt(d4*d4 + d3*d3);

          if (cos >= parallelLineCos) {
              disparity.at<short>(i,j) = 0;
          }

          if (g<threshold) {
             disparity.at<short>(i,j) = 0;
         }
       }
    }
}
    
Main::Main(int argc, char** argv, boost::shared_ptr<lcm::LCM> &lcm_, 
           std::string camera_channel, int output_color_mode, 
           bool use_convex_hulls, std::string camera_frame,
           bool verbose, bool use_mono): lcm_(lcm_){

    // Get Camera Parameters:
    botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
    camera_params_.setParams(botparam_, string("cameras." + camera_channel) );

    cols = bot_param_get_double_or_fail(botparam_, "cameras.CAMERA_LEFT.intrinsic_cal.width");
    rows = bot_param_get_double_or_fail(botparam_, "cameras.CAMERA_LEFT.intrinsic_cal.height");

    double pinhole[5];
    double translation[3];

    bot_param_get_double_array_or_fail(botparam_, "cameras.CAMERA_LEFT.intrinsic_cal.pinhole", pinhole, 5);
    bot_param_get_double_array_or_fail(botparam_, "cameras.CAMERA.translation", translation, 3);

    double fx = pinhole[0];
    double baseline = fabs(translation[0]);

    float k00 =  1./float(fx);
    float mDisparityFactor = 1/k00/baseline;

    thresh = 16.0/mDisparityFactor/depthThreshold;

    pass = Pass::Ptr (new Pass (argc, argv, lcm_,
                              camera_channel, output_color_mode,
                              use_convex_hulls, camera_frame,
                              camera_params_, verbose));
    if (use_mono){
        lcm::Subscription* sub = lcm_->subscribe("CAMERA_LEFT",&Main::cameraHandler,this);
        sub->setQueueCapacity(1);
    }else{
        lcm::Subscription* sub = lcm_->subscribe("CAMERA",&Main::multisenseHandler,this);
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
    imgutils_->decodeImageToRGB(&(msg->images[0]), img_buf_);

    // Uncompress the Disparity image, apply a mask
    uint8_t* buf = imgutils_->unzipImage( &(msg->images[1]) );
    pass->applyMask(msg_time, (uint16_t*) buf, 0, 0);

    //Apply Sobel edge detection
    cv::Mat1w disparity(rows, cols, (uint16_t*) buf);
    sobel(disparity, (unsigned char *) img_buf_, cols, rows, sobelWindowSize, sobelGradientSize);

    //Remove speckles
    miu_.removeSmall(disparity, thresh, sizeThreshold);

    buf = disparity.data;
    bot_core::image_t image = imgutils_->zipImage(buf, msg_time, camera_params_.width, camera_params_.height, 2);

    // Transmitt the CAMERA images with filters applied
    bot_core::images_t ms;
    ms.images.push_back( msg->images[0] );
    ms.images.push_back(image);
    ms.image_types.push_back( msg->image_types[0]);// left
    ms.image_types.push_back( msg->image_types[1]);// disparity
    ms.n_images = ms.images.size();
    ms.utime = msg->utime;
    lcm_->publish("CAMERA_FILTERED", &ms);
}


void Main::multisenseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, 
                             const  bot_core::images_t* msg){

    if (!pass->createMask(msg->utime) )
    return;

    pass->sendOutput(msg->utime);

    sendMasked(msg);
}


void Main::cameraHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::image_t* msg) {
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
    cout << "image-filter ready" << endl << endl;
    while(0 == lcm->handle());
    return 0;
}
