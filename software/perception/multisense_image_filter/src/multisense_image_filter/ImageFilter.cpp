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

        void applyFilters(const  bot_core::images_t* msg);
        float computeIntensity(unsigned char * rgb, int row, int col, int width);
        void filterLowTexture(cv::Mat& disparity, unsigned char * rgb, int width, int height, int windowSize, double threshold);
        void sobel(cv::Mat& disparity, unsigned char * rgb, int width, int height, int windowSize, double threshold);
        Pass::Ptr pass;

        image_io_utils*  imgutils_;
        BotParam* botparam_;
        CameraParams camera_params_;
        uint8_t* img_buf_;
        multisense_image_utils miu_;
        int rows;
        int cols;
        float thresh;

        //speckle filter parameters
        float depthThreshold = 100;
        float sizeThreshold = 6000;

        //filter for textureless area removal
        float windowSize = 5;
        float gradientSize = 25;

        //filter for textureless area removal based on Sobel operator
        float sobelWindowSize = 5;
        float sobelGradientSize = 60;

        //removing horizontal edges
        //disparity cannot be computed accurately for edges which are parallel to the epipolar line - in this case horizontal edges
        bool removeHorizontalEdges = true;

};

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


void Main::applyFilters(const  bot_core::images_t* msg){
    int64_t msg_time = msg->utime;

    // Uncompress the RGB image, apply a mask and re compress
    imgutils_->decodeImageToRGB(&(msg->images[0]), img_buf_);

    // Uncompress the Disparity image, apply a mask
    uint8_t* buf = imgutils_->unzipImage( &(msg->images[1]) );
    pass->applyMask(msg_time, (uint16_t*) buf, 0, 0);

    //Sobel operator to filter out textureless areas
    miu_.sobelEdgeFilter((unsigned short *) buf, (unsigned char *) img_buf_, cols, rows, sobelWindowSize, sobelGradientSize, removeHorizontalEdges);

    //Remove speckles
    cv::Mat1w disparity(rows, cols, (uint16_t*) buf);
    miu_.removeSmall(disparity, thresh, sizeThreshold);

    buf = disparity.data;
    bot_core::image_t image = imgutils_->zipImage(buf, msg_time, camera_params_.width, camera_params_.height, 2);

    // Transmit the CAMERA images with filters applied
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

    applyFilters(msg);
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
