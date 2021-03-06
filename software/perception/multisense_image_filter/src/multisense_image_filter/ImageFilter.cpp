#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core/image_t.hpp>
#include <lcmtypes/bot_core/images_t.hpp>

#include <image-passthrough/image-passthrough-app.hpp>
#include <multisense_image_utils/multisense_image_utils.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include <ConciseArgs>
using namespace std;

// uncomment to enable benchmarking
//#define BENCHMARK

#ifdef BENCHMARK
#include <chrono>
using namespace std::chrono;
#endif

class Main{
    public:
        enum mask { none, robot, environment };

        Main(int argc, char** argv, boost::shared_ptr<lcm::LCM> &publish_lcm,
             std::string camera_channel, int output_color_mode_,
             bool use_convex_hulls, string camera_frame,
             bool verbose, bool use_mono, unsigned int mask_edge_size, bool apply_sobel_filter,
             mask mask_type);

        ~Main(){
        }


        static const std::map<std::string, mask> mask_map;

    private:
        boost::shared_ptr<lcm::LCM> lcm_;
        void multisenseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                               const  bot_core::images_t* msg);
        void cameraHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                                 const  bot_core::image_t* msg);

        void applyFilters(const  bot_core::images_t* msg);

        /**
         * @brief growMask close holes in mask and grow a trust region
         * @param growed_mask pointer to array for new mask, memory will be allocated if NULL
         * @param size size of trust region, e.g. dimensions of the dilate kernel
         */
        void growMask(std::vector<uint8_t> &growed_mask, const unsigned int size = 0);

        /**
         * @brief applyMask filter image with own provided mask
         * @param mask vector with mask elements
         * @param image image buffer to filter
         */
        void applyMask(const std::vector<uint8_t> &mask, uint16_t* const image);

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

        unsigned int mask_edge_size;

        // activate filtering of textureless areas
        bool apply_sobel_filter;

        // store type of mask that should be applied to the data
        mask mask_type;

};

const std::map<std::string, Main::mask> Main::mask_map = {
    {"none", mask::none}, {"robot", mask::robot}, {"environment", mask::environment}
};

Main::Main(int argc, char** argv, boost::shared_ptr<lcm::LCM> &lcm_, 
           std::string camera_channel, int output_color_mode, 
           bool use_convex_hulls, std::string camera_frame,
           bool verbose, bool use_mono, unsigned int mask_edge_size, bool sobel_filter,
           mask mask_type):
    lcm_(lcm_), mask_edge_size(mask_edge_size), mask_type(mask_type), apply_sobel_filter(sobel_filter)
{

    // Get Camera Parameters:
    botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
    camera_params_.setParams(botparam_, string("cameras." + camera_channel) );

    cols = bot_param_get_double_or_fail(botparam_, "cameras.CAMERA_LEFT.intrinsic_cal.width");
    rows = bot_param_get_double_or_fail(botparam_, "cameras.CAMERA_LEFT.intrinsic_cal.height");

    double pinhole[5];
    double translation[3];

    bot_param_get_double_array_or_fail(botparam_, "cameras.CAMERA_LEFT.intrinsic_cal.pinhole", pinhole, 5);

    std::cout << "this code needs to be fixed to find the translation between CAMERA_LEFT AND CAMERA_RIGHT\n";
    exit(-1);
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

#ifdef BENCHMARK
    std::vector<milliseconds> times;
    times.push_back(duration_cast<milliseconds>(system_clock::now().time_since_epoch()));
#endif

    int64_t msg_time = msg->utime;

    // Uncompress the RGB image, apply a mask and re compress
    imgutils_->decodeImageToRGB(&(msg->images[0]), img_buf_);

    // Uncompress the Disparity image, apply a mask
    uint8_t* buf = imgutils_->unzipImage( &(msg->images[1]) );

    if(mask_type!=mask::none) {
        // fill holes in mask
        std::vector<uint8_t> mask;
        growMask(mask, mask_edge_size);

#ifdef BENCHMARK
        times.push_back(duration_cast<milliseconds>(system_clock::now().time_since_epoch()));
        std::cout<<"grow mask: "<<(times.end()[-1]-times.end()[-2]).count()<<" ms"<<std::endl;
#endif

        // apply mask to 16bit depth image
        applyMask(mask, (uint16_t*)buf);
    }

#ifdef BENCHMARK
    times.push_back(duration_cast<milliseconds>(system_clock::now().time_since_epoch()));
    std::cout<<"apply mask: "<<(times.end()[-1]-times.end()[-2]).count()<<" ms"<<std::endl;
#endif

    cv::Mat1w disparity(rows, cols, (uint16_t*) buf);
    if(apply_sobel_filter) {
        //Sobel operator to filter out textureless areas
        miu_.sobelEdgeFilter((unsigned short *) buf, (unsigned char *) img_buf_, cols, rows, sobelWindowSize, sobelGradientSize, removeHorizontalEdges);
#ifdef BENCHMARK
        times.push_back(duration_cast<milliseconds>(system_clock::now().time_since_epoch()));
        std::cout<<"sobel: "<<(times.end()[-1]-times.end()[-2]).count()<<" ms"<<std::endl;
#endif

        //Remove speckles
        miu_.removeSmall(disparity, thresh, sizeThreshold);
    }

#ifdef BENCHMARK
    times.push_back(duration_cast<milliseconds>(system_clock::now().time_since_epoch()));
    std::cout<<"speckle: "<<(times.end()[-1]-times.end()[-2]).count()<<" ms"<<std::endl;
#endif

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

#ifdef BENCHMARK
    std::cout<<"filter total: "<<(times.back()-times.front()).count()<<" ms"<<std::endl;
#endif
}

void Main::growMask(std::vector<uint8_t> &growed_mask, const unsigned int size) {
    // get dimensions
    const unsigned int w = pass->getBufferWidth();
    const unsigned int h = pass->getBufferHeight();

    // allocate memory
    growed_mask = std::vector<uint8_t>(w*h, 0);

    // copy original mask, we cannot write to the original memory
    std::memcpy(growed_mask.data(), pass->getColorBuffer(1), w*h);

    // create cv image from memory
    cv::Mat_<uint8_t> mask(h, w, growed_mask.data());

    // workaround to find contours of holes that connect to the image border
    cv::rectangle(mask, cv::Point(0,0), cv::Point(w,h), cv::Scalar(255), 2);

    // morphological operator: closing
    const cv::Mat kern_open = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7,7));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kern_open);

    // fill holes
    std::vector<std::vector<cv::Point> > cont;
    cv::findContours(mask, cont, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    cv::drawContours(mask, cont, -1, cv::Scalar(255), -1);

    // undo workaround
    cv::rectangle(mask, cv::Point(0,0), cv::Point(w,h), cv::Scalar(0), 2);

    // grow mask
    if(size>0) {
        const cv::Mat kern_dilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(size,size));
        cv::dilate(mask, mask, kern_dilate);
    }

    // set mask according to requested part of image
    switch(mask_type) {
    case Main::mask::robot:
        cv::threshold(mask, mask, 0, 255, cv::THRESH_BINARY);
        break;
    case Main::mask::environment:
        cv::threshold(mask, mask, 0, 255, cv::THRESH_BINARY_INV);
        break;
    default:
        std::cerr<<"Undefined mask! This should not happen."<<std::endl;
    }
}

void Main::applyMask(const std::vector<uint8_t> &mask, uint16_t* const image) {
    for (unsigned int i=0; i<mask.size(); i++)
        if(mask[i]==0) image[i] = 0;
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
    unsigned int mask_edge_size = 0;
    bool sobel_filter = false;
    std::string mask_type = "none";
    parser.add(camera_channel, "c", "camera_channel", "Camera channel");
    parser.add(camera_frame, "f", "camera_frame", "Camera frame");
    parser.add(output_color_mode, "o", "output_color_mode", "0rgb |1grayscale |2b/w");
    parser.add(use_convex_hulls, "u", "use_convex_hulls", "Use convex hull models");
    parser.add(verbose, "v", "verbose", "Verbose");
    parser.add(use_mono, "m", "use_mono", "Key off of the left monocularimage");
    parser.add(mask_edge_size, "b", "mask_edge_size", "Mask border size");
    parser.add(sobel_filter, "s", "sobel_filter", "filter textureless areas using Sobel operator");
    parser.add(mask_type, "t", "mask_type", "specify object to filter ('robot', 'environment') or 'none'");
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
              use_mono, mask_edge_size, sobel_filter,
              Main::mask_map.at(mask_type));
    cout << "image-filter ready" << endl << endl;
    while(0 == lcm->handle());
    return 0;
}
