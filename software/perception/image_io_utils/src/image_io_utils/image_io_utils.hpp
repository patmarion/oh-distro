#ifndef image_io_utils_HPP_
#define image_io_utils_HPP_

#include <iostream>
#include <vector>
#include <algorithm>

#include <image_utils/jpeg.h>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>
#include <boost/shared_ptr.hpp>

class image_io_utils {
  public:
    image_io_utils (boost::shared_ptr<lcm::LCM> &lcm_, int width_, int height_);
    
    void decodeStereoImageToGray(const  bot_core::image_t* msg, uint8_t* left_buf, uint8_t* right_buf);
    void decodeImageToGray(const  bot_core::image_t* msg, uint8_t* img_buf);

    void decodeImageToRGB(const  bot_core::image_t* msg, uint8_t* img_buf);
    
    void sendImageUnzipped(const bot_core::image_t *msg, std::string channel);
    uint8_t* unzipImage(const bot_core::image_t *msg);

    
    bot_core::image_t jpegImage(uint8_t* buffer, int64_t utime, int width, int height,
         int jpeg_quality, int n_colors);
    bot_core::image_t  zipImage(uint8_t* buffer, int64_t utime, int width, int height, int n_colors);


    void sendImageJpeg(uint8_t* buffer, int64_t utime, int width, int height, 
			   int jpeg_quality, std::string channel, int n_colors);

    
    void sendImageZipped(uint8_t* buffer, int64_t utime, 
                         int width, int height, int n_colors, 
                         std::string channel);

    void sendImage(uint8_t* buffer, int64_t utime, 
                   int width, int height, 
                   int n_colors, std::string channel);
  private:
    boost::shared_ptr<lcm::LCM> publish_lcm_;

    int width_, height_;

    int local_img_buffer_size_;
    uint8_t* local_img_buffer_;
};






#endif
