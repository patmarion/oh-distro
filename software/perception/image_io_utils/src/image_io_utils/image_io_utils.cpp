#include <iostream>
#include "image_io_utils.hpp"



//#include "jpeg-utils.h"
//#include "jpeg-utils-ijg.c"
//#include "jpeg-utils-ijg.h"

#include <zlib.h>


using namespace std;


image_io_utils::image_io_utils (boost::shared_ptr<lcm::LCM> &publish_lcm_, int width_, int height_):
        publish_lcm_(publish_lcm_){

  // Maximum size of reserved buffer. This will support and images smaller than this also:
  local_img_buffer_size_ = width_ * height_ * sizeof(uint8_t) * 10;
//  local_img_buffer_= new uint8_t[local_img_buffer_size_];  // x4 was used for zlib in kinect_lcm
  local_img_buffer_ = (uint8_t*)malloc(local_img_buffer_size_);  

  // is x10 necessary for  jpeg? thats waht kinect_lcm assumed
}

/// Added for RGB-to-Gray:
int pixel_convert_8u_rgb_to_8u_gray (uint8_t *dest, int dstride, int width,
        int height, const uint8_t *src, int sstride)
{
  int i, j;
  for (i=0; i<height; i++) {
    uint8_t *drow = dest + i * dstride;
    const uint8_t *srow = src + i * sstride;
    for (j=0; j<width; j++) {
      drow[j] = 0.2125 * srow[j*3+0] +
        0.7154 * srow[j*3+1] +
        0.0721 * srow[j*3+2];
    }
  }
  return 0;
}

// decodes to gray scale, assumes image is stacked
void image_io_utils::decodeStereoImageToGray(const  bot_core::image_t* msg, uint8_t* left_buf, uint8_t* right_buf){
  int h = msg->height/2; /// stacked stereo
  int w = msg->width;
  
  int buf_size = w*h;
  
  switch (msg->pixelformat) {
    case bot_core::image_t::PIXEL_FORMAT_GRAY:
      memcpy(left_buf,  msg->data.data() , msg->size/2);
      memcpy(right_buf,  msg->data.data() + msg->size/2 , msg->size/2);
      break;
    case bot_core::image_t::PIXEL_FORMAT_RGB:
      pixel_convert_8u_rgb_to_8u_gray(  left_buf, w, w, h, msg->data.data(),  w*3);
      pixel_convert_8u_rgb_to_8u_gray(  right_buf, w,  w, h, msg->data.data() + msg->size/2,  w*3);
      break;
    case bot_core::image_t::PIXEL_FORMAT_MJPEG:
      // This assumes Gray Scale MJPEG
      jpeg_decompress_8u_gray(msg->data.data(), msg->size, local_img_buffer_,
                              msg->width, msg->height, msg->width);
      std::copy(local_img_buffer_          , local_img_buffer_+buf_size   , left_buf);
      std::copy(local_img_buffer_+buf_size , local_img_buffer_+2*buf_size , right_buf);
      break;
    default:
      std::cout << "Unrecognized image format\n";
      exit(-1);
      break;
  }  
}

// decodes to gray scale
void image_io_utils::decodeImageToGray(const  bot_core::image_t* msg, uint8_t* img_buf){
  int h = msg->height;
  int w = msg->width;
  
  int buf_size = w*h;
  
  switch (msg->pixelformat) {
    case bot_core::image_t::PIXEL_FORMAT_GRAY:
      memcpy(img_buf,  msg->data.data() , msg->size);
      break;
    case bot_core::image_t::PIXEL_FORMAT_RGB:
      pixel_convert_8u_rgb_to_8u_gray(  img_buf, w, w, h, msg->data.data(),  w*3);
      break;
    case bot_core::image_t::PIXEL_FORMAT_MJPEG:
      // This assumes Gray Scale MJPEG
      jpeg_decompress_8u_gray(msg->data.data(), msg->size, local_img_buffer_,
                              msg->width, msg->height, msg->width);
      std::copy(local_img_buffer_          , local_img_buffer_+buf_size   , img_buf);
      break;
    default:
      std::cout << "Unrecognized image format gray\n";
      exit(-1);
      break;
  }  
}

void image_io_utils::decodeImageToRGB(const  bot_core::image_t* msg, uint8_t* img_buf){
  int h = msg->height;
  int w = msg->width;
  int buf_size = w*h*3;
  
  switch (msg->pixelformat) {
    // Other formats not supported yet
    case bot_core::image_t::PIXEL_FORMAT_MJPEG:
      jpeg_decompress_8u_rgb(msg->data.data(), msg->size, local_img_buffer_,
                              w, h, w*3);  
      std::copy(local_img_buffer_          , local_img_buffer_+buf_size   , img_buf);
      break;
    case bot_core::image_t::PIXEL_FORMAT_RGB:
      img_buf =  (uint8_t*) msg->data.data();
      //*img_buf = msg->data.data()[0];
      break;
    default:
      std::cout << "Unrecognized image format rgb\n";
      exit(-1);
      break;
  }    
}


bot_core::image_t image_io_utils::jpegImage(uint8_t* buffer, int64_t utime, int width, int height, int jpeg_quality, int n_colors){

  int compressed_size =  width*height*n_colors;//image_buf_size;
  int compression_status  = -1;
  if (n_colors == 1){
    int compression_status = jpeg_compress_8u_gray(buffer, width, height, width*n_colors,
                                                     local_img_buffer_, &compressed_size, jpeg_quality);
  }else if( n_colors ==3) {
    int compression_status = jpeg_compress_8u_rgb  (buffer, width, height, width*n_colors,
                                                     local_img_buffer_, &compressed_size, jpeg_quality);
  }else {
    std::cout << "number if colors is no correct " << n_colors << "\n";
    
    exit(-1);
  }

  bot_core::image_t msgout;
  msgout.utime = utime;
  msgout.width = width;
  msgout.height = height;
  msgout.row_stride = n_colors*width;
  msgout.size = compressed_size;
  msgout.pixelformat = bot_core::image_t::PIXEL_FORMAT_MJPEG;

  msgout.data.resize(compressed_size);
  memcpy(&msgout.data[0], local_img_buffer_, compressed_size);

  msgout.nmetadata =0;
  return msgout;
}



bot_core::image_t image_io_utils::zipImage(uint8_t* buffer, int64_t utime, int width, int height, int n_colors){

  int  isize = height * width;
  int uncompressed_size = n_colors*isize;
  unsigned long compressed_size = local_img_buffer_size_;
  compress2( local_img_buffer_, &compressed_size, buffer, uncompressed_size,
      Z_BEST_SPEED);

  bot_core::image_t image;
  image.utime =utime;
  image.width =width;
  image.height=height;
  image.row_stride =n_colors*width; // this is useless if doing zip

  // This label will be invalid...
  image.pixelformat = bot_core::image_t::PIXEL_FORMAT_RGB;
  image.size = (int) compressed_size;
  image.data.resize(compressed_size);
  memcpy(&image.data[0], local_img_buffer_, compressed_size);
  image.nmetadata =0;

  return image;
}


// assumes a zipped gray scale image: CPP CPP CPP CPP
uint8_t* image_io_utils::unzipImage(const bot_core::image_t *msg){
  unsigned long dlen = msg->row_stride*msg->height; // msg->depth.uncompressed_size;
  uncompress(local_img_buffer_, &dlen, msg->data.data(), msg->size);
  return local_img_buffer_;
}

void image_io_utils::sendImageJpeg(uint8_t* buffer, int64_t utime, int width, int height, int jpeg_quality, std::string channel, int n_colors){
  bot_core::image_t msgout = jpegImage(buffer, utime, width, height, jpeg_quality, n_colors);
  publish_lcm_->publish(channel.c_str(),&msgout);
}

// assumes a zipped gray scale image:
void image_io_utils::sendImageUnzipped(const bot_core::image_t *msg, std::string channel){
  unsigned long dlen = msg->row_stride*msg->height;
  uncompress(local_img_buffer_, &dlen, msg->data.data(), msg->size);
  sendImage(local_img_buffer_, msg->utime, msg->width, msg->height, 1, string(channel + "_UNZIPPED")  );
}

void image_io_utils::sendImageZipped(uint8_t* buffer, int64_t utime, 
  int width, int height, int n_colors, std::string channel){
  bot_core::image_t image = zipImage(buffer, utime, width, height, n_colors);
  publish_lcm_->publish(channel.c_str(),&image);
}

void image_io_utils::sendImage(uint8_t* buffer, int64_t utime, int width, int height, int n_colors, std::string channel)
{
  int isize= width* height;
  int dsize= n_colors*isize;

  bot_core::image_t image;
  image.utime =utime;
  image.width =width;
  image.height=height;
  image.row_stride =n_colors*width;

  if (n_colors==1){
    image.pixelformat = bot_core::image_t::PIXEL_FORMAT_GRAY;
  }else{
    image.pixelformat = bot_core::image_t::PIXEL_FORMAT_RGB;
  }
  
  image.size =dsize;
  image.data.resize(dsize);
  memcpy(&image.data[0], buffer, dsize);

  image.nmetadata =0;
  publish_lcm_->publish(channel.c_str(),&image);
}
