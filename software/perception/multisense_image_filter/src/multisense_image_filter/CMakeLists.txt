set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++0x ")

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(drc-image-filter)

find_package(VTK REQUIRED)
set(VTK_USE_FILE ${VTK_USE_FILE} CACHE INTERNAL "VTK_USE_FILE")
include (${VTK_USE_FILE})

add_executable(drc-image-filter ImageFilter.cpp)
target_link_libraries(drc-image-filter boost_system image_io_utils camera_params)
pods_use_pkg_config_packages(drc-image-filter bot2-param-client image-passthrough-app lcm pcl_io-1.7 gtk+-2.0 multisense_image_utils opencv)
pods_install_executables(drc-image-filter)
