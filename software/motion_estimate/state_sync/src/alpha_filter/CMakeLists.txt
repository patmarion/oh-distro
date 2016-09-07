cmake_minimum_required(VERSION 2.8)
project(alpha_filter)

add_library(alpha_filter AlphaFilter.cpp)
target_include_directories(alpha_filter PUBLIC .)
