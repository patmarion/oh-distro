set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(state_sync)

add_executable(se-test-body-to-local-rotations body_to_local_rotations.cpp  )
pods_use_pkg_config_packages(se-test-body-to-local-rotations eigen3
  lcmtypes_drc_lcmtypes bot2-param-client bot2-frames)
pods_install_executables(se-test-body-to-local-rotations)


add_executable(se-ers-delay se-ers-delay.cpp)
pods_use_pkg_config_packages(se-ers-delay eigen3
  lcmtypes_drc_lcmtypes bot2-param-client bot2-frames bot2-frames_cpp)
pods_install_executables(se-ers-delay)


add_executable(se-pose-to-delta pose_to_delta.cpp)
pods_use_pkg_config_packages(se-pose-to-delta eigen3
  lcmtypes_drc_lcmtypes bot2-param-client bot2-frames bot2-frames_cpp pronto_math)
pods_install_executables(se-pose-to-delta)
