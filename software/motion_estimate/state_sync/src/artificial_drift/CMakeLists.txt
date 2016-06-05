set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(artificial_drift)


add_executable(drc-artificial-drift artificial_drift.cpp  )
pods_use_pkg_config_packages(drc-artificial-drift eigen3
  model-client bot2-param-client pronto_math
  lcmtypes_drc_lcmtypes lcmtypes_bot2-core)
pods_install_executables(drc-artificial-drift)
