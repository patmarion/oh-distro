cmake_minimum_required(VERSION 2.8.12)
project(sicktoolbox_wrapper)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-deprecated-declarations")
set(POD_NAME sicktoolbox_wrapper)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  sensor_msgs
  std_srvs
  roslint
  pods
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs tf  sensor_msgs pods
  DEPENDS system_lib
)

###########
## Build ##
###########

set(TARGETS
  ros2lcm_sick
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Driver (publishes ROS)
#add_executable(sicklms5xx-app src/sicklms5xx.cpp)
#target_link_libraries(sicklms5xx-app ${catkin_LIBRARIES} sicklms5xx)
#pods_use_pkg_config_packages(sicklms5xx-app eigen3)

## Simple Translator:
add_executable(ros2lcm_sick src/ros2lcm_sick.cpp)
target_link_libraries(ros2lcm_sick ${catkin_LIBRARIES})
pods_use_pkg_config_packages(ros2lcm_sick  eigen3 lcm  bot2-core)

#############
## Install ##
#############

install(TARGETS ${TARGETS}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE
        )
install(DIRECTORY resources/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resources
        PATTERN ".svn" EXCLUDE
        )

#############
## Linting ##
#############
set(ROSLINT_CPP_OPTS "--filter=-whitespace/line_length,-runtime/references,-runtime/indentation_namespace,-whitespace/braces,-readability/todo")
roslint_cpp()
