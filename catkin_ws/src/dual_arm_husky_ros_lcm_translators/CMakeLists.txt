cmake_minimum_required(VERSION 2.8.12)
project(dual_arm_husky_ros_lcm_translators)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(POD_NAME dual_arm_husky_ros_lcm_translators)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  husky_msgs
  nav_msgs
  pods
  robotiq_force_torque_sensor_msgs
  roscpp
  sensor_msgs
  std_msgs
  trajectory_msgs
  actionlib
  move_base_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    husky_msgs
    nav_msgs
    pods
    robotiq_force_torque_sensor_msgs
    roscpp
    sensor_msgs
    std_msgs
    trajectory_msgs
    move_base_msgs
    actionlib
    tf
  DEPENDS system_lib lcm
)

###########
## Build ##
###########

add_executable(ros2lcm_dual_arm_husky src/ros2lcm_dual_arm_husky.cpp)
target_link_libraries(ros2lcm_dual_arm_husky ${catkin_LIBRARIES})
add_dependencies(ros2lcm_dual_arm_husky robotiq_force_torque_sensor_msgs_generate_messages_cpp nav_msgs_generate_messages_cpp sensor_msgs_generate_messages_cpp husky_msgs_generate_messages_cpp)
pods_use_pkg_config_packages(ros2lcm_dual_arm_husky lcm lcmtypes_bot2-core bot2-lcmgl-client lcmtypes_drc_lcmtypes)

add_executable(lcm2ros_dual_arm_husky src/lcm2ros_dual_arm_husky.cpp)
target_link_libraries(lcm2ros_dual_arm_husky ${catkin_LIBRARIES})
pods_use_pkg_config_packages(lcm2ros_dual_arm_husky lcm lcmtypes_bot2-core)


#############
## Install ##
#############

install(TARGETS ros2lcm_dual_arm_husky lcm2ros_dual_arm_husky
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
