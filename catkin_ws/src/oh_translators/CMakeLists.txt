cmake_minimum_required(VERSION 2.8.12)
project(oh_translators)

set(POD_NAME oh_translators)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-deprecated-declarations")

find_package(catkin REQUIRED COMPONENTS
  ihmc_msgs
  image_transport
  ipab_lwr_msgs
  ipab_msgs
  kdl_conversions
  octomap_msgs
  pods
  roscpp
  roslint
  sensor_msgs
  std_msgs
  std_srvs
  tf
  trajectory_msgs
)

catkin_package(
  CATKIN_DEPENDS
    ihmc_msgs
    image_transport
    ipab_lwr_msgs
    ipab_msgs
    kdl_conversions
    octomap_msgs
    pods
    roscpp
    sensor_msgs
    std_msgs
    tf
    trajectory_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

set(TARGETS
    lcm2ros_valkyrie_hands
    ros2lcm_ihmc
    lcm2ros_ihmc
    ros2lcm_camera
    ros2lcm_stereo
    ros2lcm_kinect
    lcm2ros_kuka
    ros2lcm_kuka
    ros2lcm_exotica
    lcm2ros_exotica
    ros2lcm_sdh
    lcm2ros_sdh
    ros2lcm_husky
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

######################################################
add_executable(lcm2ros_valkyrie_hands src/lcm2ros_valkyrie_hands.cpp)
target_link_libraries(lcm2ros_valkyrie_hands )
pods_use_pkg_config_packages(lcm2ros_valkyrie_hands lcm lcmtypes_robotiqhand lcmtypes_bot2-core)

######################################################
add_executable(ros2lcm_ihmc src/ros2lcm_ihmc.cpp)
target_link_libraries(ros2lcm_ihmc ${catkin_LIBRARIES} )
pods_use_pkg_config_packages(ros2lcm_ihmc lcm lcmtypes_ihmc bot2-core eigen3)
add_dependencies(ros2lcm_ihmc ipab_msgs_generate_messages_cpp ihmc_msgs_generate_messages_cpp)

set(KDL_PACKAGES urdf orocos-kdl kdl-parser forward-kinematics model-client)
add_executable(lcm2ros_ihmc src/lcm2ros_ihmc.cpp
         src/lcm2ros_ihmc_conversions.cpp  src/lcm2ros_ihmc_modes.cpp
         src/lcm2ros_ihmc_footsteps.cpp  src/lcm2ros_ihmc_motionplans.cpp
         src/lcm2ros_ihmc_pose.cpp)
target_link_libraries(lcm2ros_ihmc ${catkin_LIBRARIES})
pods_use_pkg_config_packages(lcm2ros_ihmc lcm bot2-core pronto_vis ${KDL_PACKAGES})
add_dependencies(lcm2ros_ihmc ipab_msgs_generate_messages_cpp ihmc_msgs_generate_messages_cpp)

######################################################
add_executable(ros2lcm_camera src/ros2lcm_camera.cpp)
target_link_libraries(ros2lcm_camera ${catkin_LIBRARIES})
pods_use_pkg_config_packages(ros2lcm_camera lcm bot2-core eigen3 opencv)
add_dependencies(ros2lcm_camera ipab_msgs_generate_messages_cpp ihmc_msgs_generate_messages_cpp)

add_executable(ros2lcm_stereo src/ros2lcm_stereo.cpp)
target_link_libraries(ros2lcm_stereo ${catkin_LIBRARIES} z)
pods_use_pkg_config_packages(ros2lcm_stereo lcm bot2-core eigen3 opencv)
add_dependencies(ros2lcm_stereo ipab_msgs_generate_messages_cpp ihmc_msgs_generate_messages_cpp)

add_executable(ros2lcm_test src/ros2lcm_test.cpp)
target_link_libraries(ros2lcm_test ${catkin_LIBRARIES})
add_dependencies(ros2lcm_test ipab_msgs_generate_messages_cpp ihmc_msgs_generate_messages_cpp)
pods_use_pkg_config_packages(ros2lcm_test lcm bot2-core)

add_executable(ros2lcm_kinect src/ros2lcm_kinect.cpp)
target_link_libraries(ros2lcm_kinect ${catkin_LIBRARIES} z)
pods_use_pkg_config_packages(ros2lcm_kinect lcm bot2-core opencv)

# add_executable(lcm2ros_botframes src/lcm2ros_botframes.cpp)
# target_link_libraries(lcm2ros_botframes ${catkin_LIBRARIES})
# pods_use_pkg_config_packages(lcm2ros_botframes lcm bot2-core)

######################################################
add_executable(lcm2ros_kuka src/lcm2ros_kuka.cpp)
target_link_libraries(lcm2ros_kuka ${catkin_LIBRARIES})
add_dependencies(lcm2ros_kuka ipab_msgs_generate_messages_cpp ihmc_msgs_generate_messages_cpp ipab_lwr_msgs_generate_messages_cpp)
pods_use_pkg_config_packages(lcm2ros_kuka lcm bot2-core)

add_executable(ros2lcm_kuka src/ros2lcm_kuka.cpp)
target_link_libraries(ros2lcm_kuka ${catkin_LIBRARIES})
add_dependencies(ros2lcm_kuka ipab_msgs_generate_messages_cpp ihmc_msgs_generate_messages_cpp ipab_lwr_msgs_generate_messages_cpp)
pods_use_pkg_config_packages(ros2lcm_kuka lcm bot2-core eigen3)

######################################################
add_executable(ros2lcm_exotica src/ros2lcm_exotica.cpp)
target_link_libraries(ros2lcm_exotica ${catkin_LIBRARIES})
add_dependencies(ros2lcm_exotica ipab_msgs_generate_messages_cpp ihmc_msgs_generate_messages_cpp)
pods_use_pkg_config_packages(ros2lcm_exotica lcm bot2-core eigen3)

add_executable(lcm2ros_exotica src/lcm2ros_exotica.cpp)
target_link_libraries(lcm2ros_exotica ${catkin_LIBRARIES})
add_dependencies(lcm2ros_exotica ipab_msgs_generate_messages_cpp ihmc_msgs_generate_messages_cpp)
pods_use_pkg_config_packages(lcm2ros_exotica lcm bot2-core eigen3 octomap)

######################################################
add_executable(ros2lcm_sdh src/ros2lcm_sdh.cpp)
target_link_libraries(ros2lcm_sdh ${catkin_LIBRARIES})
pods_use_pkg_config_packages(ros2lcm_sdh lcm bot2-core eigen3)

add_executable(lcm2ros_sdh src/lcm2ros_sdh.cpp)
target_link_libraries(lcm2ros_sdh ${catkin_LIBRARIES})
add_dependencies(lcm2ros_sdh ipab_msgs_generate_messages_cpp)
pods_use_pkg_config_packages(lcm2ros_sdh lcm bot2-core lcmtypes_robotiqhand std_srvs)


######################################################
add_executable(ros2lcm_husky src/ros2lcm_husky.cpp)
target_link_libraries(ros2lcm_husky ${catkin_LIBRARIES})
pods_use_pkg_config_packages(ros2lcm_husky lcm bot2-core eigen3)

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
