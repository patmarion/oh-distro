cmake_minimum_required(VERSION 2.8.3)
project(robotiq_hand_description)

catkin_package()

install(
  DIRECTORY meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
