cmake_minimum_required(VERSION 2.8.3)
project(dual_arm_husky_description)

catkin_package()

install(
  DIRECTORY meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
