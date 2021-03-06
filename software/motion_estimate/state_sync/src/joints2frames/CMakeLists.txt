cmake_minimum_required(VERSION 2.8.12 FATAL_ERROR)
project(joints2frames)

set(CMAKE_CXX_FLAGS "--std=c++11 ${CMAKE_CXX_FLAGS}")

set(PCL_LIBRARIES  pcl_io-1.7)
set(REQUIRED_PACKAGES urdf orocos-kdl kdl-parser forward-kinematics)

add_executable(drc-joints2frames joints2frames.cpp  )
target_link_libraries(drc-joints2frames boost_system)
pods_use_pkg_config_packages(drc-joints2frames
    path-util model-client   bot2-param-client
        pronto_lcm        pronto_vis        pronto_math
	lcmtypes_drc_lcmtypes  lcmtypes_visualization
       ${PCL_LIBRARIES} ${REQUIRED_PACKAGES})
pods_install_executables(drc-joints2frames )


add_executable(drc-joints2frames-kuka joints2frames_kuka.cpp)
target_link_libraries(drc-joints2frames-kuka boost_system)
pods_use_pkg_config_packages(drc-joints2frames-kuka
    path-util
    model-client
    bot2-param-client
    pronto_lcm
    pronto_vis
    lcmtypes_bot2-core
    ${REQUIRED_PACKAGES})
pods_install_executables(drc-joints2frames-kuka )


pods_install_python_script(oh-joints2frames-dual-arm-husky joints2frames_dual_arm_husky.py)
