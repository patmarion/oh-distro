set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations -Wreturn-type -Wuninitialized -Wunused-variable")

cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME motion_estimate)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

add_subdirectory(src/joints2frames)
#add_subdirectory(src/state_sync)
add_subdirectory(src/state_sync_nasa)
add_subdirectory(src/state_sync_kuka)
add_subdirectory(src/state_sync_dual_arm_husky)

add_subdirectory(src/artificial_drift)

add_subdirectory(src/frames_changer)

add_subdirectory(src/test_tools)
add_subdirectory(src/accel_from_position)
