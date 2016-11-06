cmake_minimum_required(VERSION 2.8.3)
project(robotiq_force_torque_sensor_msgs)

# Minimal package only containing the message

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY
    msg
  FILES
    ft_sensor.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
