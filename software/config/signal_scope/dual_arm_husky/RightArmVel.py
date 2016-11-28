#!/usr/bin/env

# Date : 10.11.2016
# Author : Theo @SLMC

# Info:  
# This script is utilising signal-scope to plot the right 
# arm joint velocities (rad/s) of the Dual arm husky robot
# The velocities are read from the lcm channel 'RIGHT_UR5_STATE'
# and are the measured velocities of the joints


# The velocities are read from the lcm channel 'LEFT_UR5_EXECUTE'
# and are the commanded velocities of the joints

channel = 'RIGHT_UR5_EXECUTE'

addPlot(timeWindow=15, yLimits=[-6, 6])
addSignal(channel, msg.utime, msg.joint_velocity[0])
addSignal(channel, msg.utime, msg.joint_velocity[1])
addSignal(channel, msg.utime, msg.joint_velocity[2])
addSignal(channel, msg.utime, msg.joint_velocity[3])
addSignal(channel, msg.utime, msg.joint_velocity[4])
addSignal(channel, msg.utime, msg.joint_velocity[5])




channel = 'RIGHT_UR5_STATE'

addPlot(timeWindow=15, yLimits=[-6, 6])
addSignal(channel, msg.utime, msg.joint_velocity[0])
addSignal(channel, msg.utime, msg.joint_velocity[1])
addSignal(channel, msg.utime, msg.joint_velocity[2])
addSignal(channel, msg.utime, msg.joint_velocity[3])
addSignal(channel, msg.utime, msg.joint_velocity[4])
addSignal(channel, msg.utime, msg.joint_velocity[5])
