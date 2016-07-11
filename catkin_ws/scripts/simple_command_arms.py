#!/usr/bin/env python

import rospy
import ihmc_msgs
from ihmc_msgs.msg import *
import time

if __name__ == '__main__':
    pub = rospy.Publisher("/ihmc_ros/valkyrie/control/arm_trajectory", ArmTrajectoryRosMessage, queue_size = 1)
    rospy.init_node ("DemoArmMover")
    msg = ArmTrajectoryRosMessage(joint_trajectory_messages = [OneDoFJointTrajectoryRosMessage(trajectory_points = [TrajectoryPoint1DRosMessage(time= 3.0, position = 0, velocity = 0)] )]*7, execution_mode = 0, unique_id = -1) 
    msg.robot_side = 0
    pub.publish(msg)
    pub.publish(msg)
    time.sleep(2)     

    msg_right = ArmTrajectoryRosMessage(joint_trajectory_messages = [OneDoFJointTrajectoryRosMessage(trajectory_points = [TrajectoryPoint1DRosMessage(time= 3.0, position = 0, velocity = 0)] )]*7, execution_mode = 0, unique_id = -1) 
    msg_right.robot_side = 1
    pub.publish(msg_right)
    pub.publish(msg_right)

    time.sleep(1) 