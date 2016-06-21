#!/usr/bin/env python

import rospy
import ihmc_msgs
from ihmc_msgs.msg import *
import time

if __name__ == '__main__':
    pub = rospy.Publisher("/ihmc_ros/valkyrie/control/arm_trajectory", ArmTrajectoryRosMessage, queue_size = 1)
    rospy.init_node ("DemoArmMover")
    msg = ArmTrajectoryRosMessage(joint_trajectory_messages = [OneDoFJointTrajectoryRosMessage(trajectory_points = [TrajectoryPoint1DRosMessage(time= 0.0, position = 0, velocity = 0.5)] )]*7, execution_mode = 0, unique_id = 13) #rospy.Time.now().secs)
    pub.publish(msg)
    msg.robot_side = 0
    pub.publish(msg)
    time.sleep(1) 