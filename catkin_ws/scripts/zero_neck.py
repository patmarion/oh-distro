#!/usr/bin/env python

import rospy
import ihmc_msgs
from ihmc_msgs.msg import *
import time

if __name__ == '__main__':
    pub = rospy.Publisher("/ihmc_ros/valkyrie/control/neck_trajectory", NeckTrajectoryRosMessage, queue_size = 1)
    rospy.init_node ("DemoNeckMover")
    msg = NeckTrajectoryRosMessage(joint_trajectory_messages = [OneDoFJointTrajectoryRosMessage(trajectory_points = [TrajectoryPoint1DRosMessage(time= 0.0, position = 2, velocity = 0.5)] )]*3, unique_id = 13) #rospy.Time.now().secs)

    pub.publish(msg)
    pub.publish(msg)
    time.sleep(1)

    if (1 == 0):
	    time_goal = 1.0
	    tp0 = TrajectoryPoint1DRosMessage(time= time_goal, position = 0.2, velocity = 0.5) 
	    tp1 = TrajectoryPoint1DRosMessage(time= time_goal, position = 0, velocity = 0.5) 
	    tp2 = TrajectoryPoint1DRosMessage(time= time_goal, position = -0.2, velocity = 0.5)

	    trajectory_points = [tp0, tp1, tp2]

	    m = NeckTrajectoryRosMessage()
	    m2 = OneDoFJointTrajectoryRosMessage()
	    m2.trajectory_points = trajectory_points
	    m2.unique_id = 12
	    m.joint_trajectory_messages = [m2]

	    m.unique_id = 13
	    print m

	    pub.publish(m)
	    pub.publish(m)
	    time.sleep(1)     