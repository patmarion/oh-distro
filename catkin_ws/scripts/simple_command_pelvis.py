#!/usr/bin/env python

import rospy
import ihmc_msgs
from ihmc_msgs.msg import *
import time

if __name__ == '__main__':
    pub = rospy.Publisher("/ihmc_ros/valkyrie/control/pelvis_trajectory", PelvisTrajectoryRosMessage, queue_size = 1)
    rospy.init_node ("DemoPelvisMover")

    msg_pelvis = PelvisTrajectoryRosMessage()
    pt = SE3TrajectoryPointRosMessage()
    pt.time = 3.0
    pt.position.x = -0.06
    pt.position.y = -0.06
    pt.position.z = 1.0
    pt.orientation.w = 1.0
    pt.orientation.x = 0
    pt.orientation.y = 0
    pt.orientation.z = 0

    msg_pelvis.taskspace_trajectory_points = [pt]
    msg_pelvis.unique_id = -1
    msg_pelvis.execution_mode = 0 # 0 is override, 1 is queue    


    pub.publish(msg_pelvis)
    pub.publish(msg_pelvis)

    time.sleep(1) 