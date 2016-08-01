#!/usr/bin/env python
# send a simple whole body trajectory

import rospy
import ihmc_msgs
from ihmc_msgs.msg import *
import time

if __name__ == '__main__':
    pub = rospy.Publisher("/ihmc_ros/valkyrie/control/whole_body_trajectory", WholeBodyTrajectoryRosMessage, queue_size = 1)
    rospy.init_node ("DemoWBTMover")
    msg = WholeBodyTrajectoryRosMessage()

    msg.left_hand_trajectory_message.unique_id = 0
    msg.left_hand_trajectory_message.execution_mode = 1 # 0 is override, 1 is queue
    msg.right_hand_trajectory_message.unique_id = 0
    msg.right_hand_trajectory_message.robot_side =1
    msg.right_hand_trajectory_message.execution_mode = 1 # 0 is override, 1 is queue

    msg_left = ArmTrajectoryRosMessage(joint_trajectory_messages = [OneDoFJointTrajectoryRosMessage(trajectory_points = [TrajectoryPoint1DRosMessage(time= 3.0, position = -0.3, velocity = 0)] )]*7, execution_mode = 0, unique_id = -1) #rospy.Time.now().secs)
    msg.left_arm_trajectory_message = msg_left
    msg.left_arm_trajectory_message.robot_side =0

    msg_right = ArmTrajectoryRosMessage(joint_trajectory_messages = [OneDoFJointTrajectoryRosMessage(trajectory_points = [TrajectoryPoint1DRosMessage(time= 3.0, position = 0.2, velocity = 0)] )]*7, execution_mode = 0, unique_id = -1) #rospy.Time.now().secs)
    msg.right_arm_trajectory_message = msg_right
    msg.right_arm_trajectory_message.unique_id = 0
    msg.right_arm_trajectory_message.robot_side =1

    msg.chest_trajectory_message.unique_id = 0
    msg.chest_trajectory_message.execution_mode = 1 # 0 is override, 1 is queue

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
    msg.pelvis_trajectory_message = msg_pelvis
    msg.pelvis_trajectory_message.unique_id = -1
    msg.pelvis_trajectory_message.execution_mode = 0 # 0 is override, 1 is queue

    msg.left_foot_trajectory_message.unique_id = 0
    msg.left_foot_trajectory_message.execution_mode = 1 # 0 is override, 1 is queue

    msg.right_foot_trajectory_message.unique_id = 0
    msg.right_foot_trajectory_message.robot_side =1
    msg.right_foot_trajectory_message.execution_mode = 1 # 0 is override, 1 is queue    





    msg.unique_id = 14

    pub.publish(msg)
    time.sleep(1)     
    #pub.publish(msg)
    #time.sleep(1)     