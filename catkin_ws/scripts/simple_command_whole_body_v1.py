#!/usr/bin/env python
# just sends a left arm and right fakes rest

import rospy
import ihmc_msgs
from ihmc_msgs.msg import *
import time

if __name__ == '__main__':
    #pub = rospy.Publisher("/ihmc_ros/valkyrie/control/arm_trajectory", ArmTrajectoryRosMessage, queue_size = 1)
    #rospy.init_node ("DemoArmMover")
    msg_right = ArmTrajectoryRosMessage(joint_trajectory_messages = [OneDoFJointTrajectoryRosMessage(trajectory_points = [TrajectoryPoint1DRosMessage(time= 0.0, position = 0, velocity = 0.5)] )]*7, execution_mode = 0, unique_id = 130) #rospy.Time.now().secs)
    msg_left = ArmTrajectoryRosMessage(joint_trajectory_messages = [OneDoFJointTrajectoryRosMessage(trajectory_points = [TrajectoryPoint1DRosMessage(time= 0.0, position = 0, velocity = 0.5)] )]*7, execution_mode = 0, unique_id = 13) #rospy.Time.now().secs)
    #pub.publish(msg)
    #msg.robot_side = 0
    #pub.publish(msg)
    #time.sleep(1) 

    pub = rospy.Publisher("/ihmc_ros/valkyrie/control/whole_body_trajectory", WholeBodyTrajectoryRosMessage, queue_size = 1)
    rospy.init_node ("DemoWBTMover")
    msg = WholeBodyTrajectoryRosMessage() #rospy.Time.now().secs)


    msg.left_hand_trajectory_message.unique_id = 0
    msg.left_hand_trajectory_message.execution_mode = 1 # 0 is override, 1 is queue
    msg.right_hand_trajectory_message.unique_id = 0
    msg.right_hand_trajectory_message.robot_side =1
    msg.right_hand_trajectory_message.execution_mode = 1 # 0 is override, 1 is queue


    msg.left_arm_trajectory_message = msg_left
    msg.left_arm_trajectory_message.robot_side =0

    msg.right_arm_trajectory_message = msg_right
    msg.right_arm_trajectory_message.unique_id = 0
    msg.right_arm_trajectory_message.robot_side =1
    #msg.right_arm_trajectory_message.execution_mode = 1 # 0 is override, 1 is queue

    msg.chest_trajectory_message.unique_id = 0
    msg.chest_trajectory_message.execution_mode = 1 # 0 is override, 1 is queue

    msg.pelvis_trajectory_message.unique_id = 0
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