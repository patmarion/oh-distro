#!/usr/bin/env python
# State Sync for the Dual Arm Husky

import sys
import time
import lcm

from bot_core.joint_state_t import joint_state_t
from bot_core.pose_t import pose_t
from bot_core.robot_state_t import robot_state_t
from bot_core.six_axis_force_torque_t import six_axis_force_torque_t

###############################################################################
# Hard-coded parameters
base_link_offset = 0.14493
husky_joints = ["front_left_wheel",
                "front_right_wheel",
                "rear_left_wheel",
                "rear_right_wheel",
                "husky_ptu_pan",
                "husky_ptu_tilt",
                "l_ur5_arm_shoulder_pan_joint",
                "l_ur5_arm_shoulder_lift_joint",
                "l_ur5_arm_elbow_joint",
                "l_ur5_arm_wrist_1_joint",
                "l_ur5_arm_wrist_2_joint",
                "l_ur5_arm_wrist_3_joint",
                "r_ur5_arm_shoulder_pan_joint",
                "r_ur5_arm_shoulder_lift_joint",
                "r_ur5_arm_elbow_joint",
                "r_ur5_arm_wrist_1_joint",
                "r_ur5_arm_wrist_2_joint",
                "r_ur5_arm_wrist_3_joint",
                "l_palm_finger_1_joint",
                "l_finger_1_joint_1",
                "l_finger_1_joint_2",
                "l_finger_1_joint_3",
                "l_finger_1_joint_proximal_actuating_hinge",
                "l_finger_1_joint_paraproximal_actuating_hinge",
                "l_finger_1_joint_proximal_actuating_bar",
                "l_finger_1_joint_paraproximal_bar",
                "l_finger_1_joint_median_actuating_hinge",
                "l_finger_1_joint_median_actuating_hinge_median_bar",
                "l_finger_1_joint_paramedian_hinge",
                "l_finger_1_joint_paramedian_hinge_median_bar_underactuated",
                "l_finger_1_joint_paradistal_hinge",
                "l_palm_finger_2_joint",
                "l_finger_2_joint_1",
                "l_finger_2_joint_2",
                "l_finger_2_joint_3",
                "l_finger_2_joint_proximal_actuating_hinge",
                "l_finger_2_joint_paraproximal_actuating_hinge",
                "l_finger_2_joint_proximal_actuating_bar",
                "l_finger_2_joint_paraproximal_bar",
                "l_finger_2_joint_median_actuating_hinge",
                "l_finger_2_joint_median_actuating_hinge_median_bar",
                "l_finger_2_joint_paramedian_hinge",
                "l_finger_2_joint_paramedian_hinge_median_bar_underactuated",
                "l_finger_2_joint_paradistal_hinge",
                "l_palm_finger_middle_joint",
                "l_finger_middle_joint_1",
                "l_finger_middle_joint_2",
                "l_finger_middle_joint_3",
                "l_finger_middle_joint_proximal_actuating_hinge",
                "l_finger_middle_joint_paraproximal_actuating_hinge",
                "l_finger_middle_joint_proximal_actuating_bar",
                "l_finger_middle_joint_paraproximal_bar",
                "l_finger_middle_joint_median_actuating_hinge",
                "l_finger_middle_joint_median_actuating_hinge_median_bar",
                "l_finger_middle_joint_paramedian_hinge",
                "l_finger_middle_joint_paramedian_hinge_median_bar_underactuated",
                "l_finger_middle_joint_paradistal_hinge",
                "r_palm_finger_1_joint",
                "r_finger_1_joint_1",
                "r_finger_1_joint_2",
                "r_finger_1_joint_3",
                "r_finger_1_joint_proximal_actuating_hinge",
                "r_finger_1_joint_paraproximal_actuating_hinge",
                "r_finger_1_joint_proximal_actuating_bar",
                "r_finger_1_joint_paraproximal_bar",
                "r_finger_1_joint_median_actuating_hinge",
                "r_finger_1_joint_median_actuating_hinge_median_bar",
                "r_finger_1_joint_paramedian_hinge",
                "r_finger_1_joint_paramedian_hinge_median_bar_underactuated",
                "r_finger_1_joint_paradistal_hinge",
                "r_palm_finger_2_joint",
                "r_finger_2_joint_1",
                "r_finger_2_joint_2",
                "r_finger_2_joint_3",
                "r_finger_2_joint_proximal_actuating_hinge",
                "r_finger_2_joint_paraproximal_actuating_hinge",
                "r_finger_2_joint_proximal_actuating_bar",
                "r_finger_2_joint_paraproximal_bar",
                "r_finger_2_joint_median_actuating_hinge",
                "r_finger_2_joint_median_actuating_hinge_median_bar",
                "r_finger_2_joint_paramedian_hinge",
                "r_finger_2_joint_paramedian_hinge_median_bar_underactuated",
                "r_finger_2_joint_paradistal_hinge",
                "r_palm_finger_middle_joint",
                "r_finger_middle_joint_1",
                "r_finger_middle_joint_2",
                "r_finger_middle_joint_3",
                "r_finger_middle_joint_proximal_actuating_hinge",
                "r_finger_middle_joint_paraproximal_actuating_hinge",
                "r_finger_middle_joint_proximal_actuating_bar",
                "r_finger_middle_joint_paraproximal_bar",
                "r_finger_middle_joint_median_actuating_hinge",
                "r_finger_middle_joint_median_actuating_hinge_median_bar",
                "r_finger_middle_joint_paramedian_hinge",
                "r_finger_middle_joint_paramedian_hinge_median_bar_underactuated",
                "r_finger_middle_joint_paradistal_hinge"]

###############################################################################


def timestamp_now(): return int(time.time() * 1e6)

global robot_state
robot_state = robot_state_t()
robot_state.utime = 0
robot_state.joint_name = husky_joints
robot_state.num_joints = len(husky_joints)
robot_state.joint_position = [0.0] * robot_state.num_joints
robot_state.joint_velocity = [0.0] * robot_state.num_joints
robot_state.joint_effort = [0.0] * robot_state.num_joints
robot_state.pose.translation.x = 0
robot_state.pose.translation.y = 0
robot_state.pose.translation.z = 0
robot_state.pose.rotation.w = 1
robot_state.pose.rotation.x = 0
robot_state.pose.rotation.y = 0
robot_state.pose.rotation.z = 0
robot_state.twist.linear_velocity.x = 0.0
robot_state.twist.linear_velocity.y = 0.0
robot_state.twist.linear_velocity.z = 0.0
robot_state.twist.angular_velocity.x = 0.0
robot_state.twist.angular_velocity.y = 0.0
robot_state.twist.angular_velocity.z = 0.0
robot_state.force_torque.l_foot_force_z = 0.0
robot_state.force_torque.l_foot_torque_x = 0.0
robot_state.force_torque.l_foot_torque_y = 0.0
robot_state.force_torque.r_foot_force_z = 0.0
robot_state.force_torque.r_foot_torque_x = 0.0
robot_state.force_torque.r_foot_torque_y = 0.0
robot_state.force_torque.l_hand_force = [0.0] * 3
robot_state.force_torque.l_hand_torque = [0.0] * 3
robot_state.force_torque.r_hand_force = [0.0] * 3
robot_state.force_torque.r_hand_torque = [0.0] * 3

global lc
lc = lcm.LCM()


def update_internal_robot_state(joint_state_msg):
    global robot_state
    for i in range(joint_state_msg.num_joints):
        joint_state_msg.joint_name[i] = joint_state_msg.joint_name[i].replace("left_", "l_")
        joint_state_msg.joint_name[i] = joint_state_msg.joint_name[i].replace("right_", "r_")
        index = robot_state.joint_name.index(joint_state_msg.joint_name[i])
        robot_state.joint_position[index] = joint_state_msg.joint_position[i]
        robot_state.joint_velocity[index] = joint_state_msg.joint_velocity[i]
        robot_state.joint_effort[index] = joint_state_msg.joint_effort[i]


def on_joint_state_msg(channel, data):
    joint_state = joint_state_t.decode(data)
    update_internal_robot_state(joint_state)


def on_force_torque(channel, data):
    global robot_state
    force_torque = six_axis_force_torque_t.decode(data)

    if "LEFT_" in channel:
        robot_state.force_torque.l_hand_force = force_torque.force
        robot_state.force_torque.l_hand_torque = force_torque.moment
    elif "RIGHT_" in channel:
        robot_state.force_torque.r_hand_force = force_torque.force
        robot_state.force_torque.r_hand_torque = force_torque.moment


def on_pose_body(channel, data):
    pose = pose_t.decode(data)

    # Republish as POSE_BODY to fill in the bot_frames tree
    if channel != "POSE_BODY":
        # Add z offset - all individual body estimate signals miss the offset
        pose.pos = [pose.pos[0], pose.pos[1], pose.pos[2] + base_link_offset]
        lc.publish("POSE_BODY", pose.encode())

    global robot_state
    robot_state.utime = pose.utime
    robot_state.pose.translation.x = pose.pos[0]
    robot_state.pose.translation.y = pose.pos[1]
    robot_state.pose.translation.z = pose.pos[2]
    robot_state.pose.rotation.w = pose.orientation[0]
    robot_state.pose.rotation.x = pose.orientation[1]
    robot_state.pose.rotation.y = pose.orientation[2]
    robot_state.pose.rotation.z = pose.orientation[3]

    lc.publish("EST_ROBOT_STATE", robot_state.encode())


####################################################################

def main():
    print "Started Dual Arm Husky State Sync"

    sub_channel = "POSE_BODY"
    if len(sys.argv) > 1:
        sub_channel = sys.argv[1]
        print "Root pose source: ", sub_channel
        if sub_channel != "POSE_BODY":
            print "Will republish", sub_channel, "as POSE_BODY"
    else:
        print "Root pose source: ", sub_channel

    sub1 = lc.subscribe(sub_channel, on_pose_body)
    sub2 = lc.subscribe("WHEEL_STATE", on_joint_state_msg)
    sub3 = lc.subscribe("LEFT_UR5_STATE", on_joint_state_msg)
    sub4 = lc.subscribe("RIGHT_UR5_STATE", on_joint_state_msg)
    sub5 = lc.subscribe("PTU_STATE", on_joint_state_msg)
    sub6 = lc.subscribe("ROBOTIQ_LEFT_STATE", on_joint_state_msg)
    sub7 = lc.subscribe("ROBOTIQ_RIGHT_STATE", on_joint_state_msg)
    sub8 = lc.subscribe("LEFT_FORCE_TORQUE", on_force_torque)
    sub9 = lc.subscribe("RIGHT_FORCE_TORQUE", on_force_torque)

    while True:
        lc.handle()

if __name__ == "__main__":
    main()
