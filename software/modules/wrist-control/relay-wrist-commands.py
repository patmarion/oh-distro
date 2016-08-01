#!/usr/bin/env python
#
# Filters out the lower arm joints of the COMMITTED_ROBOT_PLAN and 
# passes them to the Forearm Controller

import time
import lcm
import bot_core as lcmbotcore
import drc as lcmdrc

def onRobotPlan(channel, data):
    msg = lcmdrc.robot_plan_t.decode(data)
    planLength = len(msg.plan)
    totalTime = msg.plan[-1].utime / 1e6
    print "total time of plan with " + str(planLength) + " waypoints: " + str(totalTime)

    plan = msg.plan[-1]
    leftWristRollIndex = plan.joint_name.index("leftWristRoll")
    leftWristPitchIndex = plan.joint_name.index("leftWristPitch")
    leftForearmYawIndex = plan.joint_name.index("leftForearmYaw")

    previousUtime = 0
    startTime = time.time()
    for plan in msg.plan:
        planOffset = (plan.utime-previousUtime) / 1e6
        previousUtime = plan.utime
        # print "utime: " + str(plan.utime) + " so will sleep for " + str(planOffset)
        time.sleep(planOffset)
        leftWristRollRad = plan.joint_position[leftWristRollIndex]
        leftWristPitchRad = plan.joint_position[leftWristPitchIndex]
        leftForearmYawRad = plan.joint_position[leftForearmYawIndex]
        print "Commanding forearm yaw to " + str(leftForearmYawRad)
        print "Commanding wrist roll to " + str(leftWristRollRad)
        print "Commanding wrist pitch to " + str(leftWristPitchRad)
        sendWristCommand(plan.utime, leftForearmYawRad, leftWristRollRad, leftWristPitchRad)

    endTime = time.time()
    print "Done with plan of " + str(totalTime) + "s in " + str(endTime-startTime) + "s"


def sendWristCommand(utime, forearmYaw, roll, pitch):
	msg = lcmbotcore.joint_angles_t()
	msg.utime = utime
	msg.joint_name = [ "leftForearmYaw", "leftWristRoll", "leftWristPitch" ]
	msg.num_joints = len(msg.joint_name)
	msg.joint_position = [ forearmYaw, roll, pitch ]

	theLcm.publish("DESIRED_FOREARM_ANGLES", msg.encode())


theLcm = lcm.LCM()
theLcm.subscribe("COMMITTED_ROBOT_PLAN", onRobotPlan)

while True:
	theLcm.handle()
