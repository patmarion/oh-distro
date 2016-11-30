from director import lcmUtils
from director.utime import getUtime
import sys
import time

from bot_core import joint_state_t


class PTU:

    def setAngles(self, pan, tilt):
        js = joint_state_t()
        js.utime = getUtime()
        js.num_joints = 2
        js.joint_name = ["ptu_pan", "ptu_tilt"]
        js.joint_position = [pan, tilt]
        js.joint_velocity = [0.3, 0.3]
        js.joint_effort = [0, 0]
        lcmUtils.publish("PTU_COMMAND", js)
