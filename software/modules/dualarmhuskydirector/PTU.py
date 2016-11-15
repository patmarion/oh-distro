import rospy
import sys
import time

from sensor_msgs.msg import JointState

class PTU:
    def __init__(self):
        self.pub = rospy.Publisher("cmd", JointState, queue_size=1)

    def setAngles(self, pan, tilt):
        js = JointState()
        # TODO timestamp
        js.name = [ "ptu_pan", "ptu_tilt" ]
        js.velocity = [ 0.3, 0.3 ]
        js.position = [ pan, tilt ]
        self.pub.publish(js)
