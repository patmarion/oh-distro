#!/usr/bin/python
import numpy as np
import botpy
import lcm
from bot_core.pose_t import pose_t

lc = lcm.LCM()
print "Send POSE_BODY..."
msg = pose_t();
msg.utime = 0;
msg.pos = [0,0,1.2]
msg.orientation = botpy.euler_to_quat([0, 17.0*np.pi/180.0,0])

lc.publish("POSE_BODY", msg.encode())