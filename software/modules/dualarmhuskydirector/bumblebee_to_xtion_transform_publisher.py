#!/usr/bin/python
import numpy as np
import sys
import lcm
import botpy
from bot_core.rigid_transform_t import rigid_transform_t

lc = lcm.LCM()
msg = rigid_transform_t();
msg.utime = 0;
msg.trans = [float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])]
msg.quat = botpy.euler_to_quat([90*np.pi/180.0, 0, 90*np.pi/180.0])

lc.publish("BUMBLEBEE2_TO_XTION", msg.encode())
