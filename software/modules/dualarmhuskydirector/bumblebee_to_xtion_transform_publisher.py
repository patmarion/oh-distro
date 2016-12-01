#!/usr/bin/python
import numpy as np
import sys
import lcm
import botpy
from bot_core.rigid_transform_t import rigid_transform_t

lc = lcm.LCM()
msg = rigid_transform_t();
msg.utime = 0;
msg.trans = [-0.048, -0.03, -0.01]
msg.quat = botpy.euler_to_quat([float(sys.argv[1])*np.pi/180.0, float(sys.argv[2])*np.pi/180.0, float(sys.argv[3])*np.pi/180.0])

lc.publish("BB_TO_OPENNI_FRAME_LEFT", msg.encode())
