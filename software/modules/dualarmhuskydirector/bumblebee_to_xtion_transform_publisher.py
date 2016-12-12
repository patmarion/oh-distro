#!/usr/bin/python
import numpy as np
import sys
import lcm
import botpy
from bot_core.rigid_transform_t import rigid_transform_t

lc = lcm.LCM()
msg = rigid_transform_t();
msg.utime = 0;
# msg.trans = [-0.01, -1.225, -.225]
# msg.quat = botpy.euler_to_quat([float(sys.argv[1])*np.pi/180.0, float(sys.argv[2])*np.pi/180.0, float(sys.argv[3])*np.pi/180.0])

x = float(sys.argv[1]) #-0.01
y = float(sys.argv[2]) #-1.225
z = float(sys.argv[3]) #-.225
r = float(sys.argv[4]) #-35 #float(sys.argv[1])
p = float(sys.argv[5]) #0 #float(sys.argv[2])
yaw = float(sys.argv[6]) #0#float(sys.argv[3])

msg.trans = [x, y, z]
msg.quat = botpy.euler_to_quat([r*np.pi/180.0, p*np.pi/180.0, yaw*np.pi/180.0])


lc.publish("BB_TO_OPENNI_FRAME_LEFT", msg.encode())
