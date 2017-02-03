#!/usr/bin/python
# Script to plot the error in yaw to signal scope
 

import os,sys
import lcm
import time
from lcm import LCM
from math import *
import numpy  as np
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
import math

from threading import Thread
import threading

home_dir =os.getenv("DRC_BASE")
#print home_dir
sys.path.append(home_dir + "/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/software/build/lib/python2.7/dist-packages")

from bot_core.pose_t import pose_t

########################################################################################
def timestamp_now (): return int (time.time () * 1000000)

def quat_to_euler(q) :
  roll_a = 2.0 * (q[0]*q[1] + q[2]*q[3]);
  roll_b = 1.0 - 2.0 * (q[1]*q[1] + q[2]*q[2]);
  roll = math.atan2 (roll_a, roll_b);

  pitch_sin = 2.0 * (q[0]*q[2] - q[3]*q[1]);
  pitch = math.asin (pitch_sin);

  yaw_a = 2.0 * (q[0]*q[3] + q[1]*q[2]);
  yaw_b = 1.0 - 2.0 * (q[2]*q[2] + q[3]*q[3]);  
  yaw = math.atan2 (yaw_a, yaw_b);
  return [roll,pitch,yaw]


# was (roll, pitch, yaw)
def euler_to_quat(rpy):
  roll =  rpy[0]
  pitch = rpy[1]
  yaw =   rpy[2]

  sy = math.sin(yaw*0.5);
  cy = math.cos(yaw*0.5);
  sp = math.sin(pitch*0.5);
  cp = math.cos(pitch*0.5);
  sr = math.sin(roll*0.5);
  cr = math.cos(roll*0.5);
  w = cr*cp*cy + sr*sp*sy;
  x = sr*cp*cy - cr*sp*sy;
  y = cr*sp*cy + sr*cp*sy;
  z = cr*cp*sy - sr*sp*cy;
  return np.array([w,x,y,z])

global pose_alt
pose_alt = pose_t()

def on_pose_alt(channel, data):
  global pose_alt
  pose_alt = pose_t.decode(data)

def on_pose(channel, data):
  global pose_alt
  m = pose_t.decode(data)

  p_rpy = quat_to_euler(m.orientation)
  b_rpy = quat_to_euler(pose_alt.orientation)

  e_rpy =  np.array(p_rpy) - np.array(b_rpy)
  #print p_rpy
  #print b_rpy
  #print e_rpy
  #print " "
  #print " "
  o = pose_t()
  o.orientation = euler_to_quat(e_rpy)
  o.accel = e_rpy*180.0/np.pi
  o.utime = m.utime

  lc.publish("POSE_BODY_DIFF", o.encode()) 

  

####################################################################
lc = lcm.LCM()
print "started"

sub1 = lc.subscribe("POSE_BODY", on_pose)
sub1 = lc.subscribe("POSE_BODY_ALT", on_pose_alt)

while True:
  ## Handle LCM if new messages have arrived.
  lc.handle()

lc.unsubscribe(sub1)



