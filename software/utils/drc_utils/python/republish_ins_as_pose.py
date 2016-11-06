#!/usr/bin/python
import os,sys
import lcm
import time
from lcm import LCM
from math import *
import numpy  as np

drc_base_dir = os.getenv("DRC_BASE")
sys.path.append(drc_base_dir + "/software/build/lib/python2.7/site-packages")
sys.path.append(drc_base_dir + "/software/build/lib/python2.7/dist-packages")

from bot_core.pose_t import pose_t
from bot_core.ins_t import ins_t

def on_ms(channel, data):
  m = ins_t.decode(data)
  p = pose_t()
  p.utime = m.utime
  p.pos = [0,0,0]
  p.orientation = m.quat
  lc.publish("POSE_BODY_ALT", p.encode()) 

####################################################################
lc = lcm.LCM()
print "started"
sub1 = lc.subscribe("IMU_MICROSTRAIN", on_ms)
while True:
  lc.handle()
lc.unsubscribe(sub1)