#!/usr/bin/python
import os,sys
import lcm
import time
from lcm import LCM
from math import *
import numpy  as np

home_dir =os.getenv("HOME")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")

from pronto.pose_t import pose_t
from pronto.ins_t import ins_t

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