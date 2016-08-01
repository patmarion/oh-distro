#!/usr/bin/python

import os,sys
import lcm
import time
from lcm import LCM
import math
import numpy  as np
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab

from threading import Thread
import threading

home_dir =os.getenv("HOME")
#print home_dir
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")

from drc.robot_state_t import robot_state_t
########################################################################################
def timestamp_now (): return int (time.time () * 1000000)


global scan_period
global counter
counter =0
scan_period = 8
def on_scan(channel, data):
  global counter, scan_period
  counter=counter+1
  if (counter%scan_period ==0):
    print "republish"
    lc.publish("SICK_SCAN_LOWER_FREQ",data)

####################################################################
lc = lcm.LCM()
print "started"

sub1 = lc.subscribe("SICK_SCAN", on_scan)

while True:
  ## Handle LCM if new messages have arrived.
  lc.handle()

lc.unsubscribe(sub)



