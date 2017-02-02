#!/usr/bin/env python
# Plan Execution Sync for the Dual Arm Husky

import sys
import time
import lcm

from drc.plan_status_t import plan_status_t


def timestamp_now(): return int(time.time() * 1e6)

global lc
lc = lcm.LCM()

global plan_status
plan_status = plan_status_t()


def on_move_base_status(channel, data):
    mb_plan_status = plan_status_t.decode(data)
    publish_plan_execution_status(mb_plan_status)


def on_left_ur5_status(channel, data):
    l_plan_status = plan_status_t.decode(data)
    publish_plan_execution_status(l_plan_status)


def on_right_ur5_status(channel, data):
    r_plan_status = plan_status_t.decode(data)
    publish_plan_execution_status(r_plan_status)


def publish_plan_execution_status(ps_in):
    global plan_status
    plan_status.utime = timestamp_now()
    plan_status.execution_status = ps_in.execution_status
    plan_status.plan_type = ps_in.plan_type
    plan_status.last_plan_msg_utime = ps_in.last_plan_msg_utime
    plan_status.last_plan_start_utime = ps_in.last_plan_start_utime
    lc.publish("PLAN_EXECUTION_STATUS", plan_status.encode())

####################################################################


def main():
    print "Started Dual Arm Husky Plan Execution Status Sync"

    sub1 = lc.subscribe("HUSKY_MOVE_BASE_PLAN_STATUS", on_move_base_status)
    sub2 = lc.subscribe("LEFT_UR5_PLAN_STATUS", on_left_ur5_status)
    sub3 = lc.subscribe("RIGHT_UR5_PLAN_STATUS", on_right_ur5_status)
    subscriptions = [sub1, sub2, sub3]

    try:
        while True:
            lc.handle()  # TODO publish at 50-100 Hz and count down?
    except KeyboardInterrupt:
        [lc.unsubscribe(x) for x in subscriptions]
        pass

if __name__ == "__main__":
    main()
