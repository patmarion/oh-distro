#!/usr/bin/env python
import os
import math
import numpy
import pydrake

import lcm
import botpy
from bot_core.rigid_transform_t import rigid_transform_t
from bot_core.joint_state_t import joint_state_t

# Set up Drake Plant
base_dir = os.getenv("DRC_BASE")
urdf = base_dir + \
    "/software/models/dual_arm_husky_description/" + \
    "urdf/dual_arm_husky_original.urdf"
r = pydrake.rbtree.RigidBodyTree(urdf)

bodyNames = [r.getBodyOrFrameName(i) for i in xrange(len(r.bodies))]
bodyNameToId = {}
for i, name in enumerate(bodyNames):
    bodyNameToId[name] = i

positionNames = [r.getPositionName(i) for i in xrange(r.num_positions)]
positionNameToId = {}
for i, name in enumerate(positionNames):
    positionNameToId[name] = i

# Hard-coded config
base_offset = 0.14493
bumblebee2_to_xtion_translation = [-0.048, -0.03, 0]
bumblebee2_to_xtion_rpy = [0,0,0] #[86.0*numpy.pi/180.0, 0.0, 92.0*numpy.pi/180.0]


# Automatically retrieved
pan_joint = positionNameToId["husky_ptu_pan"]
tilt_joint = positionNameToId["husky_ptu_tilt"]

base_link = r.findLinkId("base_link")
bumblebee2_link = r.findLinkId("bumblebee2")


def quaternion_from_matrix(matrix):
    """Return quaternion from rotation matrix.
    Homogeneous Matrix to Quaternion
        Adapted from
        https://github.com/RobotLocomotion/director/blob/master/src/python/director/thirdparty/transformations.py
    """
    M = numpy.array(matrix, dtype=numpy.float64, copy=False)[:4, :4]
    q = numpy.empty((4, ))
    t = numpy.trace(M)
    if t > M[3, 3]:
        q[0] = t
        q[3] = M[1, 0] - M[0, 1]
        q[2] = M[0, 2] - M[2, 0]
        q[1] = M[2, 1] - M[1, 2]
    else:
        i, j, k = 1, 2, 3
        if M[1, 1] > M[0, 0]:
            i, j, k = 2, 3, 1
        if M[2, 2] > M[i, i]:
            i, j, k = 3, 1, 2
        t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3]
        q[i] = t
        q[j] = M[i, j] + M[j, i]
        q[k] = M[k, i] + M[i, k]
        q[3] = M[k, j] - M[j, k]
    q *= 0.5 / math.sqrt(t * M[3, 3])
    if q[0] < 0.0:
        numpy.negative(q, q)
    return q


def ptu_state_handler(channel, data):
    msg = joint_state_t.decode(data)
    utime = msg.utime
    [translation, rotation] = compute_rigid_transform(
        msg.joint_position[0], msg.joint_position[1])
    publish_rigid_transform(utime, translation, rotation)


def compute_rigid_transform(pan, tilt):
    q = r.getZeroConfiguration()
    v = r.getZeroConfiguration()
    q[2] = base_offset
    q[pan_joint] = pan
    q[tilt_joint] = tilt
    kinsol = r.doKinematics(q, v)
    homogeneous_transform = r.relativeTransform(
        kinsol, base_link, bumblebee2_link)
    # Homogeneous transform to translation and quaternion,
    # Cf.
    # http://wcms.inf.ed.ac.uk/ipab/rss/lecture-notes-2016-2017/1.RSS-Intro3DGeometry.pdf
    translation = homogeneous_transform[0:3:, 3]
    quaternion = quaternion_from_matrix(homogeneous_transform)
    return translation, quaternion


def publish_rigid_transform(utime, translation, quaternion):
    msg = rigid_transform_t()
    msg.utime = utime
    msg.trans = translation
    msg.quat = quaternion
    lc.publish("BODY_TO_BUMBLEBEE2", msg.encode())

    msg2 = rigid_transform_t()
    msg2.utime = utime
    msg2.trans = bumblebee2_to_xtion_translation
    msg2.quat = botpy.euler_to_quat(bumblebee2_to_xtion_rpy)
    lc.publish("BUMBLEBEE2_TO_XTION", msg2.encode())


# Subscribe to PTU State
lc = lcm.LCM()
subscription = lc.subscribe("PTU_STATE", ptu_state_handler)
try:
    while True:
        lc.handle()
except KeyboardInterrupt:
    pass
