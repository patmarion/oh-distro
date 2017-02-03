import numpy
import math
execfile(os.path.join(os.path.dirname(__file__), '../rotations.py'))


import numpy

def rpyFunctionPose(msg):
    return quat_to_euler(msg.orientation)
def rollFunctionPose(msg):
    '''roll'''
    return msg.utime, rpyFunctionPose(msg)[0]* 180.0/math.pi
def pitchFunctionPose(msg):
    '''pitch'''
    return msg.utime, rpyFunctionPose(msg)[1]* 180.0/math.pi
def yawFunctionPose(msg):
    '''yaw'''
    return msg.utime, rpyFunctionPose(msg)[2]* 180.0/math.pi

addPlot(timeWindow=5, yLimits=[-1, 1])
addSignal('POSE_BODY', msg.utime, msg.vel[0])
addSignal('POSE_BODY', msg.utime, msg.vel[1])
addSignal('POSE_BODY', msg.utime, msg.vel[2])

addPlot(timeWindow=5, yLimits=[-0.0001, 0.0001])
addSignal('ATLAS_IMU_BATCH', msg.utime, msg.raw_imu[0].delta_rotation[0])
addSignal('ATLAS_IMU_BATCH', msg.utime, msg.raw_imu[0].delta_rotation[1])
addSignal('ATLAS_IMU_BATCH', msg.utime, msg.raw_imu[0].delta_rotation[2])


addPlot(timeWindow=5, yLimits=[-0.1, 0.1])
addSignal('POSE_BODY', msg.utime, msg.rotation_rate[0])
addSignal('POSE_BODY', msg.utime, msg.rotation_rate[1])
addSignal('POSE_BODY', msg.utime, msg.rotation_rate[2])

addPlot(timeWindow=5, yLimits=[-1, 1])
#addSignalFunction('POSE_BODY', rollFunctionPose)
#addSignalFunction('POSE_BODY', pitchFunctionPose)
addSignalFunction('POSE_BODY', yawFunctionPose)
addSignalFunction('POSE_BDI', yawFunctionPose)

#addSignal('POSE_BODY', msg.utime, pitchFunctionPose)
#addSignal('POSE_BODY', msg.utime, yawFunctionPose)


#addPlot(timeWindow=5, yLimits=[-1500, 0])
#addSignal('FORCE_TORQUE', msg.utime, msg.sensors[0].force[2])
#addSignal('FORCE_TORQUE', msg.utime, msg.sensors[1].force[2])

#addPlot(timeWindow=5, yLimits=[-5, 5])
#addSignal('LEG_ODOMETRY_DELTA', msg.utime, msg.translation[0])
#addSignal('LEG_ODOMETRY_DELTA', msg.utime, msg.translation[1])
#addSignal('LEG_ODOMETRY_DELTA', msg.utime, msg.translation[2])
