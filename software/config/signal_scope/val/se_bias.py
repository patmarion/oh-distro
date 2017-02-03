import numpy
import math
execfile(os.path.join(os.path.dirname(__file__), 'rotations.py'))

addPlot(timeWindow=5, yLimits=[-1, 1])
# Rotation bias around x, y, z. Z is what we are most interested - yaw bias
addSignal('STATE_ESTIMATOR_STATE', msg.utime, msg.state[15])
addSignal('STATE_ESTIMATOR_STATE', msg.utime, msg.state[16])
addSignal('STATE_ESTIMATOR_STATE', msg.utime, msg.state[17])


addPlot(timeWindow=5, yLimits=[-1, 1])
addSignal('STATE_ESTIMATOR_STATE', msg.utime, msg.state[18])
addSignal('STATE_ESTIMATOR_STATE', msg.utime, msg.state[19])
addSignal('STATE_ESTIMATOR_STATE', msg.utime, msg.state[20])

addPlot(timeWindow=5, yLimits=[-1, 1])
addSignal('STATE_ESTIMATOR_STATE', msg.utime, msg.cov[132])
addSignal('STATE_ESTIMATOR_STATE', msg.utime, msg.cov[154])
addSignal('STATE_ESTIMATOR_STATE', msg.utime, msg.cov[176])

#    angular_velocity_ind = 0,
#    velocity_ind = 3,
#    chi_ind = 6,
#    position_ind = 9,
#    acceleration_ind = 12,
#    rotation rate bias = 15
#    accel bias = 18

#addPlot(timeWindow=5, yLimits=[0, 10])
#addSignal('ROBOT_BEHAVIOR', msg.utime, msg.behavior)
