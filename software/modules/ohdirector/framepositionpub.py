from director import lcmUtils
from director.timercallback import TimerCallback
from director import visualization as vis
import bot_core

class FramePosPublisher(object):
    '''
    Publish the position of a given frame as "GAZE_GOAL" to the gaze-tracker
    '''

    def __init__(self, robotSystem, frame_name="leftPalm", rate_hz=1):
        self.robotModel = robotSystem.robotStateModel
        self.target_frame = frame_name

        self.timer = TimerCallback(targetFps=rate_hz)
        self.timer.callback = self.sendGazeGoal

    def setFrameName(self, name):
        self.target_frame = name

    def start(self):
        self.timer.start()

    def stop(self):
        self.timer.stop()

    def running(self):
        return self.timer.isActive()

    # callback for timer
    def sendGazeGoal(self):
        frame = self.robotModel.getLinkFrame(self.target_frame)
        vis.updateFrame(frame, self.target_frame+"GazeGoal")
        msg = bot_core.vector_3d_t()
        [msg.x, msg.y, msg.z] = frame.GetPosition()
        lcmUtils.publish("GAZE_GOAL", msg)
