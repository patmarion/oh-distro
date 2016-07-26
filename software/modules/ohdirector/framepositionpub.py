from director import lcmUtils
from director.timercallback import TimerCallback
from director import visualization as vis
import bot_core

from director.tasks.taskuserpanel import TaskUserPanel

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


# GUI
class FrameGazePanel(TaskUserPanel):
    def __init__(self, frame_pos_publisher):
        TaskUserPanel.__init__(self, windowTitle='Frame Gaze')
        self.fg = frame_pos_publisher

        self.running_label = "Running"
        self.frame_label = "Frame name"

        self.addButtons()
        self.addProperties()

    def start(self):
        self.fg.start()
        self.params.setProperty(self.running_label, True)

    def stop(self):
        self.fg.stop()
        self.params.setProperty(self.running_label, False)

    def addButtons(self):
        self.addManualButton('Start Frame Pub', self.start)
        self.addManualButton('Stop Frame Pub', self.stop)

    def addProperties(self):
        self.params.addProperty(self.running_label, False)
        self.params.addProperty(self.frame_label, "leftPalm")

    def onPropertyChanged(self, propertySet, propertyName):
        if propertyName == self.running_label:
            running = self.params.getProperty(self.running_label)
            if running:
                self.appendMessage("STARTING frame position publisher")
                self.fg.start()
            else:
                self.appendMessage("STOPPING frame position publisher")
                self.fg.stop()
        if propertyName == self.frame_label:
            frame_name = self.params.getProperty(self.frame_label)
            self.appendMessage("Changing frame to: %s" % frame_name)
            self.fg.setFrameName(frame_name)
