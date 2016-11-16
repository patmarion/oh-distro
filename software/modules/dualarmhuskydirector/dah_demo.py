from dualarmhuskydirector import botFrameSync
from dualarmhuskydirector.perc2plan import MotionPlan, segmentScene

class demo(object):
    def __init__(self,ikPlanner, robotStateModel, planningUtils, teleopJointController, vis, view):

        # init motion plan
        self.motionplan = MotionPlan.ReachObjects(ikPlanner, robotStateModel, planningUtils, teleopJointController, vis)

        # init segmentation class
        self.segsc = segmentScene.segScene(view)

        
    # def setupBox(self,getBotFrame,affordancePanel,playbackRobotModel,vis):

    #     frameSync, boxFrame=botFrameSync.createBoxAffordanceAndSyncToBotFrame(getBotFrame,affordancePanel,playbackRobotModel,vis)
        
    #     return frameSync, boxFrame

    # def createGraspFrameTo

    def segmentTable(self, oDPC=None):

        # Generate a frame called 'object X newframe' X = 0,1 
        self.segsc.demoSegment(oDPC)

    def moveArm(self, arm, frameName, objInHand=False):
        
        if objInHand:
            self.motionplan.setGraspFrame()

        if arm == 'left':
            self.motionplan.reachLeft(frameName)
        elif arm == 'right':
            self.motionplan.reachRight(frameName)
        else:
            print 'Error: not recognised arm:', arm

    
    
