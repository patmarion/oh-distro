from dualarmhuskydirector import botFrameSync
from dualarmhuskydirector.perc2plan import MotionPlan, segmentScene

class demo(object):
    def __init__(self,ikPlanner, robotStateModel, planningUtils, teleopJointController, vis, view):

        # init motion plan
        self.motionplan = MotionPlan.ReachObjects(ikPlanner, robotStateModel, planningUtils, teleopJointController, vis)

        # init segmentation class
        self.segsc = segmentScene.segScene(view)

        # tolerance limits
        self.flexangTol = 60
        self.stricktangTol = 5

        # configuration list
        self.qs = []

    def setupBox(self,getBotFrame,affordancePanel,vis):
        """
        """
        # Create the box affordance and sync it to bot frame RUBBISH_BIN
        # Assumes RUBBISH_BIN exists.
        frameSync, boxFrame = botFrameSync.createBoxAndSync(getBotFrame,affordancePanel,vis)
         
        return frameSync, boxFrame

    def createDropFrame(self, boxFrame, vis):
        """
        """
        # Check 
        

        dropFrameTrans = botFrameSync.createDropFrameAboveBox(boxFrame,vis) # uses default name

        return dropFrameTrans

    def segmentTable(self, oDPC=None):

        # Generate a frame called 'object X newframe' X = 0,1 
        self.segsc.demoSegment(oDPC)

    def moveArm(self, arm, frameName, objInHand=False, flex = False):
        
        # Syncs current state within class with actual robot state
        self.motionplan.synchRobot()

        # Selects between flexible tolerance in the orientation
        if flex:
            aT = self.flexangTol
        else:
            aT = self.stricktangTol


        # Selects between grasp or pre-grasp frame
        if objInHand:
            self.motionplan.setGraspFrame()
        else:
            self.motionplan.setPreGraspFrame()

        # Select arm and request motion plan
        if arm == 'left':
            self.motionplan.reachLeft(frameName, aT)
        elif arm == 'right':
            self.motionplan.reachRight(frameName, aT)
        else:
            print 'Error: not recognised arm:', arm


    def store_q(self):

        # append in a list the current configuration
        self.qs.append(self.motionplan.pU.getPlanningStartPose())

    

    
