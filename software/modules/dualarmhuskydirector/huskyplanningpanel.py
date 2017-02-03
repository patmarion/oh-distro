import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools
import director.applogic as app
import director.objectmodel as om
from director.timercallback import TimerCallback
from director import robotstate
from director import visualization as vis
from director import transformUtils
from director import ikplanner
from director import footstepsdriver
from director import vtkAll as vtk
from director import drcargs
from director import affordanceurdf
from director.roboturdf import HandFactory
from director import lcmUtils
import os

from director.utime import getUtime

def addWidgetsToDict(widgets, d):

    for widget in widgets:
        if widget.objectName:
            d[str(widget.objectName)] = widget
        addWidgetsToDict(widget.children(), d)


class WidgetDict(object):

    def __init__(self, widgets):
        addWidgetsToDict(widgets, self.__dict__)
        
class HuskyPlanningPanel(object):
    
    def __init__(self, planningUtils, robotStateModel, robotStateJointController, teleopRobotModel, teleopJointController, ikPlanner, manipPlanner, affordanceManager, showPlanFunction, hidePlanFunction):
        self.planningUtils = planningUtils
        self.robotStateModel = robotStateModel
        self.robotStateJointController = robotStateJointController
        self.teleopRobotModel = teleopRobotModel
        self.teleopJointController = teleopJointController
        self.ikPlanner = ikPlanner
        self.manipPlanner = manipPlanner
        self.affordanceManager = affordanceManager
        self.showPlanFunction = showPlanFunction
        self.hidePlanFunction = hidePlanFunction
        
        loader = QtUiTools.QUiLoader()
        uifile = QtCore.QFile(os.path.join(os.path.dirname(__file__), 'ui/ddHuskyPlanningPanel.ui'))
        assert uifile.open(uifile.ReadOnly)

        self.widget = loader.load(uifile)
        self.widget.setWindowTitle('Husky Planning Panel')
        self.ui = WidgetDict(self.widget.children())
        
        self.ui.huskyPlanningButton.connect('clicked()', self.onHuskyPlanningMode)
        self.ui.epPlanButton.connect('clicked()', self.epPlanButtonClicked)
        self.ui.mpPlanButton.connect('clicked()', self.mpPlanButtonClicked)
        self.ui.larmCombo.connect('currentIndexChanged(const QString&)', self.onLeftHandChanged)
        self.ui.rarmCombo.connect('currentIndexChanged(const QString&)', self.onRightHandChanged)
        self.ui.baseCombo.connect('currentIndexChanged(const QString&)', self.onBaseChanged)

        self.ui.mpInteractiveCheck.setEnabled(False)
        
        self.constraintSet = None
        self.palmOffsetDistance = 0.0
        self.baseFrameHeightOffset = 0.17775
        self.ikPlanner.fixedBaseArm = False
        
        self.deactivate()
        
    def onHuskyPlanningMode(self):
        if self.ui.huskyPlanningButton.checked:
            self.activate()
        else:
            self.deactivate()
            
    def activate(self):
        self.ui.huskyPlanningButton.blockSignals(True)
        self.ui.huskyPlanningButton.checked = True
        self.ui.huskyPlanningButton.blockSignals(False)
        self.ui.EndPosePlanningPanel.setEnabled(True)
        self.ui.MotionPlanningPanel.setEnabled(True)
        if self.getLeftArmConstraint() == 'ee fixed':
            self.createHandGoalFrame('left')
        if self.getRightArmConstraint() == 'ee fixed':
            self.createHandGoalFrame('right')

    def deactivate(self):
        self.ui.huskyPlanningButton.blockSignals(True)
        self.ui.huskyPlanningButton.checked = False
        self.ui.huskyPlanningButton.blockSignals(False)
        self.ui.EndPosePlanningPanel.setEnabled(False)
        self.ui.MotionPlanningPanel.setEnabled(False)
        self.removePlanFolder()
        self.hideTeleopModel()
        self.hidePlanFunction()
        self.constraintSet = None
        
    def removeHandFrames(self, side):
        linkName = self.ikPlanner.getHandLink(side)
        frameName = '%s constraint frame' % linkName
        frame = om.findObjectByName(frameName)
        if frame:
            om.removeFromObjectModel(frame)
            
    def removeBaseFrame(self):
        linkName = self.ikPlanner.pelvisLink
        frameName = linkName + ' constraint frame'
        frame = om.findObjectByName(frameName)
        if frame:
            om.removeFromObjectModel(frame)
            
    def createHandGoalFrame(self, side):
        folder = self.getConstraintFrameFolder()
        startPose = self.planningUtils.getPlanningStartPose()
        linkName = self.ikPlanner.getHandLink(side)
        frameName = '%s constraint frame' % linkName
        graspToHand = self.ikPlanner.newPalmOffsetGraspToHandFrame(side, self.palmOffsetDistance)
        graspToWorld = self.ikPlanner.newGraspToWorldFrame(startPose, side, graspToHand)
        frame = vis.showFrame(graspToWorld, frameName, parent=folder, scale=0.2)
        frame.connectFrameModified(self.onGoalFrameModified)
    
    def createBaseGoalFrame(self):
        folder = self.getConstraintFrameFolder()
        startPose = self.planningUtils.getPlanningStartPose()
        linkName = self.ikPlanner.pelvisLink
        frameName = linkName + ' constraint frame'
        om.removeFromObjectModel(om.findObjectByName(frameName))
        baseLinkPose = self.ikPlanner.getLinkFrameAtPose(linkName, startPose)
        baseLinkPose.Translate([0, 0, -self.baseFrameHeightOffset])
        frame = vis.showFrame(baseLinkPose, frameName, parent=folder, scale=0.2)
        frame.connectFrameModified(self.onGoalFrameModified)
                
    def onGoalFrameModified(self, frame):
        if self.ui.fpInteractiveCheck.checked:
            self.updateIk()
    
    def getComboText(self, combo):
        return str(combo.currentText)
    
    def getBaseConstraint(self):
        return self.getComboText(self.ui.baseCombo)
    
    def getLeftArmConstraint(self):
        return self.getComboText(self.ui.larmCombo)
    
    def getRightArmConstraint(self):
        return self.getComboText(self.ui.rarmCombo)
    
    def onLeftHandChanged(self):
        if self.getLeftArmConstraint() == 'ee fixed':
            self.createHandGoalFrame('left')
        elif self.getLeftArmConstraint() == 'arm fixed':
            self.removeHandFrames('left') 
        self.updateIKConstraints()
        self.updateIk()
        
    def onRightHandChanged(self):
        if self.getRightArmConstraint() == 'ee fixed':
            self.createHandGoalFrame('right')
        elif self.getRightArmConstraint() == 'arm fixed':
            self.removeHandFrames('right') 
        self.updateIKConstraints()
        self.updateIk()

    def onBaseChanged(self):
        if self.getBaseConstraint() == 'fixed':
            self.removeBaseFrame()
        elif self.getBaseConstraint() == 'constrained':
            self.createBaseGoalFrame()
        self.updateIKConstraints()
        self.updateIk()
          
    def updateIKConstraints(self):
        ikPlanner = self.ikPlanner
        startPoseName = 'reach_start'
        startPose = self.planningUtils.getPlanningStartPose()
        ikPlanner.addPose(startPose, startPoseName)
        constraints = []
        for handModel in ikPlanner.handModels:
            side = handModel.side
            if (side == "left"):
                thisHandConstraint = self.getLeftArmConstraint()
            elif (side == "right"):
                thisHandConstraint = self.getRightArmConstraint()
                
            if thisHandConstraint == 'arm fixed':
                if (side == "left"):
                    constraints.append(ikPlanner.createLockedLeftArmPostureConstraint(startPoseName))
                elif (side == "right"):
                    constraints.append(ikPlanner.createLockedRightArmPostureConstraint(startPoseName))
                ikPlanner.setArmLocked(side,True)
            elif thisHandConstraint == 'ee fixed':
                linkName = ikPlanner.getHandLink(side)
                graspToHand = ikPlanner.newPalmOffsetGraspToHandFrame(side, self.palmOffsetDistance)
                graspToWorld = self.getGoalFrame(linkName)
            
                p, q = ikPlanner.createPositionOrientationGraspConstraints(side, graspToWorld, graspToHand)
                p.tspan = [1.0, 1.0]
                q.tspan = [1.0, 1.0]
                constraints.extend([p, q])
                ikPlanner.setArmLocked(side,False)
        
        if self.getBaseConstraint() == 'fixed':
            constraints.append(ikPlanner.createLockedBasePostureConstraint(startPoseName, lockLegs=False))
            ikPlanner.setBaseLocked(True)
        elif self.getBaseConstraint() == 'constrained':
            linkName = ikPlanner.pelvisLink
            baseToWorld = self.getGoalFrame(linkName)
            baseoffset = vtk.vtkTransform()
            baseoffset.Translate([0, 0, -self.baseFrameHeightOffset])
            p, q = ikPlanner.createPositionOrientationConstraint(linkName, baseToWorld, baseoffset)
            p.tspan = [1.0, 1.0]
            q.tspan = [1.0, 1.0]
            constraints.extend([p, q])
            ikPlanner.setBaseLocked(False)
        elif self.getBaseConstraint() == 'x only':
            YZRPY = ['base_y','base_z','base_roll','base_pitch','base_yaw']
            constraints.append(ikPlanner.createPostureConstraint(startPoseName, YZRPY))
        elif self.getBaseConstraint() == 'free':
            ZRP = ['base_z','base_roll','base_pitch']
            constraints.append(ikPlanner.createPostureConstraint(startPoseName, ZRP))
            
        self.constraintSet = ikplanner.ConstraintSet(self.ikPlanner, constraints, 'reach_end', startPoseName)

    def updateIk(self):
        if not self.constraintSet:
            self.updateIKConstraints()
        self.checkFinalPosePlanningMode()
        endPose, info = self.constraintSet.runIk()
        endPoseName = 'reach_end'
        self.ikPlanner.addPose(endPose, endPoseName)
        self.showPose(self.constraintSet.endPose)
        app.displaySnoptInfo(info)
        
    def epPlanButtonClicked(self):
        self.updateIk()
        
    def mpPlanButtonClicked(self):
        startPoseName = 'reach_start'
        startPose = self.planningUtils.getPlanningStartPose()
        self.checkReachingPlanningMode()
        self.ikPlanner.addPose(startPose, startPoseName)
        plan = self.constraintSet.runIkTraj()
        self.showPlan(plan)

    @staticmethod
    def removePlanFolder():
        om.removeFromObjectModel(om.findObjectByName('teleop plan'))
        
    @staticmethod
    def getConstraintFrameFolder():
        return om.getOrCreateContainer('constraint frames', parentObj=om.getOrCreateContainer('teleop plan', parentObj=om.findObjectByName('planning')))
    
    @staticmethod
    def getGoalFrame(linkName):
        return om.findObjectByName('%s constraint frame' % linkName)
    
    def showTeleoModel(self):
        self.teleopRobotModel.setProperty('Visible', True)
        self.robotStateModel.setProperty('Visible', True)
        self.robotStateModel.setProperty('Alpha', 0.1)
        
    def hideTeleopModel(self):
        self.teleopRobotModel.setProperty('Visible', False)
        self.robotStateModel.setProperty('Visible', True)
        self.robotStateModel.setProperty('Alpha', 1.0)
        
    def showPose(self, pose):
        self.teleopJointController.setPose('EndPose', pose)
        self.hidePlanFunction()
        self.showTeleoModel()
        
    def showPlan(self, plan):
        self.hideTeleopModel()
        self.showPlanFunction(plan)
        
    def checkFinalPosePlanningMode(self):
        self.ikPlanner.planningMode = self.getComboText(self.ui.fpModeCombo)
        
    def checkReachingPlanningMode(self):
        self.ikPlanner.planningMode = self.getComboText(self.ui.mpModeCombo)
        
def _getAction():

    actionName = 'ActionHuskyPlanningPanel'
    action = app.getToolBarActions().get(actionName)

    if action is None:
        icon = QtGui.QIcon(os.path.join(os.path.dirname(__file__), 'images/ur5.png'))
        assert os.path.isfile(os.path.join(os.path.dirname(__file__), 'images/ur5.png'))
        action = QtGui.QAction(icon, 'Husky Planning Panel', None)
        action.objectName = actionName
        action.checkable = True

        mainWindow = app.getMainWindow()
        toolbar = mainWindow.panelToolBar()

        toolbar.insertAction(toolbar.actions()[0], action)

    return action

def init(planningUtils, robotStateModel, robotStateJointController, teleopRobotModel, teleopJointController, ikPlanner, manipPlanner, affordanceManager, showPlanFunction, hidePlanFunction):

    global panel
    global dock

    panel = HuskyPlanningPanel(planningUtils, robotStateModel, robotStateJointController, teleopRobotModel, teleopJointController, ikPlanner, manipPlanner, affordanceManager, showPlanFunction, hidePlanFunction)
    dock = app.addWidgetToDock(panel.widget, action=_getAction())
    dock.hide()

    return panel