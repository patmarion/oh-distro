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
    
        self.ui.larmCombo.setEnabled(False)
        self.ui.rarmCombo.setEnabled(False)
        self.ui.baseCombo.setEnabled(False)
        self.ui.fpModeCombo.setEnabled(False)
        self.ui.mpInteractiveCheck.setEnabled(False)
        
        self.constraintSet = None
        self.palmOffsetDistance = 0.0
        
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
    
    def createHandGoalFrame(self, side):
        folder = self.getConstraintFrameFolder()
        startPose = self.planningUtils.getPlanningStartPose()
        linkName = self.ikPlanner.getHandLink(side)
        frameName = '%s constraint frame' % linkName
        graspToHand = self.ikPlanner.newPalmOffsetGraspToHandFrame(side, self.palmOffsetDistance)
        graspToWorld = self.ikPlanner.newGraspToWorldFrame(startPose, side, graspToHand)
        frame = vis.showFrame(graspToWorld, frameName, parent=folder, scale=0.2)
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
            
            linkName = ikPlanner.getHandLink(side)
            graspToHand = ikPlanner.newPalmOffsetGraspToHandFrame(side, self.palmOffsetDistance)
            graspToWorld = self.getGoalFrame(linkName)
            
            p, q = ikPlanner.createPositionOrientationGraspConstraints(side, graspToWorld, graspToHand)
            p.tspan = [1.0, 1.0]
            q.tspan = [1.0, 1.0]
            
            if thisHandConstraint == 'arm fixed':
                if (side == "left"):
                    constraints.append(ikPlanner.createLockedLeftArmPostureConstraint(startPoseName))
                elif (side == "right"):
                    constraints.append(ikPlanner.createLockedRightArmPostureConstraint(startPoseName))
                ikPlanner.setArmLocked(side,True)
            elif thisHandConstraint == 'ee fixed':
                constraints.extend([p, q])
                ikPlanner.setArmLocked(side,False)
            elif thisHandConstraint == 'free':
                ikPlanner.setArmLocked(side,False)
        
        # TODO: base constraints
        constraints.append(ikPlanner.createLockedBasePostureConstraint(startPoseName, lockLegs=False))
        ikPlanner.setBaseLocked(True)
        
        self.constraintSet = ikplanner.ConstraintSet(self.ikPlanner, constraints, 'reach_end', startPoseName)
        print 'update ik constraint'
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