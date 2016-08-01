import os
import copy
import math
import functools
import numpy as np

import drcargs

from director import transformUtils

from director.asynctaskqueue import AsyncTaskQueue
from director import objectmodel as om
from director import visualization as vis
from director import robotstate
from director import segmentation
from director import planplayback
from director.pointpicker import PointPicker
from director import vtkAll as vtk
from director.simpletimer import SimpleTimer
from director import affordanceupdater
from director import sceneloader

from director.debugVis import DebugData
from director import affordanceitems
from director import ikplanner
from director import vtkNumpy
from numpy import array
from director.uuidutil import newUUID
from director import lcmUtils
import ioUtils
import drc as lcmdrc

from director.tasks.taskuserpanel import TaskUserPanel
from director.tasks.taskuserpanel import ImageBasedAffordanceFit

import director.tasks.robottasks as rt

from director.roboturdf import HandFactory
from numpy import linalg as npla
from director import ik
import director.applogic as app


class TableSimpleDemo(object):

    def __init__(self, robotStateModel, playbackRobotModel, ikPlanner, manipPlanner, footstepPlanner,
                 view, sensorJointController, teleopRobotModel, teleopJointController, footstepsDriver, valkyrieDriver):
        self.robotStateModel = robotStateModel
        self.playbackRobotModel = playbackRobotModel
        self.ikPlanner = ikPlanner
        self.manipPlanner = manipPlanner
        self.footstepPlanner = footstepPlanner
        self.valkyrieDriver = valkyrieDriver
        self.sensorJointController = sensorJointController
        self.view = view
        self.affordanceManager = segmentation.affordanceManager

        # live operation flags:
        self.useFootstepPlanner = True
        self.visOnly = False
        self.planFromCurrentRobotState = True
        extraModels = [self.playbackRobotModel]
        self.affordanceUpdater  = affordanceupdater.AffordanceGraspUpdater(self.robotStateModel, self.ikPlanner, extraModels)

        self.affordanceManager.setAffordanceUpdater(self.affordanceUpdater)
        self.optionalUserPromptEnabled = True
        self.requiredUserPromptEnabled = True

        self.plans = []
        self.clusterObjects = []
        self.frameSyncs = {}

        self.tableData = None

        # top level switch between BDI or IHMC (locked base) and MIT (moving base and back)
        self.lockBack = True
        self.lockBase = True
        self.planner = None

        self.constraintSet = []

        self.picker = None
        
        self.teleopRobotModel = teleopRobotModel
        self.teleopJointController = teleopJointController
        self.footstepsDriver = footstepsDriver
        self.feetConstraint = 'Fixed'
        self.reachingHand = 'left'
        
        self.grasp_plan = None
    def planPlaybackFunction(plans):
        planPlayback.stopAnimation()
        playbackRobotModel.setProperty('Visible', True)
        planPlayback.playPlans(plans, playbackJointController)

    def addPlan(self, plan):
        self.plans.append(plan)


    ### Table and Bin Focused Functions
    def userFitTable(self):
        self.tableData = None
        self.picker = PointPicker(self.view, numberOfPoints=1, drawLines=False, callback=self.onSegmentTable)
        self.picker.start()
    
    def userReFitTable(self):
        om.removeFromObjectModel(om.findObjectByName('segmentation'))
        om.removeFromObjectModel(om.findObjectByName('cont debug'))
        om.removeFromObjectModel(om.findObjectByName('affordances'))
        self.userFitTable()
        
    def waitForTableFit(self):
        while not self.tableData:
            yield


    def getInputPointCloud(self):
        polyData = segmentation.getCurrentRevolutionData()
        if polyData is None:
            obj = om.findObjectByName('scene')
            if obj:
                polyData = obj.polyData

        return polyData

    def onSegmentTable(self, p1):
        print p1
        if self.picker is not None:
            self.picker.stop()
            om.removeFromObjectModel(self.picker.annotationObj)
            self.picker = None

        data = segmentation.segmentTableScene(self.getInputPointCloud(), p1)
        vis.showClusterObjects(data.clusters, parent='segmentation')
        #segmentation.showTable(data.table, parent='segmentation')

        self.data = data

        tableData = data.table

        #tableData, _ = segmentation.segmentTableAndFrame(self.getInputPointCloud(), p1)

        pose = transformUtils.poseFromTransform(tableData.frame)
        desc = dict(classname='MeshAffordanceItem', Name='table', Color=[0,1,0], pose=pose)
        aff = self.affordanceManager.newAffordanceFromDescription(desc)
        aff.setPolyData(tableData.mesh)

        self.tableData = tableData

        tableBox = vis.showPolyData(tableData.box, 'table box', parent=aff, color=[0,1,0], visible=False)
        tableBox.actor.SetUserTransform(tableData.frame)

        # how far back to stand - from the middle of the table
        # -0.6 is too far. reduced 0.5 was too low. now trying -0.55
        relativeStance = transformUtils.frameFromPositionAndRPY([-0.55, 0, 0],[0,0,0])
        self.computeTableStanceFrame(relativeStance)

        # automatically add the box on the table (skip user segmentation)
        #boxFrame = transformUtils.copyFrame(tableData.frame)
        #boxFrame.PreMultiply()
        #tableToBoxFrame = transformUtils.frameFromPositionAndRPY([-0.05, 0, 0.15], [0,0, 0])
        #boxFrame.Concatenate(tableToBoxFrame)
        #self.spawnBlockAffordanceAtFrame(boxFrame)

        safeStance = transformUtils.frameFromPositionAndRPY([-1, 0, 0],[0,0,0])
        self.computeTableStanceFrame(safeStance, 'safe stance frame')


    def computeTableStanceFrame(self, relativeStance, relativeStanceFrameName='table stance frame'):
        tableTransform = om.findObjectByName('table').getChildFrame().transform
        zGround = 0.0
        tableHeight = tableTransform.GetPosition()[2] - zGround

        t = transformUtils.copyFrame(tableTransform)
        t.PreMultiply()
        relativePosition = [relativeStance.GetPosition()[0], relativeStance.GetPosition()[1], -tableHeight]
        tableToStance = transformUtils.frameFromPositionAndRPY(relativePosition, relativeStance.GetOrientation() )
        t.Concatenate(tableToStance)

        vis.showFrame(t, relativeStanceFrameName, parent=om.findObjectByName('table'), scale=0.2)

    ### End Object Focused Functions ###############################################################
    ### Planning Functions ########################################################################

    def planFootsteps(self, goalFrame):
        startPose = self.getPlanningStartPose()
        request = self.footstepPlanner.constructFootstepPlanRequest(startPose, goalFrame)
        self.footstepPlan = self.footstepPlanner.sendFootstepPlanRequest(request, waitForResponse=True)

    def planWalking(self):
        startPose = self.getPlanningStartPose()
        plan = self.footstepPlanner.sendWalkingPlanRequest(self.footstepPlan, startPose, waitForResponse=True)
        self.addPlan(plan)

    def planWalkToStance(self, stanceTransform):
        if self.useFootstepPlanner:
            self.planFootsteps(stanceTransform)
            self.planWalking()
        else:
            self.teleportRobotToStanceFrame(stanceTransform)

    def planPostureFromDatabase(self, groupName, postureName, side='left'):
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, groupName, postureName, side=side)
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)
        # TODO: integrate this function with the ones below       

    def planSafeRaise1(self):
        self.planPostureFromDatabase('General', 'hand up avoid table 1', self.reachingHand)

    def planSafeRaise2(self):
        self.planPostureFromDatabase('General', 'hand up avoid table 2', self.reachingHand)

    def planArmsDown(self):
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'General', 'handsdown incl back')
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)

    def planReachToGoal(self):
        side = self.reachingHand
        startPose = self.getPlanningStartPose()
        goalFrame = self.getGoalFrame()
        #vis.showFrame(goalFrame.transform,'goalFrame')
        constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, side, goalFrame, lockBase=self.lockBase, lockBack=self.lockBack)
        endPose, info = constraintSet.runIk()
        plan = constraintSet.runIkTraj()
        self.addPlan(plan)

    ### End Planning Functions ####################################################################
    ########## Glue Functions #####################################################################
    def startSimulator(self):
        filename= os.environ['DRC_BASE'] + '/software/models/worlds/directorAffordances.sdf'
        msg=lcmdrc.scs_api_command_t()
        msg.command="loadSDF "+filename+"\nsimulate"
        lcmUtils.publish('SCS_API_CONTROL', msg)

    def moveRobotToTableStanceFrame(self):
        self.teleportRobotToStanceFrame(om.findObjectByName('table stance frame').transform)

    def teleportRobotToStanceFrame(self, frame):
        stancePosition = frame.GetPosition()
        stanceOrientation = frame.GetOrientation()

        q = self.sensorJointController.q.copy()
        q[:2] = [stancePosition[0], stancePosition[1]]
        q[5] = math.radians(stanceOrientation[2])
        self.sensorJointController.setPose('EST_ROBOT_STATE', q)

    def printAsync(self, s):
        yield
        print s

    def optionalUserPrompt(self, message):
        if not self.optionalUserPromptEnabled:
            return

        yield
        result = raw_input(message)
        if result != 'y':
            raise Exception('user abort.')

    def requiredUserPrompt(self, message):
        if not self.requiredUserPromptEnabled:
            return

        yield
        result = raw_input(message)
        if result != 'y':
            raise Exception('user abort.')

    def delay(self, delayTimeInSeconds):
        yield
        t = SimpleTimer()
        while t.elapsed() < delayTimeInSeconds:
            yield

    # TODO: re-enable
    #def waitForCleanLidarSweepAsync(self):
    #    currentRevolution = self.multisenseDriver.displayedRevolution
    #    desiredRevolution = currentRevolution + 2
    #    while self.multisenseDriver.displayedRevolution < desiredRevolution:
    #        yield

    def getEstimatedRobotStatePose(self):
        return self.sensorJointController.getPose('EST_ROBOT_STATE')

    def getPlanningStartPose(self):
        if self.planFromCurrentRobotState:
            return self.getEstimatedRobotStatePose()
        else:
            if self.plans:
                return robotstate.convertStateMessageToDrakePose(self.plans[-1].plan[-1])
            else:
                return self.getEstimatedRobotStatePose()

    def cleanupFootstepPlans(self):
        om.removeFromObjectModel(om.findObjectByName('walking goal'))
        om.removeFromObjectModel(om.findObjectByName('footstep plan'))
        self.footstepPlan = None

    def commitManipPlan(self):
        self.manipPlanner.commitManipPlan(self.plans[-1])

    def commitFootstepPlan(self):
        self.footstepPlanner.commitFootstepPlan(self.footstepPlan)

    def waitForPlanExecution(self, plan):
        planElapsedTime = planplayback.PlanPlayback.getPlanElapsedTime(plan)
        return self.delay(planElapsedTime + 1.0)


    def animateLastPlan(self):
        plan = self.plans[-1]
        if not self.visOnly:
            self.commitManipPlan()

        return self.waitForPlanExecution(plan)

    def onRobotModelChanged(self, model):

        for linkName in self.frameSyncs.keys():
            t = self.playbackRobotModel.getLinkFrame(linkName)
            vis.updateFrame(t, '%s frame' % linkName, scale=0.2, visible=False, parent='planning')


    #################################
    def loadTestPointCloud(self):
        filename = os.environ['DRC_BASE'] + '/../drc-testing-data/ihmc_table/ihmc_table.vtp'
        #filename = os.environ['DRC_BASE'] +  '/../drc-testing-data/tabletop/table-box-uoe.vtp'
        polyData = ioUtils.readPolyData( filename )
        vis.showPolyData(polyData,'scene')

        #stanceFrame = transformUtils.frameFromPositionAndRPY([0.0, 0, 0.0], [0,0,0])
        #self.teleportRobotToStanceFrame(stanceFrame)

                
    def placeHandModel(self):
        if not om.findObjectByName('end effector goal'):
            side = self.reachingHand
            handFrame = self.footstepPlanner.getFeetMidPoint(self.robotStateModel)
            handFrame.PreMultiply()
            if side == 'left':
                handFrame.Translate([0.4, 0.2, 1.02])
                rotation = [0, 90, -90]
            else:
                handFrame.Translate([0.4, -0.2, 1.02])
                rotation = [0, -90, -90]
            handFrame.Concatenate(transformUtils.frameFromPositionAndRPY([0,0,0], rotation))
            handFactory = HandFactory(self.robotStateModel)
            handFactory.placeHandModelWithTransform(handFrame, self.robotStateModel.views[0],
                                                    side, 'end effector goal', 'planning')
            handObj = om.findObjectByName('end effector goal frame')

            
    def changeHand(self, handName):
        self.reachingHand = handName
        if om.findObjectByName('end effector goal'):
            om.removeFromObjectModel(om.findObjectByName('end effector goal'))
            self.placeHandModel()

    def showPose(self, pose):
        self.hidePlan()
        self.teleopJointController.setPose('reach_end', pose)
        self.showTeleoModel()
        
    def hideTeleopModel(self):
        self.teleopRobotModel.setProperty('Visible', False)
        
    def showTeleoModel(self):
        self.teleopRobotModel.setProperty('Visible', True)
        self.teleopRobotModel.setProperty('Alpha', 1)
        
    def showPlan(self, plan):
        self.hideTeleopModel()
        self.playbackRobotModel.setProperty('Visible', True)
        
    def hidePlan(self):
        self.playbackRobotModel.setProperty('Visible', False)
        
    def getGoalFrame(self):
        return om.findObjectByName('end effector goal frame')

    def planWalkToTable(self):
        self.planWalkToStance(om.findObjectByName('table stance frame').transform)

           
    def handClose(self):        
        self.valkyrieDriver.sendHandCommand(side=self.reachingHand, thumbRoll=0.0, thumbPitch1=0.6, thumbPitch2=0.6, indexFingerPitch=0.6, middleFingerPitch=0.6, pinkyPitch=0.6)
        
    def handOpen(self):
        self.valkyrieDriver.openHand(side=self.reachingHand)
        
    def planNominal(self):
        self.ikPlanner.computeNominalPlan(self.sensorJointController.q)


'''
Tableboxdemo Image Fit for live-stream of webcam
'''
class TableImageFitter(ImageBasedAffordanceFit):

    def __init__(self, tablesimpleDemo):
        ImageBasedAffordanceFit.__init__(self, numberOfPoints=1)
        self.tablesimpleDemo = tablesimpleDemo

    def fit(self, polyData, points):
        pass

'''
Tablebox Task Panel
'''
class TableSimpleTaskPanel(TaskUserPanel):

    def __init__(self, tablesimpleDemo):

        TaskUserPanel.__init__(self, windowTitle='Table Task')

        self.tablesimpleDemo = tablesimpleDemo
        self.tablesimpleDemo.planFromCurrentRobotState = True

        self.addDefaultProperties()
        self.addButtons()
        self.addTasks()

        self.fitter = TableImageFitter(self.tablesimpleDemo)
        self.initImageView(self.fitter.imageView, activateAffordanceUpdater=False)

    def addButtons(self):

        self.addManualSpacer()
        self.addManualButton('Start Test Sim', self.tablesimpleDemo.startSimulator)
        self.addManualButton('Load Test Cloud', self.tablesimpleDemo.loadTestPointCloud)

        p1 = np.array([ 1.12926447, 0.09924124, 0.84685922])
        self.addManualButton('User Table', functools.partial(self.tablesimpleDemo.onSegmentTable, p1) )        
        self.addManualButton('Move to Stance',self.tablesimpleDemo.moveRobotToTableStanceFrame)

        self.addManualSpacer()
        self.addManualButton('Close Hand', self.tablesimpleDemo.handClose)
        self.addManualButton('Open Hand', self.tablesimpleDemo.handOpen)


    def addDefaultProperties(self):
        self.params.addProperty('Base', 1,
                                attributes=om.PropertyAttributes(enumNames=['Fixed', 'Free']))
        self.params.addProperty('Back', 1,
                                    attributes=om.PropertyAttributes(enumNames=['Fixed', 'Free']))

        self.params.addProperty('Reaching Hand', 0,
                                    attributes=om.PropertyAttributes(enumNames=['left', 'right']))

        # Init values as above
        self.tablesimpleDemo.lockBase = self.getLockBase()
        self.tablesimpleDemo.lockBack = self.getLockBack()

    def getLockBase(self):
        return True if self.params.getPropertyEnumValue('Base') == 'Fixed' else False

    def getLockBack(self):
        return True if self.params.getPropertyEnumValue('Back') == 'Fixed' else False
    
    def getReachingHand(self):
        return self.params.getPropertyEnumValue('Reaching Hand')

    def getPlanner(self):
        return self.params.getPropertyEnumValue('Planner') if self.params.hasProperty('Planner') else None

    def onPropertyChanged(self, propertySet, propertyName):
        propertyName = str(propertyName)

        if propertyName == 'Base':
            self.tablesimpleDemo.lockBase = self.getLockBase()

        elif propertyName == 'Back':
            self.tablesimpleDemo.lockBack = self.getLockBack()
        elif propertyName == 'Reaching Hand':
            self.tablesimpleDemo.changeHand(self.getReachingHand())
            
    def pickupMoreObjects(self):
        if len(self.tablesimpleDemo.clusterObjects) > 0: # There is still sth on the table, let's do it again!
            print "There are more objects on the table, going at it again!"
            self.taskTree.selectTaskByName('reach')
            self.onPause()
            self.onContinue()
        else:
            print "Table clear, sir!"

    def addTasks(self):

        # some helpers
        self.folder = None
        def addTask(task, parent=None):
            parent = parent or self.folder
            self.taskTree.onAddTask(task, copy=False, parent=parent)
        def addFunc(name, func, parent=None):
            addTask(rt.CallbackTask(callback=func, name=name), parent=parent)
        def addFolder(name, parent=None):
            self.folder = self.taskTree.addGroup(name, parent=parent)
            return self.folder


        def addManipulation(func, name, parent=None, confirm=False):
            group = self.taskTree.addGroup(name, parent=parent)
            addFunc(func, name='plan motion', parent=group)
            addTask(rt.CheckPlanInfo(name='check manip plan info'), parent=group)
            addFunc(v.commitManipPlan, name='execute manip plan', parent=group)

        def addManipTask(name, planFunc, userPrompt=False):

            prevFolder = self.folder
            addFolder(name, prevFolder)
            addFunc('plan motion', planFunc)
            if not userPrompt:
                addTask(rt.CheckPlanInfo(name='check manip plan info'))
            else:
                addTask(rt.UserPromptTask(name='approve manip plan', message='Please approve manipulation plan.'))
            addFunc('execute manip plan', self.tablesimpleDemo.commitManipPlan)
            addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'))
            self.folder = prevFolder

        v = self.tablesimpleDemo

        self.taskTree.removeAllTasks()

        ###############
        # preparation
        addFolder('prepare')
        addManipTask('move hands down', v.planArmsDown, userPrompt=True)
        addTask(rt.SetNeckPitch(name='set neck position to 35', angle=35))
        addFunc('activate table fit', v.userFitTable)
        addTask(rt.UserPromptTask(name='approve table fit', message='Please approve the table fit.'))
        
        # walk to table (end-pose planning)
        addFolder('walk to table')
        addTask(rt.RequestFootstepPlan(name='plan walk to table', stanceFrameName='table stance frame'))
        addTask(rt.UserPromptTask(name='approve footsteps', message='Please approve footstep plan.'))
        addTask(rt.CommitFootstepPlan(name='walk to table', planName='table stance frame footstep plan'))
        addTask(rt.WaitForWalkExecution(name='wait for walking'))

        addFolder('safely raise arm')
        addManipTask('action 1', v.planSafeRaise1, userPrompt=True)
        addManipTask('action 2', v.planSafeRaise2, userPrompt=True)

        # refit box
        addFolder('refit table')
        addFunc('re-activate table fit', v.userReFitTable)
        addTask(rt.UserPromptTask(name='approve table fit', message='Please approve the table fit again.'))

        addFolder('reach')
        addFunc('place hand goal', v.placeHandModel)
        addTask(rt.UserPromptTask(name='approve reach goal', message='Please approve the reach goal.'))
        addManipTask('reach to goal', v.planReachToGoal, userPrompt=True)


        # finish up
        addFolder('safely lower arm')
        addManipTask('action 2', v.planSafeRaise2, userPrompt=True)
        addManipTask('action 1', v.planSafeRaise1, userPrompt=True)
        addManipTask('move hands down', v.planArmsDown, userPrompt=True)