import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools
import director
from director import lcmUtils
from director import applogic as app
from director import actionhandlers
from director.utime import getUtime
from director.timercallback import TimerCallback
from director.simpletimer import SimpleTimer
import numpy as np
import math
from time import sleep
import subprocess
import os
import vtkAll as vtk

from time import time
from copy import deepcopy
from director import transformUtils
import director.visualization as vis
import director.objectmodel as om
from director import robotstate
from director import drcargs

from director import ioUtils as io
from director import vtkNumpy as vnp
from director import segmentation

import director.filterUtils as filterUtils

from kinect.map_command_t import map_command_t

from director.tasks.taskuserpanel import TaskUserPanel
from director.tasks.taskuserpanel import ImageBasedAffordanceFit
import director.tasks.robottasks as rt
import functools


class MappingPanel(object):

    def __init__(self, jointController, footstepDriver):

        self.jointController = jointController
        self.footstepDriver = footstepDriver

        self.queue = PythonQt.dd.ddBotImageQueue(lcmUtils.getGlobalLCMThread())
        self.queue.init(lcmUtils.getGlobalLCMThread(), drcargs.args().config_file)

    def onStartMappingButton(self):
        msg = map_command_t()
        msg.timestamp = getUtime()
        msg.command = 0
        lcmUtils.publish('KINECT_MAP_COMMAND', msg)

        utime = self.queue.getCurrentImageTime('KINECT_RGB')
        self.cameraToLocalInit = vtk.vtkTransform()
        self.queue.getTransform('KINECT_RGB', 'local', utime, self.cameraToLocalInit)
        vis.updateFrame(self.cameraToLocalInit, 'initial cam')
        print "starting mapping", utime
        print self.cameraToLocalInit.GetPosition()
        print self.cameraToLocalInit.GetOrientation()

    def onStopMappingButton(self):
        msg = map_command_t()
        msg.timestamp = getUtime()
        msg.command = 1
        lcmUtils.publish('KINECT_MAP_COMMAND', msg)

    def loadMap(self, filename, transform=None):
        print filename
        pd = io.readPolyData(filename)
        if transform is not None:
            pd = filterUtils.transformPolyData(pd, self.cameraToLocalInit)

        pdi = vis.updatePolyData(pd, 'map')
        try:
            pdi.setProperty('Color By', 'rgb_colors')
        except Exception, e:
            print "Could not set color to RGB - not an element"  # raise e

    def onShowMapButton(self):
        vis.updateFrame(self.cameraToLocalInit, 'initial cam')
        filename = os.path.expanduser('~/Kinect_Logs/Kintinuous.pcd')
        self.loadMap(filename, transform=self.cameraToLocalInit)

    def onHideMapButton(self):
        folder = om.getOrCreateContainer("segmentation")
        om.removeFromObjectModel(folder)


def init(jointController, footstepDriver):
    global panel
    panel = MappingPanel(jointController, footstepDriver)
    return panel

'''
Mapping Image Fit for live-stream of webcam
'''


class MappingImageFitter(ImageBasedAffordanceFit):

    def __init__(self, mappingDemo):
        ImageBasedAffordanceFit.__init__(self, numberOfPoints=1)
        self.mappingDemo = mappingDemo

    def fit(self, polyData, points):
        pass

'''
Mapping Task Panel
'''


class MappingTaskPanel(TaskUserPanel):

    def __init__(self, mappingDemo, mappingPanel):

        TaskUserPanel.__init__(self, windowTitle='Room Mapping')

        self.mappingDemo = mappingDemo
        self.mappingPanel = mappingPanel

        self.addButtons()
        self.addTasks()

        self.fitter = MappingImageFitter(self.mappingDemo)
        self.initImageView(self.fitter.imageView)

    def addButtons(self):

        self.addManualSpacer()
        self.addManualButton('Start Mapping', self.mappingPanel.onStartMappingButton)
        self.addManualSpacer()
        self.addManualButton('Stop/Reset Mapping', self.mappingPanel.onStopMappingButton)
        self.addManualSpacer()
        self.addManualButton('Show Map', self.mappingPanel.onShowMapButton)
        self.addManualSpacer()
        self.addManualButton('Hide Map', self.mappingPanel.onHideMapButton)
        self.addManualSpacer()
        self.addManualButton('Load file as map', self.loadFileAsMap)

    def onPropertyChanged(self, propertySet, propertyName):
        self.taskTree.removeAllTasks()
        self.addTasks()

    def loadFileAsMap(self):
        fileFilters = "Map Files (*.obj *.pcd *.ply *.stl *.vtk *.vtp)"
        filename = QtGui.QFileDialog.getOpenFileName(app.getMainWindow(), "Open...", actionhandlers.getDefaultDirectory(), fileFilters)
        if not filename:
            return
        self.mappingPanel.loadMap(filename)

    def addTasks(self):

        # some helpers
        def addTask(task, parent=None):
            self.taskTree.onAddTask(task, copy=False, parent=parent)

        def addFunc(func, name, parent=None, confirm=False):
            addTask(rt.CallbackTask(callback=func, name=name), parent=parent)
            if confirm:
                addTask(rt.UserPromptTask(name='Confirm execution has finished', message='Continue when plan finishes.'), parent=parent)

        def addManipulation(func, name, parent=None, confirm=False):
            group = self.taskTree.addGroup(name, parent=parent)
            addFunc(func, name='plan motion', parent=group)
            addTask(rt.CheckPlanInfo(name='check manip plan info'), parent=group)
            addFunc(v.commitManipPlan, name='execute manip plan', parent=group)
            addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'),
                    parent=group)
            if confirm:
                addTask(rt.UserPromptTask(name='Confirm execution has finished', message='Continue when plan finishes.'),
                        parent=group)

        v = self.mappingDemo

        self.taskTree.removeAllTasks()

        # graspingHand is 'left', side is 'Left'
        side = 'left'  # self.params.getPropertyEnumValue('Hand')

        if v.ikPlanner.fixedBaseArm:
            addManipulation(functools.partial(v.planPostureFromDatabase, 'General', 'arm up pregrasp', side=side), 'Arm up')
            addManipulation(functools.partial(v.planPostureFromDatabase, 'roomMapping', 'p1_up', side=side), 'Pose 1')
            addManipulation(functools.partial(v.planPostureFromDatabase, 'roomMapping', 'p1_down', side=side), 'Pose 2')
            addManipulation(functools.partial(v.planPostureFromDatabase, 'roomMapping', 'p2_up', side=side), 'Pose 3')
            addManipulation(functools.partial(v.planPostureFromDatabase, 'roomMapping', 'p2_down', side=side), 'Pose 4')
            addManipulation(functools.partial(v.planPostureFromDatabase, 'roomMapping', 'p3_up', side=side), 'Pose 5')
            addManipulation(functools.partial(v.planPostureFromDatabase, 'roomMapping', 'p3_down', side=side), 'Pose 6')
            addManipulation(functools.partial(v.planPostureFromDatabase, 'roomMapping', 'p4_up', side=side), 'Pose 7')
            addManipulation(functools.partial(v.planPostureFromDatabase, 'roomMapping', 'p4_down', side=side), 'Pose 8')
            addManipulation(functools.partial(v.planPostureFromDatabase, 'roomMapping', 'p5_up', side=side), 'Pose 9')
            addManipulation(functools.partial(v.planPostureFromDatabase, 'roomMapping', 'p5_down', side=side), 'Pose 10')
            addManipulation(functools.partial(v.planPostureFromDatabase, 'General', 'arm up pregrasp', side=side), 'Arm up')
