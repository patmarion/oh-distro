import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools
import director.applogic as app
import director.objectmodel as om
from director import transformUtils
from director import ikplanner
from director import footstepsdriver
#from director import vtkAll as vtk
from director import lcmUtils
from director import visualization as vis
import os

import rospy
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalStatusArray
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Twist

navigationStatusStrings = ['PENDING', 'ACTIVE', 'PREEMPTED', 'SUCCEEDED', 'ABORTED', 'REJECTED', 'PREEMPTING', 'RECALLING', 'RECALLED', 'LOST']

def addWidgetsToDict(widgets, d):

    for widget in widgets:
        if widget.objectName:
            d[str(widget.objectName)] = widget
        addWidgetsToDict(widget.children(), d)


class WidgetDict(object):

    def __init__(self, widgets):
        addWidgetsToDict(widgets, self.__dict__)
        
class HuskyNavigationPanel(object):
    def __init__(self, robotStateModel, teleopRobotModel, teleopJointController):
        self.robotStateModel = robotStateModel
        self.teleopRobotModel = teleopRobotModel
        self.teleopJointController = teleopJointController
        
        loader = QtUiTools.QUiLoader()
        uifile = QtCore.QFile(os.path.join(os.path.dirname(__file__), 'ui/ddHuskyNavigationPanel.ui'))
        assert uifile.open(uifile.ReadOnly)
        self.widget = loader.load(uifile)
        self.widget.setWindowTitle('Husky Navigation Panel')
        self.ui = WidgetDict(self.widget.children())
        
        # Activate/Deactivate different panels
        self.ui.navigationPanel.enabled = False
        self.ui.manualPanel.enabled = False
        self.ui.navigationControlButton.connect('clicked()', self.onActivateNavigationControl)
        self.ui.manualControlButton.connect('clicked()', self.onActivateManualControl)

        # Navigation panel control
        self.ui.newGoalButton.connect('clicked()', self.onNewNavigationGoal)
        self.ui.moveButton.connect('clicked()', self.moveToGoal)
        self.ui.stopButton.connect('clicked()', self.cancelGoal)
        
        self.goalPub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)
        self.cancelPub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
        self.cmdVelPub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        #Some unknown lags in getting live status, TODO
        self.statusSub = rospy.Subscriber("/move_base/status", GoalStatusArray, self.statusCallback, queue_size=1)
        
        # Manual control
        self.ui.forwardButton.connect('clicked()', self.onMoveForward)
        self.ui.backwardButton.connect('clicked()', self.onMoveBackward)
        self.ui.turnLeftButton.connect('clicked()', self.onTurnLeft)
        self.ui.turnRightButton.connect('clicked()', self.onTurnRight)
        self.ui.forwardRightButton.connect('clicked()', self.onMoveForwardRight)
        self.ui.forwardLeftButton.connect('clicked()', self.onMoveForwardLeft)
        self.ui.backwardRightButton.connect('clicked()', self.onMoveBackwardRight)
        self.ui.backwardLeftButton.connect('clicked()', self.onMoveBackwardLeft)

    def removeNavGoal(self):
        obj = om.findObjectByName('navigation goal')
        if obj:
            om.removeFromObjectModel(obj)

    def onActivateNavigationControl(self):
        self.cancelGoal()
        self.removeNavGoal()
        self.ui.navigationPanel.enabled = self.ui.navigationControlButton.checked

    def onActivateManualControl(self):
        self.ui.manualPanel.enabled = self.ui.manualControlButton.checked

    def newNavGoalFrame(self, robotStateModel, distanceForward=1.0):
        t = robotStateModel.getLinkFrame('base_footprint')
        t.PreMultiply()
        t.Translate(distanceForward, 0.0, 0.0)
        #t.PostMultiply()
        return t
    
    def onNewNavigationGoal(self, navGoal=None):
        navGoal = navGoal or self.newNavGoalFrame(self.robotStateModel)
        frameObj = vis.updateFrame(navGoal, 'navigation goal', parent='planning', scale=0.25)
        frameObj.setProperty('Edit', True)
        
        rep = frameObj.widget.GetRepresentation()
        rep.SetTranslateAxisEnabled(2, False)
        rep.SetRotateAxisEnabled(0, False)
        rep.SetRotateAxisEnabled(1, False)
        frameObj.widget.HandleRotationEnabledOff()
        
    def moveToGoal(self):
        frameObj = om.findObjectByName('navigation goal')
        t = frameObj.transform
        goal_msg = MoveBaseActionGoal()
        goal_msg.goal.target_pose.header.frame_id = 'odom'
        goal_msg.goal.target_pose.pose.position.x = t.GetPosition()[0]
        goal_msg.goal.target_pose.pose.position.y = t.GetPosition()[1]
        goal_msg.goal.target_pose.pose.position.z = 0

        goal_msg.goal.target_pose.pose.orientation.x = 0
        goal_msg.goal.target_pose.pose.orientation.y = 0
        goal_msg.goal.target_pose.pose.orientation.z = 1
        goal_msg.goal.target_pose.pose.orientation.w = t.GetOrientation()[2]/180.0
        
        self.goalPub.publish(goal_msg)
        
    def cancelGoal(self):
        cancel_msg = GoalID()
        self.cancelPub.publish(cancel_msg)
        
    def statusCallback(self, data):
        if data.status_list[0]:
            self.ui.status.setText(navigationStatusStrings[data.status_list[0].status])
    
    def getLinearSpeed(self):
        return self.ui.linearSpeedBox.value
    
    def getAngularSpeed(self):
        return self.ui.angularSpeedBox.value
    
    def onMoveForward(self):
        new_msg = Twist()
        new_msg.linear.x = self.getLinearSpeed()
        self.cmdVelPub.publish(new_msg)
        rospy.sleep(1.0)
        new_msg.linear.x = 0.0
        self.cmdVelPub.publish(new_msg)
        
    def onMoveBackward(self):
        new_msg = Twist()
        new_msg.linear.x = -self.getLinearSpeed()
        self.cmdVelPub.publish(new_msg)
        rospy.sleep(1.0)
        new_msg.linear.x = 0.0
        self.cmdVelPub.publish(new_msg)
        
    def onTurnRight(self):
        new_msg = Twist()
        new_msg.angular.z = -self.getLinearSpeed()
        self.cmdVelPub.publish(new_msg)
        rospy.sleep(2.0)
        new_msg.angular.z = 0.0
        self.cmdVelPub.publish(new_msg)
        
    def onTurnLeft(self):
        new_msg = Twist()
        new_msg.angular.z = self.getLinearSpeed()
        self.cmdVelPub.publish(new_msg)
        rospy.sleep(2.0)
        new_msg.angular.z = 0.0
        self.cmdVelPub.publish(new_msg)
        
    def onMoveForwardRight(self):
        new_msg = Twist()
        new_msg.linear.x = self.getLinearSpeed()
        new_msg.angular.z = -self.getLinearSpeed()
        self.cmdVelPub.publish(new_msg)
        rospy.sleep(1.0)
        new_msg.angular.z = 0.0
        new_msg.linear.x = 0.0
        self.cmdVelPub.publish(new_msg)
    
    def onMoveForwardLeft(self):
        new_msg = Twist()
        new_msg.linear.x = self.getLinearSpeed()
        new_msg.angular.z = self.getLinearSpeed()
        self.cmdVelPub.publish(new_msg)
        rospy.sleep(1.0)
        new_msg.angular.z = 0.0
        new_msg.linear.x = 0.0
        self.cmdVelPub.publish(new_msg)
        
    def onMoveBackwardRight(self):
        new_msg = Twist()
        new_msg.linear.x = -self.getLinearSpeed()
        new_msg.angular.z = self.getLinearSpeed()
        self.cmdVelPub.publish(new_msg)
        rospy.sleep(1.0)
        new_msg.angular.z = 0.0
        new_msg.linear.x = 0.0
        self.cmdVelPub.publish(new_msg)
        
    def onMoveBackwardLeft(self):
        new_msg = Twist()
        new_msg.linear.x = -self.getLinearSpeed()
        new_msg.angular.z = -self.getLinearSpeed()
        self.cmdVelPub.publish(new_msg)
        rospy.sleep(1.0)
        new_msg.angular.z = 0.0
        new_msg.linear.x = 0.0
        self.cmdVelPub.publish(new_msg)
        
def _getAction():

    actionName = 'ActionHuskyNavigationPanel'
    action = app.getToolBarActions().get(actionName)

    if action is None:
        icon = QtGui.QIcon(os.path.join(os.path.dirname(__file__), 'images/husky.png'))
        assert os.path.isfile(os.path.join(os.path.dirname(__file__), 'images/husky.png'))
        action = QtGui.QAction(icon, 'Husky Navigation Panel', None)
        action.objectName = actionName
        action.checkable = True

        mainWindow = app.getMainWindow()
        toolbar = mainWindow.panelToolBar()

        toolbar.insertAction(toolbar.actions()[0], action)

    return action

def init(robotStateModel, teleopRobotModel, teleopJointController):
    global panel
    global dock

    panel = HuskyNavigationPanel(robotStateModel, teleopRobotModel, teleopJointController)
    dock = app.addWidgetToDock(panel.widget, action=_getAction())
    dock.hide()

    return panel
