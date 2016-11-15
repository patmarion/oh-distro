'''
Robot-specific modifications for the Dual Arm Husky
'''

from director import tasklaunchpanel
from director import applogic
from director import teleoppanel
from director import planningutils

import tabledemo
import mappingpanel
import mappingdemo
import huskyplanningpanel
import huskynavigationpanel

import rospy

def startup(robotSystem, globalsDict=None):
    rs = robotSystem

    # Assert that we are starting the Dual Arm Husky
    assert globalsDict is not None
    assert 'directorConfig' in globalsDict
    directorConfig = globalsDict['directorConfig']

    # Initialise global rosNode
    rospy.init_node('director')

    # Reduce number of SICK_SCAN scan lines
    assert 'perception' in globalsDict
    perception = globalsDict['perception']
    perception._lidarItem.setProperty('Number of Scan Lines', 2)
    perception._lidarItem.setProperty('Point Size', 2)
    perception._lidarItem.setProperty('Min Height', -1)
    perception._lidarItem.setProperty('Max Height', 2.5)
    perception._lidarItem.setProperty('Min Range', 0.4)

    # Automatically activate and set OpenNI settings
    assert 'openniDepthPointCloud' in globalsDict
    openniDepthPointCloud = globalsDict['openniDepthPointCloud']
    openniDepthPointCloud.setProperty('Target FPS', 15)
    openniDepthPointCloud.setProperty('Visible', True)
    openniDepthPointCloud.setProperty('Max Range', 4.0)

    # Remove Humanoid Motion Planning Panel
    humanoidMotionPlanningPanel = applogic.getToolBarActions()['ActionMotionPlanningPanel']
    applogic.getMainWindow().panelToolBar().removeAction(humanoidMotionPlanningPanel)

    # Dual Arm Husky Demos
    playPlans = globalsDict['playPlans']
    playbackPanel = globalsDict['playbackPanel']
    jointLimitChecker = globalsDict['jointLimitChecker']
    perception = globalsDict['perception']
    planningUtils = globalsDict['planningUtils']

    ## Remove all task panels
    for taskPanel in tasklaunchpanel.panel.getTaskPanelNames():
        tasklaunchpanel.panel.removeTaskPanel(taskPanel)

    ## Add task panels
    # add a new task panel
    #exampleTaskPanel = exampletaskpanel.ExampleTaskPanel(robotSystem)
    #tasklaunchpanel.panel.addTaskPanel('Example Task', exampleTaskPanel.widget)

    mappingDemo = mappingdemo.MappingDemo(rs.ikPlanner, rs.manipPlanner, rs.robotStateJointController)
    mappingPanel = mappingpanel.init(rs.robotStateJointController, rs.footstepsDriver)
    mappingTaskPanel = mappingpanel.MappingTaskPanel(mappingDemo, mappingPanel)

    tableDemo = tabledemo.TableDemo(rs.robotStateModel, rs.playbackRobotModel,
            rs.ikPlanner, rs.manipPlanner, rs.footstepsDriver, rs.lHandDriver, rs.rHandDriver,
            rs.view, rs.robotStateJointController, playPlans, playbackPanel, jointLimitChecker)
    tableTaskPanel = tabledemo.TableTaskPanel(tableDemo)
    
    huskyNavigationPanel = huskynavigationpanel.init(rs.robotStateModel, rs.teleopRobotModel, rs.teleopJointController)
    
    huskyPlanningPlanel = huskyplanningpanel.init(planningUtils, rs.robotStateModel, 
            rs.robotStateJointController, rs.teleopRobotModel, rs.teleopJointController, 
            rs.ikPlanner, rs.manipPlanner, rs.affordanceManager, playbackPanel.setPlan, 
            playbackPanel.hidePlan)
    
    tasklaunchpanel.panel.addTaskPanel('Clearing Demo', tableTaskPanel.widget)
    tasklaunchpanel.panel.addTaskPanel('Mapping Demo', mappingTaskPanel.widget)

    # Export demos and panels to global
    globalsDict['tableDemo'] = tableDemo
    globalsDict['tableTaskPanel'] = tableTaskPanel
    globalsDict['mappingDemo'] = mappingDemo
    globalsDict['mappingPanel'] = mappingPanel
    globalsDict['mappingTaskPanel'] = mappingTaskPanel
    globalsDict['huskyNavigationPanel'] = huskyNavigationPanel
    globalsDict['huskyPlanningPlanel'] = huskyPlanningPlanel