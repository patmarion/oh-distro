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

def startup(robotSystem, globalsDict=None):
    rs = robotSystem

    # Remove Atlas Panel
    atlasPanelAction = applogic.getToolBarActions()['ActionAtlasDriverPanel']
    applogic.getMainWindow().panelToolBar().removeAction(atlasPanelAction)

    # Assert that we are starting the Dual Arm Husky
    assert globalsDict is not None
    assert 'directorConfig' in globalsDict
    directorConfig = globalsDict['directorConfig']

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
    globalsDict['huskyPlanningPlanel'] = huskyPlanningPlanel