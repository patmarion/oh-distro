import valkyriedriver
import valkyriedriverpanel
import exampletaskpanel
import tableboxdemo
import tablesimpledemo
import tableplanningdemo
import manualwalkingdemo
import stairsdemo
import trajectorytrackingtest

from director import tasklaunchpanel
from director import applogic
from director import teleoppanel

import finger_test

import tablemapping
import manualwalkingdemo
import calisthenicsdemo


def startup(robotSystem, globalsDict=None):
    rs = robotSystem

    valkyrieDriver = valkyriedriver.ValkyrieDriver(rs.ikPlanner, rs.handFactory)
    valkyrieDriverPanel = valkyriedriverpanel.init(valkyrieDriver)

    atlasPanelAction = applogic.getToolBarActions()['ActionAtlasDriverPanel']
    applogic.getMainWindow().panelToolBar().removeAction(atlasPanelAction)

    tableboxDemo = tableboxdemo.TableboxDemo(rs.robotStateModel, rs.playbackRobotModel,
                    rs.ikPlanner, rs.manipPlanner, rs.footstepsDriver, rs.lHandDriver, rs.rHandDriver,
                    rs.view, rs.robotStateJointController)
    tableboxTaskPanel = tableboxdemo.TableboxTaskPanel(tableboxDemo)
    tasklaunchpanel.panel.addTaskPanel('Box Pick', tableboxTaskPanel.widget)



    tablesimpleDemo = tablesimpledemo.TableSimpleDemo(rs.robotStateModel, rs.playbackRobotModel,
                   rs.ikPlanner, rs.manipPlanner, rs.footstepsDriver, rs.lHandDriver, rs.rHandDriver,
                   rs.view, rs.robotStateJointController)#, teleopPanel, playbackPanel, jointLimitChecker)
    tablesimpleTaskPanel = tablesimpledemo.TableSimpleTaskPanel(tablesimpleDemo)
    tasklaunchpanel.panel.addTaskPanel('Table Simple', tablesimpleTaskPanel.widget)


    #tableDemo = tabledemo.TableDemo(robotStateModel, playbackRobotModel,
    #               ikPlanner, manipPlanner, footstepsDriver, atlasdriver.driver, lHandDriver, rHandDriver,
    #                perception.multisenseDriver, view, robotStateJointController, playPlans, teleopPanel, playbackPanel, jointLimitChecker)
    #tableTaskPanel = tabledemo.TableTaskPanel(tableDemo)

    tableplanningDemo = tableplanningdemo.TableplanningDemo(rs.robotStateModel, rs.playbackRobotModel,
                    rs.ikPlanner, rs.manipPlanner, rs.footstepsDriver, rs.lHandDriver, rs.rHandDriver,
                    rs.view, rs.robotStateJointController, rs.teleopRobotModel, rs.teleopJointController, rs.footstepsDriver, valkyrieDriver)
    tableplanningTaskPanel = tableplanningdemo.TableplanningTaskPanel(tableplanningDemo)
    tasklaunchpanel.panel.addTaskPanel('Table Planning', tableplanningTaskPanel.widget)

    tableMapping = tablemapping.TableMapping(rs.robotStateModel, rs.manipPlanner, rs.view,  rs.ikPlanner, rs.robotStateJointController)
    tableMappingTaskPanel = tablemapping.TableTaskPanel(tableMapping)
    tasklaunchpanel.panel.addTaskPanel("Table Mapping", tableMappingTaskPanel.widget)
    
    stairsDemo = stairsdemo.StairsDemo(rs.robotStateModel, rs.footstepsDriver, rs.robotStateJointController, rs.ikPlanner, rs.manipPlanner)
    stairsTaskPanel = stairsdemo.StairsTaskPanel(stairsDemo)
    tasklaunchpanel.panel.addTaskPanel('Stairs', stairsTaskPanel.widget)

    
    # Standing on one leg and doing some postures
    calisthenicsDemo = calisthenicsdemo.CalisthenicsDemo(rs.robotStateModel,
                    rs.footstepsDriver, rs.robotStateJointController, rs.ikPlanner, rs.manipPlanner)
    calisthenicsTaskPanel = calisthenicsdemo.CalisthenicsTaskPanel(calisthenicsDemo)
    tasklaunchpanel.panel.addTaskPanel('Calisthenics', calisthenicsTaskPanel.widget)

    ############## Module Specifically for testing performance
    # Test Walking Performance
    manualWalkingDemo = manualwalkingdemo.ManualWalkingDemo(rs.robotStateModel,
                    rs.footstepsDriver, rs.robotStateJointController, rs.ikPlanner)
    manualWalkingTaskPanel = manualwalkingdemo.ManualWalkingTaskPanel(manualWalkingDemo)
    tasklaunchpanel.panel.addTaskPanel('Walking Test', manualWalkingTaskPanel.widget)

    # Test Arm Tracking
    trajectoryTrackingTest = trajectorytrackingtest.TrajectoryTrackingTest(rs.ikPlanner, rs.manipPlanner, rs.robotStateJointController,
                    rs.footstepsDriver, rs.robotStateModel)
    trackingTestPanel = trajectorytrackingtest.TrackingTestPanel(trajectoryTrackingTest)
    tasklaunchpanel.panel.addTaskPanel('Tracking Test', trackingTestPanel.widget)

    # testing fingers by opening/clsoing sequence
    fingerTest = finger_test.FingerTest()
    fingerTestTaskPanel = finger_test.FingerTestTaskPanel(fingerTest)
    tasklaunchpanel.panel.addTaskPanel('Finger test', fingerTestTaskPanel.widget)


    if globalsDict is not None:
        globalsDict['valkyrieDriver'] = valkyrieDriver
        globalsDict['valkyrieDriverPanel'] = valkyrieDriverPanel

        globalsDict['tableboxDemo'] = tableboxDemo

        globalsDict['tablesimpleDemo'] = tablesimpleDemo
        globalsDict['tableplanningDemo'] = tableplanningDemo


        globalsDict['manualWalkingDemo'] = manualWalkingDemo
        globalsDict['manualWalkingTaskPanel'] = manualWalkingTaskPanel

        globalsDict['stairsDemo'] = stairsDemo
        globalsDict['stairsTaskPanel'] = stairsTaskPanel

        globalsDict['tableMappingTaskPanel'] = tableMappingTaskPanel

        globalsDict['calisthenicsDemo'] = calisthenicsDemo
