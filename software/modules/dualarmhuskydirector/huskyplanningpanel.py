import PythonQt
import os

from PythonQt import QtCore, QtGui, QtUiTools
from director import applogic as app
from director.utime import getUtime
from director import transformUtils
from director import lcmUtils

def addWidgetsToDict(widgets, d):

    for widget in widgets:
        if widget.objectName:
            d[str(widget.objectName)] = widget
        addWidgetsToDict(widget.children(), d)


class WidgetDict(object):

    def __init__(self, widgets):
        addWidgetsToDict(widgets, self.__dict__)
        
class HuskyPlanningPanel(object):
    
    def __init__(self):
        loader = QtUiTools.QUiLoader()
        uifile = QtCore.QFile(os.path.join(os.path.dirname(__file__), 'ui/ddHuskyPlanningPanel.ui'))
        assert uifile.open(uifile.ReadOnly)

        self.widget = loader.load(uifile)
        self.widget.setWindowTitle('Husky Planning Panel')
        self.ui = WidgetDict(self.widget.children())
        
        self.ui.epPlanButton.connect('clicked()', self.epPlanButtonClicked)
        self.ui.mpPlanButton.connect('clicked()', self.mpPlanButtonClicked)

    def getBaseConstraint(self):
        return self.getComboText(self.ui.baseCombo)
    
    def getLeftArmConstraint(self):
        return self.getComboText(self.ui.larmCombo)
    
    def getRightArmConstraint(self):
        return self.getComboText(self.ui.rarmCombo)
    
    def updateConstraints(self):
        
        
    def epPlanButtonClicked(self):
        
    def mpPlanButtonClicked(self):    

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

def init():

    global panel
    global dock

    panel = HuskyPlanningPanel()
    dock = app.addWidgetToDock(panel.widget, action=_getAction())
    dock.hide()

    return panel