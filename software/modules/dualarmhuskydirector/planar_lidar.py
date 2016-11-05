# Hacky, needs more testing - to run:
# from dualarmhuskydirector import planar_lidar; planar_lidar.setup_planar_lidar()

from director import vtkAll as vtk
from director import visualization as vis
from director import cameraview
from director import filterUtils
from director import lcmUtils
import bot_core as lcmbot
import math

def setup_planar_lidar():
    def onPlanarLidar(msg, channel):
        sensorToLocal = vtk.vtkTransform()
        polyData = vtk.vtkPolyData()
        points = vtk.vtkPoints()
        verts = vtk.vtkCellArray()

        cameraview.imageManager.queue.getTransform('SICK_SCAN', 'local', msg.utime, sensorToLocal)
        
        t = msg.rad0
        for r in msg.ranges:
            if r >= 0:
                x = r * math.cos(t)
                y = r * math.sin(t)

                pointId = points.InsertNextPoint([x,y,0])
                verts.InsertNextCell(1)
                verts.InsertCellPoint(pointId)

            t += msg.radstep

        polyData.SetPoints(points)
        polyData.SetVerts(verts)

        polyData = filterUtils.transformPolyData(polyData, sensorToLocal)

        vis.updatePolyData(polyData, channel)

    lcmUtils.addSubscriber('FRONT_SICK_SCAN', lcmbot.planar_lidar_t, onPlanarLidar, callbackNeedsChannel=True)
