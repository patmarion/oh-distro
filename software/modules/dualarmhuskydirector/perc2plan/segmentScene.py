# This class is responsible for providing tools to : 
#		i)  segment the point cloud
# 		ii) extract frames of target objects


import numpy as np


from director import transformUtils
from director import visualization as vis
from director import objectmodel as om
from director import segmentation


class segScene(object):

	# oDPC --> openniDepthPointCloud
	def __init__(self, view, oDPC):


		# internal variable pointing to the view instance of director 
		self.view = view

		#  init a seed point for the segmentation
		self.seedSegmPoint = np.array(3)

		self.ptc = oDPC.polyData
	

	def getPointMouse(self):

		# internal callback for picking up point with mouse and saving it
		def ppCallback(p):
			# print "The selected points is at : ", p
			self.seedSegmPoint = np.array(p)


		from director import pointpicker

		# init a point picker class
		pPicker = pointpicker.PointPicker(view=self.view, numberOfPoints=1, callback=ppCallback)
		# start mouse picking functionality
		pPicker.start()

		# block till the point is selected
		raw_input("Press enter as soon as the point is selected")

		# stop mouse picking functionality
		pPicker.stop()
		
		# print "The selected point is at ", self.seedSegmPoint


	def fitTable(self):
		'''
		!!!!!!!!!!!!!!!
		Attention!!!!!!!!!!!!!!!!!!!
		Duplicated function from tabledemo.py!!!!!!!!!!!!!!!!!#
		Copy paste!
		'''

		# fit a plane on the surface of the "table"
		tableData = segmentation.segmentTableAndFrame(self.ptc, self.seedSegmPoint)

		# get the transformation of the fitted plane
		pose = transformUtils.poseFromTransform(tableData[0].frame)	

		desc = dict(classname='MeshAffordanceItem', Name='table', Color=[0, 1, 0], pose=pose)

		self.affordanceManager = segmentation.affordanceManager
		aff = self.affordanceManager.newAffordanceFromDescription(desc)
		aff.setPolyData(tableData[0].mesh)

		self.tableData = tableData[0]
		
		tableBox = vis.showPolyData(self.tableData.box, 'table box', parent=aff, color=[0, 1, 0], visible=False)
		tableBox.actor.SetUserTransform(self.tableData.frame)	
		
		self.addCollisionObject(aff)



	def addCollisionObject(self, obj):
		'''
		!!!!!!!!!!!!!!!
		Attention!!!!!!!!!!!!!!!!!!!
		Duplicated function from tabledemo.py!!!!!!!!!!!!!!!!!#
		Copy paste!
		'''
		from director import vtkNumpy
		
		# Affordance has been created previously
		if om.getOrCreateContainer('affordances').findChild(obj.getProperty('Name') + ' affordance'): return  

		frame = obj.findChild(obj.getProperty('Name') + ' frame')
		(origin, quat) = transformUtils.poseFromTransform(frame.transform)
		(xaxis, yaxis, zaxis) = transformUtils.getAxesFromTransform(frame.transform)

		# TODO: move this into transformUtils as getAxisDimensions or so
		#        box = obj.findChild(obj.getProperty('Name') + ' box')
		box = obj
		box_np = vtkNumpy.getNumpyFromVtk(box.polyData, 'Points')
		box_min = np.amin(box_np, 0)
		box_max = np.amax(box_np, 0)
		xwidth = np.linalg.norm(box_max[0] - box_min[0])
		ywidth = np.linalg.norm(box_max[1] - box_min[1])
		zwidth = np.linalg.norm(box_max[2] - box_min[2])
		name = obj.getProperty('Name') + ' affordance'

		boxAffordance = segmentation.createBlockAffordance(origin, xaxis, yaxis, zaxis, xwidth, ywidth, zwidth, name, \
						parent='affordances')
		boxAffordance.setSolidColor(obj.getProperty('Color'))
		boxAffordance.setProperty('Alpha', 0.3)



	def segmentTableObjects(self):
		'''
		!!!!!!!!!!!!!!!
		Attention!!!!!!!!!!!!!!!!!!!
		Duplicated function from tabledemo.py!!!!!!!!!!!!!!!!!#
		Copy paste!
		'''

		from director import affordanceitems


		tableFrame = om.findObjectByName('table').getChildFrame()
        #tableCentroid = segmentation.computeCentroid(self.tableData.box)
        #self.tableData.frame.TransformPoint(tableCentroid, tableFrame)

		data = segmentation.segmentTableScene(self.ptc, tableFrame.transform.GetPosition())
		data.clusters = self.sortClustersOnTable(data.clusters)

		objects = vis.showClusterObjects(data.clusters, parent='affordances')
		self.segmentationData = data

		self.clusterObjects = []
		for i, cluster in enumerate(objects):
		    affObj = affordanceitems.MeshAffordanceItem.promotePolyDataItem(cluster)
		    self.affordanceManager.registerAffordance(affObj)
		    self.clusterObjects.append(affObj)



	def sortClustersOnTable(self, clusters):
		'''
		!!!!!!!!!!!!!!!
		Attention!!!!!!!!!!!!!!!!!!!
		Duplicated function from tabledemo.py!!!!!!!!!!!!!!!!!#
		Copy paste!
		'''
		'''
		returns list copy of clusters, sorted left to right using the
		table coordinate system.  (Table y axis points right to left)
		'''
		tableTransform = om.findObjectByName('table').getChildFrame().transform

		tableYAxis = transformUtils.getAxesFromTransform(tableTransform)[1]
		tableOrigin = np.array(tableTransform.GetPosition())

		origins = [np.array(c.frame.GetPosition()) for c in clusters]
		dists = [np.dot(origin - tableOrigin, -tableYAxis) for origin in origins]

		return [clusters[i] for i in np.argsort(dists)]


	

	def reOrientFrame(self, namebase, namemove):
		'''
		Generates a new reorientated frame frame for the given object
		according to the orientation of the table
		'''

		objBase = om.findObjectByName(namebase)
		fobjBase = objBase.getChildFrame()
		fobjBasem = fobjBase.transform.GetMatrix()

		obj1 = om.findObjectByName(namemove)
		if obj1 == None:
			return

		fobj1 = obj1.getChildFrame()

		# retrieve the position and the orientation of the coord frame 
		pobj1 = fobj1.transform.GetPosition()
		oobj1 = fobjBase.transform.GetOrientation()

		# compose a new coordinate frame 
		f = transformUtils.frameFromPositionAndRPY(pobj1,oobj1)
		# added to the viewer
		newFrame = vis.showFrame(f, namemove+' newframe') 

		fobj1m = newFrame.transform.GetMatrix()

		for i in range(3):
			for j in range(3):
				fobj1m.SetElement(i,j,fobjBasem.GetElement(i,j))
 		for i in range(3):
 			fobj1m.SetElement(i,3,0.)

		newFrame.transform.SetMatrix(fobj1m)
		newFrame.transform.Translate(pobj1)


	def demoSegment(self):

		# select seed point 
		self.getPointMouse()

		# fit plane and table
		self.fitTable()

		# segment objects on the table
		self.segmentTableObjects()

		# fix objects's zero frame
		namebase = 'table'
		namemove = 'object 0'
		self.reOrientFrame(namebase,namemove)

		# fix objects's one frame
		namebase = 'table'
		namemove = 'object 1'
		self.reOrientFrame(namebase,namemove)

