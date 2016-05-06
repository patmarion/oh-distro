import numpy as np
from director.debugVis import DebugData
from director import transformUtils
from director import objectmodel as om
from director import segmentation
from director import visualization as vis
from director import vtkAll as vtk

def showUoeViconMarkers():
	# Used for experiments - 3 May 2016
	# from pelvis_val.vsk
	m1 = np.array([42.430198669433594, 0.38291072845458984, -75.745498657226562]) * 0.001
	m2 = np.array([36.803276062011719, 0.23376750946044922, -16.981033325195312]) * 0.001
	m3 = np.array([-24.306129455566406, -119.755126953125, 47.446334838867188]) * 0.001
	m4 = np.array([-54.927345275878906, 119.13845062255859, 45.280197143554688]) * 0.001

	# measurment info from jlack:
	point1 = np.array([0.1099,0,-0.2015])
	point2 = np.array([0.1099,0,-0.14053])
	point3 = np.array([0.0512,-0.1194,-0.0717])
	point4 = np.array([0.0512,0.1194,-0.0717])
	# 5cm and 2.5cm extensions with 7.5mm diameter marker dots
	point1[0] += 0.0575
	point2[0] += 0.0575
	point3[0] += 0.0575
	point4[0] += 0.0325

	d2 = DebugData()
	d2.addSphere(m1, radius=0.01)
	d2.addSphere(m2, radius=0.01)
	d2.addSphere(m3, radius=0.01)
	d2.addSphere(m4, radius=0.01)

	d = DebugData()
	d.addSphere(point1, radius=0.01)
	d.addSphere(point2, radius=0.01)
	d.addSphere(point3, radius=0.01)
	d.addSphere(point4, radius=0.01)

	robotStateModel = om.findObjectByName('robot state model')

	worldToPelvis = transformUtils.copyFrame( robotStateModel.getLinkFrame( 'pelvis') )

	# hand made alignment
	pos = [-0.143, -0.002, 0.1065]
	rpy = [0, -6.*np.pi/180.0, 1.0*np.pi/180.0]

	quat = transformUtils.rollPitchYawToQuaternion(rpy)
	viconToPelvis = transformUtils.transformFromPose(pos, quat)


	pd = segmentation.transformPolyData(d.getPolyData(), worldToPelvis)
	obj = vis.updatePolyData(pd, 'nasa model', visible=True, color=[1,0,0])



	worldToVicon =  transformUtils.copyFrame(worldToPelvis)
	worldToVicon.PreMultiply()
	worldToVicon.Concatenate( viconToPelvis.GetLinearInverse() )

	vis.showFrame(worldToVicon,'vicon markers frame')

	pd2 = segmentation.transformPolyData(d2.getPolyData(), worldToVicon)
	obj = vis.updatePolyData(pd2, 'vicon markers', visible=True, color=[0,1,0])

	p = np.matrix([point1, point2, point3, point4]).transpose()
	m = np.matrix([m1, m2, m3, m4]).transpose()

	pMean = np.empty([3,1])
	mMean = np.empty([3,1])

	for row in range(4):
		mMean += m[:, row]
		pMean += p[:, row]

	mMean /= 4.
	pMean /= 4.

	w = np.empty([3,3])

	for row in range(4):
		w = w + (p[:, row] - pMean) * (m[:, row] - mMean).transpose() 

	[u,s,vT] = np.linalg.svd(w)

	r = u * vT
	t = pMean - r * mMean

	pelvisToVicon = vtk.vtkMatrix4x4()

	for line in range(3):
		for row in range(3):
			pelvisToVicon.SetElement(line, row, r[line, row])

	for line in range(3):
		pelvisToVicon.SetElement(line, 3, t[line, 0])

	vtkPelvisToVicon = vtk.vtkTransform()
	vtkPelvisToVicon.SetMatrix(pelvisToVicon)

	#markers relative to vicon pelvis frame
	d3 = DebugData()
	d3.addSphere(m1, radius=0.01)
	d3.addSphere(m2, radius=0.01)
	d3.addSphere(m3, radius=0.01)
	d3.addSphere(m4, radius=0.01)

	worldToViconClosed =  transformUtils.copyFrame(worldToPelvis)
	worldToViconClosed.PreMultiply()
	worldToViconClosed.Concatenate(vtkPelvisToVicon)

	pd3 = segmentation.transformPolyData(d3.getPolyData(), worldToViconClosed)
	obj = vis.updatePolyData(pd3, 'closed form solution', visible=True, color=[0,0,1])
