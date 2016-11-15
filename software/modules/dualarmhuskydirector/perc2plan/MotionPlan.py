# This class is responsible for providing tools to : 
#		i)   Solve IK for both arms of the dual arm husky
# 		ii)  Plan motion to reach obects 
#       iii) Plan motions between two configurations (q)	


import numpy as np
from director import objectmodel as om

#-----------------------------------------------------------------------------
#-----------------------------------------------------------------------------

class ReachObjects(object):

	def __init__(self, ikPlanner, robotStateModel, planningUtils, teleopJointController, vis):

		self.graspToPalmFrame = self.genPreGrasp2Palm()

		self.ikP = ikPlanner
		self.rSM = robotStateModel
		# planning utils class
		self.pU = planningUtils
		# teleoperation controller class
		self.tJC = teleopJointController

		# playback robotstate init
		#self.pbSM = playbackRobotModel

		self.vis = vis

		# list of configurations q
		self.configlist = [self.tJC.q]


		# set planner to exotica
		self.ikP.planningMode = 'exotica'

		# the arm is not fixed, because then it means that it is only a single arm
		self.ikP.fixedBaseArm = False



	def visualPCPframe(self, side):
		'''
		Visualisation function to illustrate where the pcp frame is placed
		'''

		endeff = self.ikP.getHandLink(side)
		frame = self.rSM.getLinkFrame(endeff)
 		
		grasp2ArmLink = self.ikP.newGraspToHandFrame(side, self.graspToPalmFrame)

 		frame.PreMultiply()
 		frame.Concatenate(grasp2ArmLink)
 		vis.showFrame(frame, side + '_pcpFrame') 


 	def setGraspFrame(self):
 		self.graspToPalmFrame = self.genGrasp2Palm()


	@staticmethod 
	def genGrasp2Palm():
		'''
		Generate the transformation from the palm frame 
		to the palm-contact-point(pcp) frame
		'''

		from director import vtkAll as vtk

		t = vtk.vtkTransform()
		t.RotateZ(90)
		t.Translate([0.09,0,0])

		return t


 	def setPreGraspFrame(self):
 		self.graspToPalmFrame = self.genPreGrasp2Palm()

 		
	@staticmethod 
	def genPreGrasp2Palm():
		'''
		Generate the transformation from the palm frame 
		to the pre grasp frame
		'''

		from director import vtkAll as vtk

		t = vtk.vtkTransform()
		t.RotateZ(90)
		t.Translate([0.17,0,0])

		return t


	def reachDualFrames(self, rtarFrame = None, ltarFrame = None):
		'''
		Init all planning parameters and perform the planning
		'''

		# set side of the manipulator
		rside ='right'
		lside ='left'

		# give a name to the start pose	
		startPoseName = 'reach_start'
		endPoseName = 'reach_end'

		# get starting configuration of the robot
		# startPose = self.pU.getPlanningStartPose()
		startPose = self.configlist[-1]

		# give to ik the starting pose
		self.ikP.addPose(startPose, startPoseName)

		# create constraint w.r.t the task 
		# ===========================
		constraints = []

		if rtarFrame != None:
			rpc, roc = self.genArmConstraints(rside, rtarFrame)
			constraints.append(rpc)
			constraints.append(roc)
		
		if ltarFrame != None:	
			lpc, loc = self.genArmConstraints(lside, ltarFrame)
			constraints.append(lpc)
			constraints.append(loc)

		
		conSet = self.genConstraints(startPoseName, endPoseName, constraints)

		# run IK for the final configuration
		reach_end, info = conSet.runIk()
		print " info of the plan " , info

		self.configlist.append(np.asarray(reach_end))
		# visualise final configuration..
		#self.tJC.setPose(endPoseName,reach_end)

		# realise the plan
		plan = conSet.runIkTraj()


	def plan_q1_q2(self, startq = None, endq = None):
		'''
		Init all planning parameters and perform the planning
		'''

		# give a name to the start pose	
		startPoseName = 'reach_start'
		endPoseName = 'reach_end'

		# give to ik the starting pose
		self.ikP.addPose(startq, startPoseName)

		# give to ik the final pose
		self.ikP.addPose(endq, endPoseName)

		# realise the plan
		plan = self.ikP.runIkTraj([],startPoseName,endPoseName)


	def genConstraints(self, startPoseName, endPoseName, constraints):
		'''
		Introduce general constraints of the robot
		'''
		
		# lock - fix the base of the robot
		constraints.append(self.ikP.createLockedBasePostureConstraint(startPoseName))

		from director import ikplanner
		constraintSet = ikplanner.ConstraintSet(self.ikP, constraints, \
											endPoseName, startPoseName)

		return constraintSet


	def genArmConstraints(self, side, targetFrame):
		'''
		Introduce arm constraints
		'''

		graspToHandLinkFrame = self.ikP.newGraspToHandFrame(side, self.graspToPalmFrame)
		posCon, orientCon = self.ikP.createPositionOrientationGraspConstraints(side, \
    			targetFrame, graspToHandLinkFrame, \
    			positionTolerance=0.0, angleToleranceInDegrees=0.0)
	
		posCon.tspan = [1.0, 1.0]
		orientCon.tspan = [1.0, 1.0]
	
		return posCon, orientCon



	def reachDual(self, rframe = None, lframe = None):
		'''
		Both arms are planning a reaching motion to the respective goal frames
		'''

		# reach the target object
		#if rframe != None and lframe != None:
		self.reachDualFrames(rtarFrame = om.findObjectByName(rframe), \
							 ltarFrame = om.findObjectByName(lframe))
		#else: 

		#	print "Not enough goal frames were provided!"


	def reachLeft(self, name):
		'''
		The left arm is planning a reaching motion to the goal frame
		
		name : is the name of the goal frame
		'''

		# reach the target object
		self.reachDualFrames(ltarFrame = om.findObjectByName(name))


	def reachRight(self, name):
		'''
		The right arm is planning a reaching motion to the goal frame

		name : is the name of the goal frame
		'''

		# reach the target object
		self.reachDualFrames(rtarFrame = om.findObjectByName(name))
