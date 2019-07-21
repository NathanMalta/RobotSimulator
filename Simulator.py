
import random
import math
import matplotlib.pyplot as plt
from MotorModel import MotorModel

class Simulator():
	#Constants
	robotPosRange = (-50, 50) #robot x and y coord min and max values
	targetRelAngleRange = (-math.pi/8, math.pi/8) #range that is considered "in front of" the robot for the target to exist
	targetRelDistanceRange = (65, 70) #max and min allowed distances away from the target
	targetRelThetaRange = (-math.pi/6, math.pi/6) #range for the target's 
	simLineLength = 8 #length of simulator lines to display target and robot angle

	dtRange = (0.01, 0.03) #simulated time change between loops

	robotWheelBase = 8 #wheelbase of the robot #TODO: fix

	#instance variables
	robotPosAbsolute = (0,0,0) #x(in), y (in), heading (rad)
	robotStartPosAbsolute = (0,0,0) #x(in), y (in), heading (rad)
	targetPosAbsolute = (0,0,0) #x(in), y (in), heading (rad)
	rightMotor = None #a simple model of the right motor
	leftMotor = None #a simple model of the left motor
	simTime = 0 #time since the simulation has started
	maxTime = 15 #the amount of time allowed before the simulation stops

	def __init__(self, testNum = -1): #test num -> attempt to make trials more similar to eliminate luck from selection; -1 is random 

		'''Create an new simulation environment with a random robot position and a target someplace in front of the robot
		'''
		self.rightMotor = MotorModel()
		self.leftMotor = MotorModel()

		#generate a random position for the robot
		robotX = random.uniform(self.robotPosRange[0], self.robotPosRange[1])
		robotY = random.uniform(self.robotPosRange[0], self.robotPosRange[1])
		#generate a random heading for the robot
		robotTheta = random.uniform(-math.pi, math.pi)
		#save to instance variable
		self.robotPosAbsolute = (robotX, robotY, robotTheta)
		self.robotStartPosAbsolute = (robotX, robotY, robotTheta)

		#create polar coordinates for the target's position
		targetAngle = math.pi

		if testNum == -1: #generate a completely random map
			#make sure that the target is not exactly horizontal as it will throw off collision detection
			while targetAngle == math.pi or targetAngle == 2*math.pi or targetAngle == -math.pi: 
				targetAngle = robotTheta + random.uniform(self.targetRelAngleRange[0], self.targetRelAngleRange[1])

			robotToTargetDistance = random.uniform(self.targetRelDistanceRange[0], self.targetRelDistanceRange[1])
		elif testNum == 0:
			while targetAngle == math.pi or targetAngle == 2*math.pi or targetAngle == -math.pi:
				targetAngle = self.targetRelAngleRange[0] + random.gauss(0, 0.2)
			robotToTargetDistance = 65 #+ random.gauss(0, 5)
		elif testNum == 1:
			while targetAngle == math.pi or targetAngle == 2*math.pi or targetAngle == -math.pi:
				targetAngle = 0 + random.gauss(0, 0.2)
			robotToTargetDistance = 65 #+ random.gauss(0, 5)
		elif testNum == 2:
			while targetAngle == math.pi or targetAngle == 2*math.pi or targetAngle == -math.pi:
				targetAngle = self.targetRelAngleRange[0] + random.gauss(0, 0.2)
			robotToTargetDistance = 65 #+ random.gauss(0, 5)


		#convert to absolute cartesian coords
		targetX = robotX + robotToTargetDistance * math.cos(targetAngle)
		targetY = robotY + robotToTargetDistance * math.sin(targetAngle)

		#generate a valid angle for the target, then add math.pi so the target faces toward the robot
		targetTheta = robotTheta + random.uniform(self.targetRelThetaRange[0], self.targetRelThetaRange[1]) + math.pi
		self.targetPosAbsolute = (targetX, targetY, targetTheta)

		#wall collision detection points
		self.wallAngle = self.targetPosAbsolute[2] + (math.pi/2) #target's heading is offset from its wall by pi/2 rad
		self.wallSlope = math.tan(self.wallAngle)

		#construct a point 5in in behind the target to figure out which side of the line collision should be detected for
		self.wallTestPoint = (self.targetPosAbsolute[0] - math.cos(self.targetPosAbsolute[2]) * 5, self.targetPosAbsolute[1] - math.sin(self.targetPosAbsolute[2]) * 5)

		#determine if the "solid" part of the line is above or below it
		self.aboveLine = self.wallTestPoint[1] > self.wallSlope * (self.wallTestPoint[0] - self.targetPosAbsolute[0]) + self.targetPosAbsolute[1]


	def renderSimulation(self):
		'''render a simulation window
		'''

		plt.plot(self.robotPosAbsolute[0], self.robotPosAbsolute[1], color='k', marker="o")
		startRobotLine = (self.robotPosAbsolute[0] + self.simLineLength * math.cos(self.robotPosAbsolute[2]), self.robotPosAbsolute[1] + self.simLineLength * math.sin(self.robotPosAbsolute[2]))
		plt.plot([startRobotLine[0], self.robotPosAbsolute[0]], [startRobotLine[1], self.robotPosAbsolute[1]], color='k')

		plt.plot(self.targetPosAbsolute[0], self.targetPosAbsolute[1], color='g', marker="o")
		startTargetLine = (self.targetPosAbsolute[0] + self.simLineLength * math.cos(self.targetPosAbsolute[2]), self.targetPosAbsolute[1] + self.simLineLength * math.sin(self.targetPosAbsolute[2]))
		plt.plot([startTargetLine[0], self.targetPosAbsolute[0]], [startTargetLine[1], self.targetPosAbsolute[1]], color='g')
		
		print("neurons: {}".format(self.getInputActivations()))

		plt.xlabel('x')
		plt.ylabel('y')
		plt.ylim(-150, 150)
		plt.xlim(-150, 150)
		plt.title('sim')
		plt.legend()
		plt.show()


	def update(self, rightPercentOutput, leftPercentOutput):
		''' takes new motor commands and updates the simulation given the new information 
			rightMotor: a value from 0 to 1 indicating the voltage (around 0-9v) going to the right drive motor
			leftMotor: a value from 0 to 1 indicating the voltage (around 0-9v) going to the left drive motor
		'''
		dt = random.uniform(self.dtRange[0], self.dtRange[1])

		rightPercentOutput = self.capActivation(rightPercentOutput)
		leftPercentOutput = self.capActivation(leftPercentOutput)

		rightVel = self.rightMotor.getSpeedApprox(rightPercentOutput)

		leftVel = self.leftMotor.getSpeedApprox(leftPercentOutput)

		#calculate new heading based on new wheel velocities
		heading = self.robotPosAbsolute[2] + ((leftVel - rightVel) / self.robotWheelBase) * dt

		#convert heading to a value from -pi and pi
		while (heading > math.pi):
			heading -= 2 * math.pi

		while (heading <= -math.pi):
			heading += 2 * math.pi

		xPos = self.robotPosAbsolute[0] + (leftVel + rightVel)/2 * math.cos(heading) * dt;
		yPos = self.robotPosAbsolute[1] + (leftVel + rightVel)/2 * math.sin(heading) * dt;

		self.robotPosAbsolute = (xPos, yPos, heading)

		self.simTime += dt


	def getInputActivations(self):
		''' create new input neuron values given the new situation
		'''

		#convert the target to robot-centric coordinates
		# targetRoboCentric = (self.targetPosAbsolute[0] - self.robotPosAbsolute[1], self.targetPosAbsolute[1] - self.robotPosAbsolute[1])

		# targetRoboCentric = (targetRoboCentric[0] * math.cos(self.robotPosAbsolute[2]) - targetRoboCentric[1] * math.sin(self.robotPosAbsolute[2]), targetRoboCentric[0] * math.sin(self.robotPosAbsolute[2]) + targetRoboCentric[1] * math.cos(self.robotPosAbsolute[2]))

		# targetPositiveRoboCentricActivations = (self.capActivation(-targetRoboCentric[0]/100.0), self.capActivation(-targetRoboCentric[1]/100.0))
		# targetNegativeRoboCentricActivations = (self.capActivation(-targetRoboCentric[0]/100.0), self.capActivation(-targetRoboCentric[1]/100.0))


		# robotHeadingTruth = self.robotPosAbsolute[2]
		# robotHeadingActivation = (robotHeadingTruth + math.pi + random.gauss(0, 0.02)) / (2 * math.pi)
		# robotHeadingActivation = self.capActivation(robotHeadingActivation)
		
		# targetHeadingTruth = self.targetPosAbsolute[2] + math.pi

		# while(targetHeadingTruth > math.pi):
		# 	targetHeadingTruth -= 2 * math.pi
		# while(targetHeadingTruth > math.pi):
		# 	targetHeadingTruth += 2 * math.pi

		# targetHeadingActivation = (targetHeadingTruth + random.gauss(0, 0.02)) / (2 * math.pi)
		# targetHeadingActivation = self.capActivation(targetHeadingActivation)

		# rightVelActivation = self.capActivation(self.rightMotor.getSpeed() / self.rightMotor.maxSpeed)
		# leftVelActivation = self.capActivation(self.leftMotor.getSpeed() / self.leftMotor.maxSpeed)

		targetRoboCentric = (self.targetPosAbsolute[0] - self.robotPosAbsolute[0], self.targetPosAbsolute[1] - self.robotPosAbsolute[1])

		targetRoboCentric = (targetRoboCentric[0] * math.cos(self.robotPosAbsolute[2]) - targetRoboCentric[1] * math.sin(self.robotPosAbsolute[2]), targetRoboCentric[0] * math.sin(self.robotPosAbsolute[2]) + targetRoboCentric[1] * math.cos(self.robotPosAbsolute[2]))


		headingDiff = (self.targetPosAbsolute[2] + math.pi) - self.robotPosAbsolute[2] #add pi because the robot and target are facing opposite directions

		#convert heading difference to a value from -pi and pi
		while (headingDiff > math.pi):
			headingDiff -= 2 * math.pi

		while (headingDiff <= -math.pi):
			headingDiff += 2 * math.pi

		headingReq = math.atan2(self.targetPosAbsolute[1] - self.robotPosAbsolute[1], self.targetPosAbsolute[0] - self.robotPosAbsolute[0]) - self.robotPosAbsolute[2] #add pi because the robot and target are facing opposite directions

		#convert heading difference to a value from -pi and pi
		while (headingReq > math.pi):
			headingReq -= 2 * math.pi

		while (headingReq <= -math.pi):
			headingReq += 2 * math.pi


		distance = math.hypot(self.robotPosAbsolute[0] - self.targetPosAbsolute[0], self.robotPosAbsolute[1] - self.targetPosAbsolute[1])

		return [headingDiff, headingReq, targetRoboCentric[0], targetRoboCentric[1], distance]


	def capActivation(self, activation):
		''' caps an input neuron activation between 0 and 1
		'''
		if activation > 1:
			return 1
		if activation < 0:
			return 0

		return activation



	def getFitness(self):
		''' describes the fitness of a given situation
			higher if: closer to target, collides with target, same angle as target, faster
		'''
		fitness = 0

		if (self.noMovement()):
			return -10000 #if the robot does not move, give it a very low fitness

		# dot = (self.robotStartPosAbsolute[0] - self.targetPosAbsolute[0]) * (self.robotStartPosAbsolute[0] - self.robotPosAbsolute[0]) + (self.robotStartPosAbsolute[1] - self.targetPosAbsolute[1]) * (self.robotStartPosAbsolute[1] - self.robotPosAbsolute[1])

		# dot = dot / math.hypot(self.robotStartPosAbsolute[0] - self.targetPosAbsolute[0], self.robotStartPosAbsolute[1] - self.targetPosAbsolute[1])

		# # print("dot {}".format(dot))
		# fitness += dot

		distance = math.hypot(self.robotPosAbsolute[0] - self.targetPosAbsolute[0], self.robotPosAbsolute[1] - self.targetPosAbsolute[1])
		fitness -= distance

		if distance < 7:
			fitness += 500 
			if self.rightMotor.getSpeed() < 0.01 and self.leftMotor.getSpeed() < 0.01:
				fitness += 250 #robot should stop once it reaches the target  
					#convert heading difference to a value from -pi and pi
			
				headingDiff = self.targetPosAbsolute[2] - self.robotPosAbsolute[2] #don't add pi because we want on target to be the largest value

				while (headingDiff > math.pi):
					headingDiff -= 2 * math.pi

				while (headingDiff <= -math.pi):
					headingDiff += 2 * math.pi

				headingDiff = abs(headingDiff)**4 #largest when the robot is exactly on target

				fitness += headingDiff

		# if self.didCollideWithTarget():
		# 	fitness += 100

		# fitness = self.robotStartPosAbsolute[0] - self.robotPosAbsolute[0]

		return fitness


	def didCollideWithTarget(self):
		#create two points on the two front corners of the robot
		robotTestPoint1 = (self.robotPosAbsolute[0] + (math.sin(self.robotPosAbsolute[2]) + math.cos(self.robotPosAbsolute[2])) * self.robotWheelBase/2, self.robotPosAbsolute[1] + (math.sin(self.robotPosAbsolute[2]) - math.cos(self.robotPosAbsolute[2])) * self.robotWheelBase/2)
		robotTestPoint2 = (self.robotPosAbsolute[0] - (math.sin(self.robotPosAbsolute[2]) - math.cos(self.robotPosAbsolute[2])) * self.robotWheelBase/2, self.robotPosAbsolute[1] + (math.sin(self.robotPosAbsolute[2]) + math.cos(self.robotPosAbsolute[2])) * self.robotWheelBase/2)



		#check for collision accordingly
		if self.aboveLine:
			return (robotTestPoint1[1] >= self.wallSlope * (robotTestPoint1[0] - self.targetPosAbsolute[0]) + self.targetPosAbsolute[1]) or (robotTestPoint2[1] >= self.wallSlope * (robotTestPoint2[0] - self.targetPosAbsolute[0]) + self.targetPosAbsolute[1])
		else:
			return (robotTestPoint1[1] <= self.wallSlope * (robotTestPoint1[0] - self.targetPosAbsolute[0]) + self.targetPosAbsolute[1]) or (robotTestPoint2[1] <= self.wallSlope * (robotTestPoint2[0] - self.targetPosAbsolute[0]) + self.targetPosAbsolute[1])


	def noMovement(self):
		if math.hypot(self.robotStartPosAbsolute[0] - self.robotPosAbsolute[0], self.robotStartPosAbsolute[1] - self.robotPosAbsolute[1]) < 1:
			return True
		return False


	def shouldContinue(self):
		'''describes if the simulation should continue based on the actions of the robot
		'''
		if (self.noMovement() and self.simTime > 0.5):
			return False
		#return not (self.didCollideWithTarget() or self.simTime > self.maxTime)
		return not self.simTime > self.maxTime
