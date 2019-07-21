

class MotorModel():
	'''simple linear model for proof of concept #TODO: improve
	'''
	maxSpeed = 18 #in/sec max #TODO: guess

	def __init__(self):
		self.currentSpeed = 0
		self.currentPercentOutput = 0

	def getSpeedApprox(self, percentOutput):
		'''converts a percent output (voltage representation between 0 and 1) into a speed
		   with a simple linear approximation
		'''
		speed = percentOutput * self.maxSpeed
		self.setSpeed(speed)
		self.setPercentOutput(percentOutput)
		return speed

	def setSpeed(self, speed):
		self.currentSpeed = speed

	def getSpeed(self):
		return self.currentSpeed

	def setPercentOutput(self, p):
		self.percentOutput = p

	def getPercentOutput(self):
		return self.percentOutput