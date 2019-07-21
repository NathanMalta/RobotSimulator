import arcade
import math
import sys
import os
import pickle
from Simulator import Simulator
import neat

class SimulationRenderer(arcade.Window):

	def __init__(self, isTeleop, neuralNet = None):
		super().__init__(500, 500, title="Sim", resizable=False)
		self.isTeleop = isTeleop
		self.neuralNet = neuralNet

		self.robotPos = 0
		self.targetPos = 0

		self.moveRight = 0
		self.moveLeft = 0
		
		self.simulator = Simulator(testNum=0)

		arcade.set_background_color(arcade.color.WHITE)

		self.setup()
		arcade.run()

	def on_draw(self):
		""" Render the screen. """
		arcade.start_render()

		arcade.draw_text("Sim Time: " + str(round(self.simulator.simTime,1)), 500, 500, arcade.color.BLACK, 9, width=130, align="left", anchor_x="right", anchor_y="top", font_name="Monaco")
		arcade.draw_text("Right Motor: " + str(round(self.simulator.rightMotor.getPercentOutput(),2)) + "%", 500, 480, arcade.color.BLACK, 9, width=130, align="left", anchor_x="right", anchor_y="top", font_name="Monaco")
		arcade.draw_text("Left Motor:  " + str(round(self.simulator.leftMotor.getPercentOutput(),2)) + "%", 500, 460, arcade.color.BLACK, 9, width=130, align="left", anchor_x="right", anchor_y="top", font_name="Monaco")

		#draw the robot
		arcade.draw_rectangle_filled(self.simulator.robotPosAbsolute[0] + 150, self.simulator.robotPosAbsolute[1] + 150, #x,y,width,height
								 self.simulator.robotWheelBase * 2, self.simulator.robotWheelBase * 2,
								 arcade.color.GRAY, tilt_angle = math.degrees(self.simulator.robotPosAbsolute[2]))

		arcade.draw_rectangle_filled(self.simulator.robotPosAbsolute[0] + 150 + 10 * math.cos(self.simulator.robotPosAbsolute[2]), self.simulator.robotPosAbsolute[1] + 150 + 10 * math.sin(self.simulator.robotPosAbsolute[2]), #x,y,width,height
								 10, 10,
								 arcade.color.LIGHT_GRAY, tilt_angle = math.degrees(self.simulator.robotPosAbsolute[2]))

		#draw the target
		arcade.draw_rectangle_filled(self.simulator.targetPosAbsolute[0] + 150, self.simulator.targetPosAbsolute[1] + 150, #x,y,width,height
						 self.simulator.robotWheelBase * 2, self.simulator.robotWheelBase * 2,
						 arcade.color.GREEN, tilt_angle = math.degrees(self.simulator.targetPosAbsolute[2]))

		arcade.draw_rectangle_filled(self.simulator.targetPosAbsolute[0] + 150 + 10 * math.cos(self.simulator.targetPosAbsolute[2]), self.simulator.targetPosAbsolute[1] + 150 + 10 * math.sin(self.simulator.targetPosAbsolute[2]), #x,y,width,height
								 10, 10,
								 arcade.color.LIGHT_GREEN, tilt_angle = math.degrees(self.simulator.targetPosAbsolute[2]))



	def on_key_press(self, key, modifiers):
		"""
		Called whenever a key is pressed.
		"""
		if (modifiers == arcade.key.MOD_CTRL or modifiers == arcade.key.MOD_COMMAND) and (key == arcade.key.Q or key == arcade.key.W):
			sys.exit() # make command + Q work so the program can be quit more easily

		if not self.isTeleop: #ignore keypresses if teleop is disabled
			return

		if key == arcade.key.A:
			self.moveRight = 1
			self.moveLeft = 0
		elif key == arcade.key.S:
			self.moveRight = 1
			self.moveLeft = 1
		elif key == arcade.key.D:
			self.moveRight = 0
			self.moveLeft = 1

	def on_key_release(self, key, modifiers):
		"""Called when the user releases a key. """
		if not self.isTeleop: #ignore keypresses if teleop is disabled
			return

		if key == arcade.key.A or key == arcade.key.S or key == arcade.key.D:
			self.moveRight = 0
			self.moveLeft = 0

	def setMovement(self, right, left):
		self.moveRight = right
		self.moveLeft = left
		self.simulator.update(self.moveRight, self.moveLeft)

	def getInputActivations(self):
		return self.simulator.getInputActivations()

	def setup(self):
		# Set up your game here
		pass

	def update(self, delta_time):
		if self.isTeleop: #update automatically when it is teleop mode
			self.simulator.update(self.moveRight, self.moveLeft)
		elif self.neuralNet != None:
			cmd = self.neuralNet.activate(self.getInputActivations())
			self.setMovement(cmd[0], cmd[1])

	def sigmoid(self, x):
		return 1 / (1 + math.exp(-x))

	def stopRender(self):
		arcade.close_window()

	def startTeleop():
		window = SimulationRenderer(True)

	def startAI():
		local_dir = os.path.dirname(__file__)
		with open(local_dir + '/winner-net', 'rb') as f:
			print(f)
			winner_net = pickle.load(f)

		simWindow = SimulationRenderer(False, winner_net)

if __name__ == '__main__':
	#SimulationRenderer.startTeleop()
	SimulationRenderer.startAI()
	
