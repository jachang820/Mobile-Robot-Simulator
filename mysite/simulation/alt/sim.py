import asyncio
import math
import random
from math import pi
from robot import Robot, Position, State, Box
from interface import HTTP_Interface

Sandbox = Box(80.0, 100.0)

BUFFER = 9.0
TRUE_NORTH = round(random.uniform(-pi, pi), 2)

class SimulatedRobot(Robot):

	# Generates random legal starting state
	def random_state(length, width, buffer):
		theta = random.uniform(-pi, pi)
		
		# robot extremities in robot frame
		fl = Robot.rotate(Robot.FRONT_LEFT, theta)
		fr = Robot.rotate(Robot.FRONT_RIGHT, theta)
		bl = Robot.rotate(Robot.BACK_LEFT, theta)
		br = Robot.rotate(Robot.BACK_RIGHT, theta)
		robot_x = [fl.x, fr.x, bl.x, br.x]
		robot_y = [fl.y, fr.y, bl.y, br.y]

		# robot extremities in field frame
		min_x = min(robot_x)
		max_x = max(robot_x)
		min_y = min(robot_y)
		max_y = max(robot_y)

		return State(
			random.uniform(buffer - min_x, length - max_x - buffer),
			random.uniform(buffer - min_y, width - max_y - buffer),
			theta
		)


	def add_noise(input, ratio):
		return input + random.gauss(0, 1) * input * ratio


	def __init__(self):
		super().__init__(SimulatedRobot.random_state(Sandbox.length, Sandbox.width, BUFFER))
		self._period = 5.0
		self._active = True


	def state_simulator(self, left, right):
		# input sequence
		left_ratio = left / Robot.ANALOG_MAX
		right_ratio = right / Robot.ANALOG_MAX

		for step in range(len(left)):

			ratio_diff = right_ratio[step] - left_ratio[step]

			max_dist = Robot.MAX_CM_PER_MS * self._period

			avg_ratio = (left_ratio + right_ratio) / 2;

			dist = avg_ratio * max_dist;

			turn_radius = left_ratio[step] * Robot.AXLE_LENGTH / ratio_diff

			d_theta = ratio_diff * max_dist / Robot.AXLE_LENGTH

			# set robot as axis of rotation to find position of center
			if abs(turn_radius) < Sandbox.width / 2:
				rel_pos = Position( -(turn_radius + Robot.AXLE_LENGTH / 2), 0)
				center = Robot.transform(rel_pos, self._state)

				# travel along circumference of circle
				rel_pos = Position(self._state.x - center.x,
													self._state.y - center.y)
				d_state = State(center.x, center.y, d_theta)

			# straight path
			else:

				# set original state as axis of rotation
				rel_pos = Position(0, avg_dist)
				d_state = self._state
				
			# update state
			cur_pos = Robot.transform(rel_pos, d_state)
			self._prev._state = self._state
			self._state = State(cur_pos.x, 
													cur_pos.y, 
													self._state.theta + d_theta,
													avg_dist)



	def get_front_dist(self):
		self._prev._front_laser['dist'] = self._front_laser['dist']
		self._front_laser['dist'] = Robot.distance_from_point(
				State(self._front_laser['pos'].x,
							self._front_laser['pos'].y,
							self._state.theta))[0]
		return self._front_laser['dist']

		
	def get_right_dist(self):
		self._prev._right_laser['dist'] = self._right_laser['dist']
		self._right_laser['dist'] = Robot.distance_from_point(
				State(self._right_laser['pos'].x,
							self._right_laser['pos'].y, 
							self._state.theta - pi/2))[0]
		return self._right_laser['dist']


	def get_mag(self):
		self._prev._imu['mag'] = self._imu['mag']
		self._imu['mag'] = self._state.theta - TRUE_NORTH
		return self._imu['mag']


	def get_gyro(self): # scaled to cm/s
		self._prev._imu['gyro'] = self._imu['gyro']
		self._imu['gyro'] = (self.imu['mag'] - self._prev.imu['mag']) * \
			1000 / self._period
		return self._imu['gyro']


	def get_accel(self): # scaled to cm/s
		self._prev._imu['accel'] = self._imu['accel']
		self._imu['accel'] = (self._state.vel - self._prev._state.vel) * \
			1000 / self._period
		return self._imu['accel']


	# Returns a dictionary of sensors
	def update_sensors(self):
		return {
			"front": SimulatedRobot.add_noise(self.get_front_dist(), 0.05),
			"right": SimulatedRobot.add_noise(self.get_right_dist(), 0.05),
			"mag": SimulatedRobot.add_noise(self.get_mag(), 0.05),
			"gyro": SimulatedRobot.add_noise(self.get_gyro(), 0.05),
			"accel": SimulatedRobot.add_noise(self.get_accel(), 0.05)
		}


async def simulate_loop(sim, loop=None):

	while True:
		# break if turned off
		if sim._active == False:

			# undraw
			
			break
		print("what")
		# check for inputs
		ctrl_str = HTTP_Interface.receive_controls()
		controls = ctrl_str[6:].split(",")
		left = controls[0::2]
		right = controls[1::2]

		# iterate the state
		sim.state_simulator(left, right)

		# recalculate simulated robot positions
		sim.update_positions()

		# simulate HTTP response to send sensor data
		HTTP_Interface.send_sensor_data(sim.update_sensors())

		# draw

		# let other tasks run
		await asyncio.sleep(sim._period/1000.0)


def schedule_simulator(loop=None):
	print("bye")
	# create simulation
	sim = SimulatedRobot()
	if loop is None:
		loop = asyncio.get_event_loop()
	sim._loop = loop
	print("hi")
	# schedule task loop
	sim._task = asyncio.ensure_future(simulate_loop(sim, loop))
	return sim

def stop_simulator(sim, loop):
	sim._active = False
	sim._task.cancel()
