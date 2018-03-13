import asyncio
from math import pi
from robot import Robot, Position, State, Box
from interface import HTTP_Interface, FrontEnd_Interface
from sim import SimulatedRobot, schedule_simulator, stop_simulator
from operator import add
import numpy as np


class RobotState(Robot):

	def __init__(self):
		super().__init__()
		self._period = 50.0
		self._calibrated = False

		# control option
		self._manual = True
		
		# inputs
		self._analog_left = 0
		self._analog_right = 0

		# estimate of box size and angle
		self._field = { "width" : 0,
										"length" : 0,
										"orient" : 0 }

		# kalman filter initial settings		
		# state covariance matrix
		self._P = [ [1, 0, 0, 0],
								[0, 1, 0, 0],
								[0, 0, 1, 0],
								[0, 0, 0, 1] ]

		# input covariance matrix
		self._Gamma = [ [0.00101, 0       ],
										[0      , 0.00257 ] ]
		self._Gamma = np.multiply(self._Gamma, self._period)

		# the following are to be determined through calibration
		# observation error covariance matrix
		self._R = []

		# process error covariance matrix
		self._Q = []


	def mirror_state(self):
		self._state.x = self._field['length'] - self._state.x
		self._state.y = self._field['width'] - self._state.y
		self._field.orient = Robot.reduce_angle(self._field._orient - pi)
		self._state.theta = Robot.reduce_angle(self._state.theta - pi)
		# draw
	

	def accelerate(self, cap=255):

		# retrieve input
		inputs = FrontEnd_Interface.recieve_inputs()

		# gradually change speed
		decel = lambda analog : \
			analog - math.copysign(25, analog) if abs(analog) > 25 else 0
		accel = lambda analog, input : analog + 40 * input

		# determine left analog
		self._analog_left = decel(self._analog_left) + \
			accel(self._analog_left, inputs['left'])
		if abs(self._analog_left) > cap:
			self._analog_left = math.copysign(cap, self._analog_left)

		# determine right analog
		self._analog_right = decel(self._analog_right) + \
			accel(self._analog_right, inputs['right'])
		if abs(self._analog_right) > cap:
			self._analog_right = math.copysign(cap, self._analog_right)


	def calibrate(self):

		FrontEnd_Interface.set_flag("calibrate")



	def update_sensors(self, sensors):
		
		# get new sensor data from robot
		self._front_laser['dist'] = sensors['front']
		self._right_laser['dist'] = sensors['right']
		self._imu['mag'] = sensors['mag']
		self._imu['gyro'] = sensors['gyro']
		self._imu['accel'] = sensors['accel']


	def estimate_state(self):

		# shorten variable names
		x_prev = self._state.x
		y_prev = self._state.y
		v_prev = self._state.vel
		theta_prev = self._state.theta
		sin = math.sin(theta_prev)
		cos = math.cos(theta_prev)
		sin_delta = math.sin(theta_prev + dtheta)
		cos_delta = math.cos(theta_prev + dtheta)
		dtheta = self._imu['gyro'] * self._period / 1000
		a = self._imu['accel'] * self._period / 1000

		# estimate arc
		arc_length = (v_prev + a) / 2

		# robot is turning
		if dtheta > 0.1:

			# turning radius
			r = arc_length / dtheta

			# estimated state
			x = r*(cos_delta - cos) + x_prev
			y = r*(sin_delta - sin) + y_prev

			# state and input matrices
			A = [ [1, 0, r*(sin - sin_delta), (cos_delta - cos)/(2*dtheta)],
						[0, 1, r*(cos_delta - cos), (sin_delta - sin)/(2*dtheta)],
						[0, 0, 1, 0],
						[0, 0, 0, 1] ]
			B = [ [-r*((cos_delta - cos)/dtheta + sin_delta), (cos_delta - cos)/(2*dtheta)],
						[-r*((sin_delta - sin)/dtheta - cos_delta), (sin_delta - sin)/(2*dtheta)],
						[1, 0],
						[0, 1] ]

		# straight path
		else:

			# estimated state
			x = x_prev - arc_length * sin
			y = y_prev + arc_length * cos

			# state and input matrices
			A = [ [1, 0, 0, -cos / 2],
						[0, 1, 0, sin / 2],
						[0, 0, 1, 0],
						[0, 0, 0, 1] ]
			B = [ [0, -cos / 2], 
						[0, sin / 2],
						[0, 1], 
						[1, 0] ]

		# process error, w
		w = [ random.gauss(0, self._Q[0][0] ** 0.5),
					random.gauss(0, self._Q[1][1] ** 0.5),
					random.gauss(0, self._Q[2][2] ** 0.5),
					random.gauss(0, self._Q[3][3] ** 0.5) ]

		# estimated state, x_hat
		return { 
			"x_hat" : State(x + w[0],
											y + w[1],
											theta_prev + dtheta + w[2],
											v_prev + a + w[3] ),
			"A" : A, # Jacobian of f/x_prev
			"B" : B  # Jacobian of f/u
		} 



	def measure_state(self, x_hat):

		# shorter variables
		theta = self._imu['mag']
		sin = math.sin(theta)
		cos = math.cos(theta)
		f = self._front_laser['dist'] + Robot.FRONT_LASER.y
		r = self._right_laser['dist'] + Robot.RIGHT_LASER.x + \
				abs(Robot.RIGHT_LASER.y) * sin/cos;
		h = self._field['width']
		w = self._field['length']
		f_pos = self._front_laser['pos']
		r_pos = self._right_laser['pos']

		# use predicted state to get which wall laser projects on
		front_wall = Robot.distance_from_point(State(f_pos.x, f_pos.y, theta))
		right_wall = Robot.distance_from_point(State(r_pos.x, r_pos.y, theta))

		# [x, y, dx, dy]
		estimator = {
			"front" : {
				"right"  : [ f*sin, 0, f*cos, 0 ],
				"top"    : [ 0, h - f*cos, 0, f*sin ],
				"left"   : [ w + f*sin, 0, f*cos, 0 ],
				"bottom" : [ 0, -f*cos, 0, f*sin ]
			},
			"right" : {
				"right"  : [-r*cos, 0, r*sin, 0 ],
				"top"    : [ 0, h - r*sin, 0 -r*cos ],
				"left"   : [ w - r*cos, 0, r*sin, 0 ],
				"bottom" : [ 0, -r*sin, 0, -r*cos ]
			}
		}

		# get estimate from both lasers and average values
		f_est = estimator['front'][front_wall[1]]
		r_est = estimator['right'][right_wall[1]]
		combined = [(f_est[i]+r_est[i])/2 for i, _ in enumerate(f_est)]
		
		# fill in values
		if combined[0] == 0:
			combined[0] = front_wall[2] + f*sin
			combined[2] = f*cos
		if combined[1] == 0:
			combined[1] = front_wall[2] - f*cos
			combined[3] = f*sin

		# observation error, v
		v = [ random.gauss(0, self._R[0][0] ** 0.5),
					random.gauss(0, self._R[1][1] ** 0.5),
					random.gauss(0, self._R[2][2] ** 0.5),
					random.gauss(0, self._R[3][3] ** 0.5) ]

		# innovation, measured state, z
		return {
			"z" : State(combined[0] + v[0],
									combined[1] + v[1],
									theta + v[2],
									x_hat.vel + v[3]),
			"H" : [ [1       , sin*cos, combined[2], 0],
							[-sin*cos, 1      , combined[3], 0],
							[0       , 0      , 1          , 0],
							[0       , 0      , 0          , 1] ]
		}


		def kalman_predict(self):
			
			# state prediction, x := f(x, u, w)
			estimate = self.estimate_state()
			
			# covariance prediction, P := APA^T + BGB^T + Q
			self._prev._P = self._P
			A = np.matmul(estimate['A'], self._P)
			A = np.matmul(A, np.transpose(estimate['A']))
			B = np.matmul(estimate['B'], self._Gamma)
			B = np.matmul(B, np.transpose(estimate['B']))
			self._P = A + B + self._Q

			return estimate


		def kalman_update(self, estimate):

			# state measurement, z := h(x, v)
			measure = self.measure_state(estimate['x_hat'])

			# innovation residual, y := z - x_hat
			y = [ measure['z'].x - estimate['x_hat'].x,
						measure['z'].y - estimate['x_hat'].y,
						measure['z'].theta - estimate['x_hat'].theta,
						measure['z'].vel - estimate['x_hat'].vel ] 

			# innovation covariance, S := HPH^T + R
			S = np.matmal(measure['H'], self._P)
			S = np.matmul(S, np.transpose(measure['H']))
			S += self._R

			# Kalman gain, K = P H^T S^-1
			K = np.matmul(self._P, np.transpose(measure['H']))
			K = np.matmul(K, np.linalg.pinv(S))

			# update state, x := x_hat + Ky
			x = estimate['x_hat'] + np.matmul(K, y)

			# update covariance, P := (I-KH)P
			I = np.identity(4)
			P = np.matmul((I - np.matmul(K, H)), P)

			# save updated state
			self._prev._state = self._state
			self._state = x
			return x


async def state_loop(robot, loop=None):

	sim = None

	while True:
		print("fuck")
		FrontEnd_Interface.listen()
		print("fuck")
		# listen for simulate
		if FrontEnd_Interface.retrieve_options("simulate"):
			if sim == None:
				sim = schedule_simulator(loop)
				robot._state = sim._state #temporary
			else:
				print("hi")
				stop_stimulator(sim, loop)
		print("fuck again")
		# listen for control
		if FrontEnd_Interface.retrieve_options("control"):
			robot._manual = not robot._manual

		# listen for mirror
		if FrontEnd_Interface.retrieve_options("mirror"):
			robot.mirror_state()

		# retrieve sensor data
		sensors = HTTP_Interface.retrieve_sensor_data()
		robot.update_sensors(sensors)

		# Extended Kalman Filter
		predict = robot.kalman_predict()
		robot.kalman_update(predict)

		# draw
		print("Robot(" + robot._state.x + ", " + robot._state.y + ", " + robot._state.theta + ")")
		print("Sim(" + sim._state.x + ", " + sim._state.y + ", " + sim._state.theta + ")")

		# let other tasks run
		await asyncio.sleep(robot._period/1000.0)


def run_robot(loop=None):

	# create simulation
	robot = RobotState()
	if loop is None:
		loop = asyncio.get_event_loop()
	robot._loop = loop

	# schedule task loop
	robot._task = asyncio.ensure_future(state_loop(robot, loop))
	loop.run_forever()
	

if __name__ == "__main__":
	run_robot()