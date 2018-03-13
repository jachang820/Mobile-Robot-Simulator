import math
from math import pi


class Box:
	def __init__(self, width=0, length=0):
		self._length = length
		self._width = width

	@property
	def length(self):
		return self._length

	@length.setter
	def length(self, val):
		self._length = val

	@property
	def width(self):
		return self._width

	@width.setter
	def width(self, val):
		self._width = val


class Position:
	def __init__(self, x=0, y=0):
		self._x = x
		self._y = y

	@property
	def x(self):
		return self._x

	@x.setter
	def x(self, val):
		self._x = val

	@property
	def y(self):
		return self._y

	@y.setter
	def y(self, val):
		self._y = val

	@property
	def pos(self):
		return Position(self._x, self._y)

	@pos.setter
	def pos(self, val):
		self._x = val.x
		self._y = val.y


class State:
	def __init__(self, x=0, y=0, theta=0, vel=0):
		self._x = x
		self._y = y
		self._theta = theta
		self._vel = vel

	@property
	def x(self):
		return self._x

	@x.setter
	def x(self, val):
		self._x = val

	@property
	def y(self):
		return self._y

	@y.setter
	def y(self, val):
		self._y = val

	@property
	def theta(self):
		return self._theta

	@theta.setter
	def theta(self, val):
		self._theta = val

	@property
	def pos(self):
		return Position(self._x, self._y)

	@pos.setter
	def pos(self, val):
		self._x = val.x
		self._y = val.y

	@property
	def vel(self):
		return self._vel

	@vel.setter
	def vel(self, val):
		self._vel = val


class Robot:
	# class variables
	WIDTH = 9.0
	LENGTH = 10.5
	AXLE_LENGTH = 8.4
	WHEEL_DIAMETER = 5.0
	WHEEL_CIRCUMFERENCE = 15.3938
	IMU = Position(0.0, 0.0)
	FRONT_LASER = Position(0.0, 2.5)
	RIGHT_LASER = Position(3.0, -2.5)
	LEFT_WHEEL = Position(-4.2, 0.0)
	RIGHT_WHEEL = Position(4.2, 0.0)
	FRONT_LEFT = Position(-4.2, 3.7)
	FRONT_RIGHT = Position(4.2, 3.7)
	BACK_LEFT = Position(-4.2, -7.1)
	BACK_RIGHT = Position(4.2, -7.1)
	LASER_MAX_RANGE = 150.0
	MAX_RPM = 100
	MAX_CM_PER_MS = (MAX_RPM/60000) * WHEEL_CIRCUMFERENCE
	ANALOG_MAX = 255

	# Rotates position about axis by angle counterclockwise.
	def rotate(rel_pos, d_theta):
		cos = math.cos(d_theta)
		sin = math.sin(d_theta)
		return Position(
			rel_pos.x * cos - rel_pos.y * sin,
			rel_pos.x * sin + rel_pos.y * cos
		)

	# Translates a position in field frame.
	def translate(pos, dx, dy):
		return Position(
			pos.x + dx,
			pos.y + dy
		)

	# Rotates and translates in field frame.
	def transform(rel_pos, d_state):
		pos = Robot.rotate(rel_pos, d_state.theta)
		pos = Robot.translate(pos, d_state.x, d_state.y)
		return pos

	# Keep angle between -pi and pi
	def reduce_angle(theta):
		return (theta % (2*pi)) - (2*pi)


	# Quadrant of angle
	def quadrant(theta):
		quad = int(round(Robot.reduce_angle(theta) / (pi/4))) + 2
		quad_map = [4, 1, 2, 3]
		return quad_map[quad]


	# Distance from a point in one direction
	# [distance to wall, wall projected on, point on wall projected on]
	def distance_from_point(state):
		eps = 0.1 # eliminate large calculations
		cos = math.cos(state.theta)
		sin = math.sin(state.theta)
		abs_theta = abs(state.theta)

		# walls that the robot might be facing
		vert_wall = Box.length if state.theta < 0 else 0
		hori_wall = Box.width if abs_theta < pi/2 else 0

		# distance to each relevant wall
		if abs(sin) > eps:
			vert_dist = abs(vert_wall - state.x) / sin
		else:
			vert_dist = math.inf

		if abs(cos) > eps:
			hori_dist = abs(hori_wall - state.y) / cos
		else:
			hori_dist = math.inf

		quad = Robot.quadrant(state.theta)
		
		# laser projects on side walls
		if vert_dist < hori_dist:
			min_dist = vert_dist
			nearest_wall = "left" if quad == 2 or quad == 3 \
				else "right"
			proj_pos = state.y + min_dist * cos

		# laser projects on top or bottom wall
		else:
			min_dist = hori_dist
			nearest_wall = "top" if quad == 1 or quadrant == 2 \
				else "bottom"
			proj_pos = state.x - min_dist * sin

		return [min_dist, nearest_wall, proj_pos]


	def __init__(self, startState=None, isPrev=False):
		
		# state of robot
		self._state = startState or State()
		self._period = 5.0
		self._loop = None
		self._task = None

		# parts of the robot
		self._imu = {"mag": 0, "gyro": 0, "accel": 0}
		self._front_laser = {"dist": 0}
		self._right_laser = {"dist": 0}
		self._left_wheel = {}
		self._right_wheel = {}
		self._front_left = {}
		self._front_right = {}
		self._back_left = {}
		self._back_right = {}

		# save positions relative to field
		self.update_positions()

		# input
		self._left_signal = False
		self._right_signal = False

		# previous state
		if isPrev == False:
			self._prev = Robot(self._state, True)


	def update_positions(self):
		self._imu['pos'] = Robot.transform(Robot.IMU, self._state)
		self._front_laser['pos'] = Robot.transform(Robot.FRONT_LASER, self._state)
		self._right_laser['pos'] = Robot.transform(Robot.RIGHT_LASER, self._state)
		self._left_wheel['pos'] = Robot.transform(Robot.LEFT_WHEEL, self._state)
		self._right_wheel['pos'] = Robot.transform(Robot.RIGHT_WHEEL, self._state)
		self._front_left['pos'] = Robot.transform(Robot.FRONT_LEFT, self._state)
		self._front_right['pos'] = Robot.transform(Robot.FRONT_RIGHT, self._state)
		self._back_left['pos'] = Robot.transform(Robot.BACK_LEFT, self._state)
		self._back_right['pos'] = Robot.transform(Robot.BACK_RIGHT, self._state)


