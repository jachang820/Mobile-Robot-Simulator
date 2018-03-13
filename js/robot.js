class Position {
	constructor(x=0, y=0) {
		this.x = x;
		this.y = y;
	}

	get pos() {
		return this;
	}

	set pos(position) {
		this.x = position.x;
		this.y = position.y;
	}
}


class State {
	constructor(x=0, y=0, theta=0, v=0) {
		this.update(x, y, theta, v);
	}

	get pos() {
		return new Position(this.x, this.y);
	}

	set pos(position) {
		this.x = position.x;
		this.y = position.y;
	}

	update(x, y, theta, v=0) {
		this.x = x;
		this.y = y;
		this.theta = theta;
		this.v = v;
	}
}


class Robot {

	static get WIDTH() { return 9.0; }
	static get LENGTH() { return 10.5; }
	static get AXLE_LENGTH() { return 8.4; }
	static get WHEEL_DIAMETER() { return 5.0; }
	static get WHEEL_CIRCUMFERENCE() { return 15.3938; }
	static get IMU() { return new Position(0.0, 0.0); }
	static get FRONT_LASER() { return new Position(0.0, 2.5); }
	static get RIGHT_LASER() { return new Position(3.0, -2.5); }
	static get LEFT_WHEEL() { return new Position(-4.2, 0.0); }
	static get RIGHT_WHEEL() { return new Position(4.2, 0.0); }
	static get FRONT_LEFT() { return new Position(-4.2, 3.7); }
	static get FRONT_RIGHT() { return new Position(4.2, 3.7); }
	static get BACK_LEFT() { return new Position(-4.2, -7.1); }
	static get BACK_RIGHT() { return new Position(4.2, -7.1); }
	static get LASER_MAX_RANGE() { return 150.0; }
	static get MAX_RPM() { return 100.0; }
	static get MAX_CM_PER_MS() { return (Robot.MAX_RPM/60000) * Robot.WHEEL_CIRCUMFERENCE; }
	static get ANALOG_MAX() { return 255; }

	
	static rotate(rel_pos, dtheta) {
		var cos = Math.cos(dtheta);
		var sin = Math.sin(dtheta);

		return new Position(
			rel_pos.x * cos - rel_pos.y * sin,
			rel_pos.x * sin + rel_pos.y * cos
		);
	}


	static translate(pos, dx, dy) {
		return new Position(
			pos.x + dx,
			pos.y + dy
		);
	}


	static transform(rel_pos, dstate) {
		return Robot.translate(
			Robot.rotate(rel_pos, dstate.theta), 
			dstate.x, 
			dstate.y
		);
	}


	static next_orientation(state, arc_length, radius, dtheta) {
		var cos = Math.cos(state.theta);
		var sin = Math.sin(state.theta);
		var cos_delta = Math.cos(state.theta + dtheta);
		var sin_delta = Math.sin(state.theta + dtheta);

		var x;
		var y;

		if (radius < 200) {
			x = radius*(cos_delta - cos) + state.x;
			y = radius*(sin_delta - sin) + state.y;
		} else {
			x = state.x - arc_length * sin;
			y = state.y + arc_length * cos;
		}

		return new State(x, y, Robot.reduce_angle(state.theta + dtheta));
		
	}


	static reduce_angle(theta) {
		var angle = theta % (2*Math.PI);
		if (angle < -Math.PI) {
			angle += 2*Math.PI;
		} else if (angle > Math.PI) {
			angle -= 2*Math.PI;
		}
		return angle;
	}


	static quadrant(theta) {
		var quad = Math.floor(Robot.reduce_angle(theta)/(Math.PI/4)) + 2;
		var quad_map = [4, 1, 2, 3];
		return quad_map[quad];
	}


	static distance_from_point(state, box) {
		const EPS = 0.1;
		var cos = Math.cos(state.theta);
		var sin = Math.sin(state.theta);

		var vert_wall = (state.theta < 0 ? box.length_cm : 0);
		var hori_wall = (Math.abs(state.theta) < Math.PI/2 ? box.width_cm : 0);

		var vert_dist;
		var hori_dist;

		if (Math.abs(sin) > EPS) {
			vert_dist = Math.abs((vert_wall - state.x) / sin);
		} else {
			vert_dist = Infinity;
		}

		if (Math.abs(cos) > EPS) {
			hori_dist = Math.abs((hori_wall - state.y) / cos);
		} else {
			hori_dist = Infinity;
		}
		
		var quad = Robot.quadrant(state.theta);
		var min_dist;
		var nearest_wall;
		var proj_pos;

		if (vert_dist < hori_dist) {
			min_dist = vert_dist;
			nearest_wall = (quad == 2 || quad == 3 ? "left" : "right");
			proj_pos = state.y + min_dist * cos;
		} else {
			min_dist = hori_dist;
			nearest_wall = (quad == 1 || quad == 2 ? "top" : "bottom");
			proj_pos = state.x - min_dist * sin;
		}

		return { dist: min_dist, 
						 wall: nearest_wall, 
						 pos: proj_pos };

	}


	static random_angle() {
		return Math.random() * 2.0 * Math.PI - Math.PI;
	}


	static random_float(min, max) {
		return Math.random() * (max - min) + min;
	}


	static gaussian(mean=0, variance=1) {
		// central limit theorem
		var sum = 0;
		var n = Math.sqrt(12*variance);
		for (var i = 0; i < 10; i += 1) {

			sum += Math.random()*n - (n/2 - mean);

		}
		return sum/10.0;
	}


	constructor(startState=null, isPrev=false) {
		this.state = startState || new State();
		this.period = 5.0;
		
		this.imu = { mag: 0, gyro: 0, accel: 0 };
		this.front_laser = { dist: 0 };
		this.right_laser = { dist: 0 };
		this.left_wheel = {}
		this.right_wheel = {}
		this.front_left = {}
		this.front_right = {}
		this.back_left = {}
		this.back_right = {}

		this.update_positions();

		this.left_signal = false;
		this.right_signal = false;

		if (isPrev == false) {
			this.prev = new Robot(self.state, true);
		}
	}


	update_positions() {
		this.imu.pos = Robot.transform(Robot.IMU, this.state);
		this.front_laser.pos = Robot.transform(Robot.FRONT_LASER, this.state);
		this.right_laser.pos = Robot.transform(Robot.RIGHT_LASER, this.state);
		this.left_wheel.pos = Robot.transform(Robot.LEFT_WHEEL, this.state);
		this.right_wheel.pos = Robot.transform(Robot.RIGHT_WHEEL, this.state);
		this.front_left.pos = Robot.transform(Robot.FRONT_LEFT, this.state);
		this.front_right.pos = Robot.transform(Robot.FRONT_RIGHT, this.state);
		this.back_left.pos = Robot.transform(Robot.BACK_LEFT, this.state);
		this.back_right.pos = Robot.transform(Robot.BACK_RIGHT, this.state);
	}
}
