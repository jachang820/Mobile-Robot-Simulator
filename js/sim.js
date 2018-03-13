const BUFFER = 9.0;
const TRUE_NORTH = 0; //Robot.random_angle();

class Simulation extends Robot {

	static random_state(width, length, buffer) {
		var theta = Robot.random_angle();

		var fl = Robot.rotate(Robot.FRONT_LEFT, theta);
		var fr = Robot.rotate(Robot.FRONT_RIGHT, theta);
		var bl = Robot.rotate(Robot.BACK_LEFT, theta);
		var br = Robot.rotate(Robot.BACK_RIGHT, theta);
		var robot_x = [fl.x, fr.x, bl.x, br.x];
		var robot_y = [fl.y, fr.y, bl.y, br.y];

		var min_x = Math.min(...robot_x);
		var max_x = Math.max(...robot_x);
		var min_y = Math.min(...robot_y);
		var max_y = Math.max(...robot_y);

		var state = new State(
			Robot.random_float(buffer - min_x, length - max_x - buffer),
			Robot.random_float(buffer - min_y, width - max_y - buffer),
			theta,
			0
		);

		return state;
	}


	static add_noise(input, ratio) {
		return input + Robot.gaussian() * input * ratio + Robot.gaussian() * ratio;
	}


	constructor() {
		super(Simulation.random_state(sandbox.width_cm, sandbox.length_cm, BUFFER));
		this.period = 5.0;
		this.active = true;	
	}


	state_simulator(left, right) {
		
		for (var step = 0; step < left.length; step += 1) {

			let left_ratio = left[step] / Robot.ANALOG_MAX;
			let right_ratio = right[step] / Robot.ANALOG_MAX;

			let ratio_diff = right_ratio - left_ratio;
			let ratio_sum = right_ratio + left_ratio;

			let max_dist = Robot.MAX_CM_PER_MS * this.period;
			let dist = ratio_sum * max_dist / 2;
			let turn_radius = Robot.AXLE_LENGTH * ratio_sum / (2 * ratio_diff);
			let dtheta = ratio_diff * max_dist / Robot.AXLE_LENGTH;

			let orient = Robot.next_orientation(this.state, dist, turn_radius, dtheta);
			this.prev.state = this.state;
			this.state.update(orient.x, orient.y, orient.theta, dist);
		}
	}


	get_front_dist() {
		this.prev.front_laser.dist = this.front_laser.dist;
		this.front_laser.dist = Robot.distance_from_point(
			new State(this.front_laser.pos.x,
								this.front_laser.pos.y,
								this.state.theta),
			sandbox).dist;

		return this.front_laser.dist;
	}


	get_right_dist() {
		this.prev.right_laser.dist = this.right_laser.dist;
		this.right_laser.dist = Robot.distance_from_point(
			new State(this.right_laser.pos.x,
								this.right_laser.pos.y,
								Robot.reduce_angle(this.state.theta - Math.PI/2)),
			sandbox).dist;

		return this.right_laser.dist;
	}



	get_mag() {
		this.prev.imu.mag = this.imu.mag;
		this.imu.mag = Robot.reduce_angle(this.state.theta - TRUE_NORTH);
		return this.imu.mag;
	}


	get_gyro() {
		this.prev.imu.gyro = this.imu.gyro;
		this.imu.gyro = Robot.reduce_angle(this.imu.mag - this.prev.imu.mag) * 
										this.period / 1000;
		return this.imu.gyro;
	}


	get_accel() {
		this.prev.imu.accel = this.imu.accel;
		this.imu.accel = (this.state.v - this.prev.state.v) * 
											this.period / 1000;
		return this.imu.accel;
	}


	update_sensors() {
		return {
			front : [Simulation.add_noise(this.get_front_dist(), 0.05)],
			right : [Simulation.add_noise(this.get_right_dist(), 0.05)],
			accel : [Simulation.add_noise(this.get_accel(), 0.05)],
			mag   : [Simulation.add_noise(this.get_mag(), 0.05)],
			gyro  : [Simulation.add_noise(this.get_gyro(), 0.05)],
		};
	}
}


function simulator(simbot) {

	if (simbot.active) {

		// detect new inputs
		var target = http_IF.receive_target();
		var controls = http_IF.receive_controls();

		if (target != null) {
			http_IF.ajax = true;
			console.log("target exists");
			// UNIMPLEMENTED

		// read a list of inputs and react
		} else if (controls != null) {

			controls = controls.slice(5,-1);
			let count = 0;
			let left = [];
			let right = [];

			while (count < controls.length) {

				left.push(parseInt(controls.substr(count, 4)));
				right.push(parseInt(controls.substr(count + 4, 4)));
				count += 8;

			}

			simbot.state_simulator(left, right);
		}

		// update relative parts positions
		simbot.update_positions();

		// update sensors
		http_IF.send_sensor_data(simbot.update_sensors());

		// draw
		draw_robot(simbot.state, false);

	}

	if (fend_IF.receive_options("simulate")) {
		simbot.active = false;
		fend_IF.simululation_off();
	}
}