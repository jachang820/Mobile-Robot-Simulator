class RobotState extends Robot {

	constructor() {
		super();
		this.period = 50.0;
		this.calibrated = false;
		this.is_planning = false;

		this.analog = { left: 0, right: 0};
		
		this.field = { width: 0, length: 0, orient: 0};

		this.P = [[1, 0, 0, 0],
							[0, 1, 0, 0],
							[0, 0, 1, 0],
							[0, 0, 0, 1]];

		this.Gamma = [[0.00101*this.period, 0],
									[0, 0.00257*this.period]];

		this.R = [[1, 0, 0, 0],
							[0, 1, 0, 0],
							[0, 0, 1, 0],
							[0, 0, 0, 1]];
		this.Q = [[1, 0, 0, 0],
							[0, 1, 0, 0],
							[0, 0, 1, 0],
							[0, 0, 0, 1]];
	}


	mirror_state() {
		this.state.x = this.field.length - this.state.x;
		this.state.y = this.field.width - this.state.y;
		this.field.orient = Robot.reduce_angle(this.field.orient - Math.PI);
		this.state.theta = Robot.reduce_angle(this.state.theta - Math.PI);
	}


	motion_planning(target) {
		this.is_planning = true;

		var robot = this;

		var motion_update = new setInterval(function() {

			var dx = target.x - robot.state.x;
			var dy = target.y - robot.state.y;
			var dtheta = Math.atan2(dy, dx);
			var hypot = dy / Math.sin(dtheta);

			// let the state loop clear sensor backlog
			if (http_IF.flags.sensors == true) {

				if (dx < 1.0 && dy < 1.0) {
					http_IF.clear_sensor_data();
					clearInterval(motion_update);
				}

			// there is no more sensor data to process
			} else {

				/* Either simulation processes the target, or
					 an ajax is sent, and the actual robot processes the target.
					 The response will repopulate sensor data. Repeat until it
					 gets close.
				*/

				// ajax not sent
				if (http_IF.ajax == false) {

					http_IF.send_target(hypot, dtheta);

					if (fend_IF.simul_off == true) {
						http_IF.ajax_controls();
					}

				}

			}

		}, 500);
			
	}


	accelerate(inputs, cap=255) {

		if (inputs == null) inputs = {left: 0, right: 0 };
		if (cap > Robot.ANALOG_MAX) cap = Robot.ANALOG_MAX;

		var decel = function(analog, input) {
			var a = input == 0 ? 40 : 25;
			return (Math.abs(analog) > a ? analog - a*Math.sign(analog) : 0);			
		}

		var accel = function(analog, input) {
			return analog + 40 * input;
		}

		this.analog.left = accel(this.analog.left, inputs.left);
		this.analog.left = decel(this.analog.left, inputs.left);
		if (Math.abs(this.analog.left) > cap) {
			this.analog.left = cap * Math.sign(this.analog.left);
		}

		this.analog.right = accel(this.analog.right, inputs.right);
		this.analog.right = decel(this.analog.right, inputs.right);
		if (Math.abs(this.analog.right) > cap) {
			this.analog.right = cap * Math.sign(this.analog.right);
		}

		return this.analog;
	}


	calibrate() {

		var robot = this;
		var EPSILON = 0.05 // 3 degrees

		var angle = robot.imu.mag;
		
		var least_angle;
		var front_one;
		var right_one;

		var opp_angle
		var front_two;
		var right_two;

		var dist_one;
		var angle_one;
		var dist_two;
		var angle_two;

		var max_voltage = 8;
		var signalled = false;

		$("#status").show();

		var dist_avg;
		var dist2_avg;
		var angle_avg;
		var measurements;


		var promise = new Promise(function(resolve) {

			/* Initial distance measurement */

			dist_avg = 0;
			angle_avg = 0;
			measurements = 0;

			var first_measurement = setInterval(function() {

				if (http_IF.flags.sensors == true) {
				
					robot.update_sensors(http_IF.receive_sensor_data());

					dist_avg += robot.front_laser.dist;
					angle_avg += robot.imu.mag;
					measurements += 1;

				}

				if (measurements > 100) {

					clearInterval(first_measurement);
					dist_one = (dist_avg / measurements) + Robot.FRONT_LASER.y;
					angle_one = (angle_avg / measurements);

					console.log("Done first distance, ", dist_one, " angle, ", angle_one*180/Math.PI);
					resolve(angle_one);
				}

			}, robot.period);

		}).then(function(starting_angle) {

			return new Promise(function(resolve) {

				measurements = 0;

				var turn_small_angle = setInterval(function() {

					// maintain accuracy
					max_voltage = Math.abs(Robot.reduce_angle(robot.imu.mag - robot.prev.imu.mag)) > 
												0.05 ? max_voltage - 0.25 : max_voltage + 0.25; 
					if (max_voltage < 6) max_voltage = 6;

					// provide input, then wait for response
					if (signalled == false) {
						signalled = true;
						robot.accelerate({left: -1, right: 1}, max_voltage);
						http_IF.send_controls([robot.analog.left], [robot.analog.right]);
					}

					// sensor data has been updated
					if (http_IF.flags.sensors == true) {

						signalled = false;

						// update sensor measurements
						robot.update_sensors(http_IF.receive_sensor_data());

						// turn a small angle
						var delta = Robot.reduce_angle(starting_angle + 0.3);
						if (robot.imu.mag > delta && robot.prev.imu.mag > delta) {

							measurements += 1;

							robot.accelerate({left: 0, right: 0}, 0.0);
							http_IF.send_controls([0], [0]);

							if (Math.abs(robot.imu.mag - robot.prev.imu.mag) < 0.05 && measurements >= 3) {

								clearInterval(turn_small_angle);

								console.log("Done small angle, ", robot.imu.mag*180/Math.PI);
								resolve(robot.imu.mag);
							}

						}

					}

				}, robot.period);

			});

		}).then(function(starting_angle) {

			return new Promise(function(resolve) {

				/* Second distance measurement */

				dist_avg = 0;
				angle_avg = 0;
				measurements = 0;

				var second_measurement = setInterval(function() {

					if (http_IF.flags.sensors == true) {
					
						robot.update_sensors(http_IF.receive_sensor_data());

						dist_avg += robot.front_laser.dist;
						angle_avg += robot.imu.mag;
						measurements += 1;

					}

					if (measurements > 100) {

						clearInterval(second_measurement);
						dist_two = (dist_avg / measurements) + Robot.FRONT_LASER.y;
						angle_two = (angle_avg / measurements);

						console.log("Done second distance, ", dist_two, " angle, ", angle_two*180/Math.PI);
						resolve(angle_two);
					}

				}, robot.period);

			});

		}).then(function(last_angle) {

			// law of cosines
			var dtheta = angle_two - angle_one;
			var opp_side = Math.sqrt(Math.pow(dist_one, 2) + Math.pow(dist_two, 2) -
										 2 * dist_one * dist_two * Math.cos(dtheta));

			// law of sines
			var theta_ratio = Math.sin(dtheta) / opp_side;
			var alpha = Math.asin(dist_two * theta_ratio);
			var beta = Math.asin(dist_one * theta_ratio);

			console.log("t: ", dtheta, " a: ", alpha, " b: ", beta);

			// lucky! angle right on target
			if (beta == Math.PI/2) {

				least_angle = beta;

			// since shortest distance is perpendicular, find the third angle
			} else {

				least_angle = Math.abs(beta - Math.PI/2);

			}

			console.log("Least angle: ", least_angle);

			// if acute, need to travel backwards
			var sign = Math.sign(beta - Math.PI/2);

			/* find perpendicular angle */

			return new Promise(function(resolve) {

				var find_perpendicular = setInterval(function() {

					// maintain accuracy
					max_voltage = Math.abs(Robot.reduce_angle(robot.imu.mag - robot.prev.imu.mag)) > 
						0.05 ? max_voltage - 0.25 : max_voltage + 0.25; 
					if (max_voltage < 8) max_voltage = 8;

					// provide input, then wait for response
					if (signalled == false) {
						signalled = true;
						robot.accelerate({left: -1*sign, right: 1*sign}, max_voltage);
						http_IF.send_controls([robot.analog.left], [robot.analog.right]);
					}

					// sensor data has been updated
					if (http_IF.flags.sensors == true) {

						signalled = false;
						measurements += 1

						// update sensor measurements
						robot.update_sensors(http_IF.receive_sensor_data());

						// go within some epsilon of the perpendicular angle
						if (Math.abs(Robot.reduce_angle(robot.imu.mag - (last_angle + least_angle*sign))) < EPSILON) {

							robot.accelerate({left: 0, right: 0}, 0.0);
							http_IF.send_controls([0], [0]);

							if (Math.abs(robot.imu.mag - robot.prev.imu.mag) < 0.05) {
								
								clearInterval(find_perpendicular);

								console.log("Found first least angle, ", robot.imu.mag*180/Math.PI);
								resolve(robot.imu.mag);
							}
						}
					}

				}, robot.period);

			});
	
		}).then(function(least) {

			return new Promise(function(resolve) {

				/* Least distance measurement */

				dist_avg = 0;
				dist2_avg = 0;
				angle_avg = 0;
				measurements = 0;

				var least_measurement = setInterval(function() {

					if (http_IF.flags.sensors == true) {
					
						robot.update_sensors(http_IF.receive_sensor_data());

						dist_avg += robot.front_laser.dist;
						dist2_avg += robot.right_laser.dist;
						angle_avg += robot.imu.mag + 2*Math.PI;
						measurements += 1;

					}

					if (measurements > 100) {

						clearInterval(least_measurement);
						front_one = (dist_avg / measurements) + Robot.FRONT_LASER.y;
						right_one = (dist2_avg / measurements) + Robot.RIGHT_LASER.x;
						least_angle = Robot.reduce_angle(angle_avg / measurements);

						console.log("Done least angle measurement, ", least_angle*180/Math.PI, " dist: ", front_one, right_one);
						resolve(least_angle);
					}

				}, robot.period);

			});
		
		}).then(function(least) {

			/* find opposite angle */

			return new Promise(function(resolve) {

				var find_opposite = setInterval(function() {

					var opp_angle = Robot.reduce_angle(least_angle + Math.PI);
					console.log("opp: ", opp_angle*180/Math.PI, " least: ", least_angle*180/Math.PI);

					// maintain accuracy
					max_voltage = Math.abs(Robot.reduce_angle(robot.imu.mag - robot.prev.imu.mag)) >
											 0.05 ? max_voltage - 0.25 : max_voltage + 0.25; 
					if (max_voltage < 8) max_voltage = 8;

					// provide input, then wait for response
					if (signalled == false) {

						signalled = true;

						// go max speed until within 30 degrees of target
						robot.accelerate({left: -1, right: 1}, 
														 (Math.abs(Robot.reduce_angle(opp_angle - robot.imu.mag)) < 
														 	0.5 ? max_voltage : Robot.ANALOG_MAX));
						http_IF.send_controls([robot.analog.left], [robot.analog.right]);
					}

					// sensor data has been updated
					if (http_IF.flags.sensors == true) {

						signalled = false;
						measurements += 1

						// update sensor measurements
						robot.update_sensors(http_IF.receive_sensor_data());


						// go within some epsilon of the perpendicular angle
						if (Math.abs(Robot.reduce_angle(robot.imu.mag - opp_angle)) < EPSILON) {

							robot.accelerate({left: 0, right: 0}, 0.0);
							http_IF.send_controls([0], [0]);

							if (Math.abs(robot.imu.mag - robot.prev.imu.mag) < 0.05) {
								clearInterval(find_opposite);

								console.log("Found opposite angle, ", robot.imu.mag*180/Math.PI);
								resolve(robot.imu.mag);
							}
						}
					}

				}, robot.period);

			});

		}).then(function(opp) {

			return new Promise(function(resolve) {

				/* Opposite distance measurement */

				dist_avg = 0;
				dist2_avg = 0;
				angle_avg = 0;
				measurements = 0;

				var opposite_measurement = setInterval(function() {

					if (http_IF.flags.sensors == true) {
					
						robot.update_sensors(http_IF.receive_sensor_data());

						dist_avg += robot.front_laser.dist;
						dist2_avg += robot.right_laser.dist;
						angle_avg += robot.imu.mag + 2*Math.PI;
						measurements += 1;

					}

					if (measurements > 100) {

						clearInterval(opposite_measurement);
						front_two = (dist_avg / measurements) + Robot.FRONT_LASER.y;
						right_two = (dist2_avg / measurements) + Robot.RIGHT_LASER.x;
						opp_angle = Robot.reduce_angle(angle_avg / measurements);

						console.log("Done opposite angle measurement, ", opp_angle*180/Math.PI, " dist: ", front_two, right_two);
						resolve(opp_angle);
					}

				}, robot.period);

			});

		}).then(function(opp) {

			var front_measure = front_one + front_two;
			var right_measure = right_one + right_two;

			if (front_measure >= right_measure) {

				// assume robot points right at least distance, then the facing
				// dimension is the length, orientation faces up at +90deg, and x, y is
				// measured from behind, and right, respectively
				robot.field.length = front_measure;
				robot.field.width = right_measure;
				robot.field.orient = least_angle + Math.PI / 2;
				robot.state.x = front_two;
				robot.state.y = right_one;

			} else {

				// assume robot points up at least distance, then the facing dimension
				// of the field is the width, orientation faces up at 0deg, and x, y is
				// measured from the bottom and left, respectively
				robot.field.width = front_measure;
				robot.field.length = right_measure;
				robot.field.orient = least_angle;
				robot.state.x = right_two;
				robot.state.y = front_two;

			}

			// try to orient 0deg along width of field
			robot.state.theta = Robot.reduce_angle(robot.imu.mag - robot.field.orient);
			console.log("State, ", robot.state);
			console.log("Field, ", robot.field);

			robot.calibrated = true;
			sandbox.draw(robot.field.width, robot.field.length);

		});

	}


	update_sensors(sensors) {
		this.prev.front_laser.dist = this.front_laser.dist;
		this.prev.right_laser.dist = this.right_laser.dist;
		this.prev.imu.mag = this.imu.mag;
		this.prev.imu.gyro = this.imu.gyro;
		this.prev.imu.accel = this.imu.accel;
		this.front_laser.dist = sensors.front;
		this.right_laser.dist = sensors.right;
		this.imu.mag = Robot.reduce_angle(sensors.mag);
		this.imu.gyro = Robot.reduce_angle(sensors.gyro);
		this.imu.accel = sensors.accel;
	}


	estimate_state() {
		var sin = Math.sin(this.state.theta);
		var cos = Math.cos(this.state.theta);
		var dtheta = this.imu.gyro * this.period/1000;
		var a = this.imu.accel * this.period/1000;
		var arc_length = (this.state.v + a) / 2;
		var radius = arc_length / dtheta;
		var sin_delta = Math.sin(this.state.theta + dtheta);
		var cos_delta = Math.cos(this.state.theta + dtheta);

		var orient = Robot.next_orientation(this.state, arc_length, radius, dtheta);

		if (dtheta > 0.1) {
			A = [ [1, 0, r*(sin - sin_delta), (cos_delta - cos)/(2*dtheta)],
						[0, 1, r*(cos_delta - cos), (sin_delta - sin)/(2*dtheta)],
						[0, 0, 1, 0],
						[0, 0, 0, 1] ];
			B = [ [-r*((cos_delta - cos)/dtheta + sin_delta), (cos_delta - cos)/(2*dtheta)],
						[-r*((sin_delta - sin)/dtheta - cos_delta), (sin_delta - sin)/(2*dtheta)],
						[1, 0],
						[0, 1] ];
		} else {
			A = [ [1, 0, 0, -cos / 2],
						[0, 1, 0, sin / 2],
						[0, 0, 1, 0],
						[0, 0, 0, 1] ];
			B = [ [0, -cos / 2], 
						[0, sin / 2],
						[0, 1], 
						[1, 0] ];
		}

		var w = [ Robot.gaussian(0, this.Q[0][0] ** 0.5),
							Robot.gaussian(0, this.Q[1][1] ** 0.5),
							Robot.gaussian(0, this.Q[2][2] ** 0.5),
							Robot.gaussian(0, this.Q[3][3] ** 0.5),];

		return {
			x_hat : new State(orient.x + w[0],
												orient.y + w[1],
												orient.theta + w[2],
												this.state.v + a + w[3]),
			A : A,
			B : B
		};
	}


	measure_state(x_hat) {

		var theta = this.imu.mag;
		var sin = Math.sin(theta);
		var cos = Math.cos(theta);
		var f = this.front_laser.dist + Robot.FRONT_LASER.y;
		var r = this.right_laser.dist + Robot.RIGHT_LASER.x +
						Math.abs(Robot.RIGHT_LASER.y) * sin/cos;
		var h = this.field.width;
		var w = this.field.length;
		var f_pos = this.front_laser.pos;
		var r_pos = this.right_laser.pos;

		var front_wall = Robot.distance_from_point(
										 new State(f_pos.x, f_pos.y, theta));
		var right_wall = Robot.distance_from_point(
										 new State(r_pos.x, r_pos.y, theta));

		var estimator = {
			front : {
				right  : [ f*sin, 0, f*cos, 0 ],
				top    : [ 0, h - f*cos, 0, f*sin ],
				left   : [ w + f*sin, 0, f*cos, 0 ],
				bottom : [ 0, -f*cos, 0, f*sin ]
			},
			right : {
				right  : [-r*cos, 0, r*sin, 0 ],
				top    : [ 0, h - r*sin, 0 -r*cos ],
				left   : [ w - r*cos, 0, r*sin, 0 ],
				bottom : [ 0, -r*sin, 0, -r*cos ]
			}
		};


		var f_est = estimator.front[front_wall.wall];
		var r_est = estimator.right[right_wall.wall];
		var combined = [
			[(f_est[0] + r_est[0]) / 2],
			[(f_est[1] + r_est[1]) / 2],
			[(f_est[2] + r_est[2]) / 2],
			[(f_est[3] + r_est[3]) / 2]
		];

		if (combined[0] == 0) {
			combined[0] = front_wall.pos + f*sin;
			combined[2] = f*cos;
		}
		if (combined[1] == 0) {
			combined[1] = front_wall.pos - f*cos;
			combined[3] = f*sin;
		}

		var v = [ Robot.gaussian(0, this.R[0][0] ** 0.5),
							Robot.gaussian(0, this.R[1][1] ** 0.5),
							Robot.gaussian(0, this.R[2][2] ** 0.5),
							Robot.gaussian(0, this.R[3][3] ** 0.5),];

		return {
			z : new State(combined[0] + v[0],
										combined[1] + v[1],
										theta + v[2],
										x_hat.v + v[3]),
			H : [ [1       , sin*cos, combined[2], 0],
						[-sin*cos, 1      , combined[3], 0],
						[0       , 0      , 1          , 0],
						[0       , 0      , 0          , 1] ]
		};
	}


	kalman_predict() {

		var estimate = this.estimate_state();

		this.prev.P = this.P;
		var A = matmul(estimate.A, this.P);
		A = matmul(A, transpose(estimate.A));
		var B = matmul(estimate.B, this.Gamma);
		B = matmul(B, transpose(estimate.B));
		this.P = matadd(matadd(A + B) + this.Q);

		this.estimate = estimate;
	}


	kalman_update() {

		var estimate = this.estimate;

		var measure = this.measure_state(estimate.x_hat);

		var y = [ measure.z.x - estimate.x_hat.x,
							measure.z.y - estimate.x_hat.y,
							measure.z.theta - estimate.x_hat.theta,
							measure.z.vel - estimate.x_hat.v ];

		var S = matmul(measure.H, this.P);
		S = matmul(S, transpose(measure.H));
		S = matadd(S, this.R);

		var K = matmul(this.P, transpose(measure.H));
		K = matmul(K, pinv(S));

		var x = matadd(estimate.x_hat, matmul(K, y));

		I = identity(4);
		P = matmul(matsub(I, matmul(K, H)), P);

		this.prev.state = this.state;
		this.state = x;
		return x;
	}

}


function state_loop(robot) {

	if (robot.calibrated) {

		// check for long-range movement
		var click = fend_IF.receive_click();
		
		// only check arrows if no clicks
		if (click == null || robot.is_planning == true) {

			// only receive arrow inputs with simulator
			if (fend_IF.simul_off == false) {

				// check if user pressed moved the bot
				var inputs = fend_IF.receive_inputs();
				var analog = robot.accelerate(inputs);
				http_IF.send_controls([analog.left], [analog.right]);

			}

		} else {

			// only receive motion planning for actual robot for now
			if (fend_IF.simul_off == true) {

				var controls_list = robot.motion_planning(click); 
				http_IF.send_controls(controls_list.left, controls_list.right);

			}
		}

		// update sensors
		robot.update_sensors(http_IF.receive_sensor_data());

		// update current state
		robot.kalman_update();

		// predict next state
		robot.kalman_predict();

		// receive options
		if (fend_IF.receive_options("mirror")) {
			robot.mirror_state();
		}

		// update relative parts positions
		robot.update_positions();

		// draw
		draw_robot(robot.state, true);

	} else {

		// update relative parts positions
		robot.update_positions();

	}


	if (fend_IF.receive_options("calibrate")) {

		robot.update_sensors(http_IF.receive_sensor_data());
		
		// prepare for calibration
		http_IF.clear_sensor_data();

		// simulation has been turned off
		if (fend_IF.simul_off == true) {

			// take in sensor and calibration data
			if (http_IF.flags.sensors == true) {
				console.log("oops, shouldn't be here");
				robot.update_sensors(http_IF.receive_sensor_data());
				var data = http_IF.receive_calibration_data();
				robot.field.width = data.width;
				robot.field.length = data.length;
				robot.field.orient = data.orient;
				robot.R = data.R;
				robot.Q = data.Q;

				robot.calibrated = true;
				sandbox.draw(robot.field.width, robot.field.length);

			// ajax not sent yet
			} else if (http_IF.ajax == false) {
				
				http_IF.send_calibrate_signal();
				http_IF.ajax_controls();
			
			}

		// simulation is still on
		} else {
			
			robot.calibrate();
		}
	}

}