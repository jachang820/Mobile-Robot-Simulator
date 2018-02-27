// Assumptions
const SANDBOX_HEIGHT_RATIO = 0.95;
const BIAS_ANGLE = 0.0;
const LASER_MAX_RANGE = 150.0;
const BUFFER = 9.0;

// Pixel dimensions
var docWidthPx;
var docHeightPx;
var sandboxWidthPx;
var sandboxLengthPx;
var sandboxLeftPx;
var sandboxTopPx;
var scalePxPerCm;

// Parameter settings
var sandboxWidthCm = 80.0;
var sandboxLengthCm = 100.0;
var trueNorth;

// Controls
var upKeyPressed;
var downKeyPressed;
var leftKeyPressed;
var rightKeyPressed;
var leftVoltageRatio;
var rightVoltageRatio;


/**
 * A collection of properties and methods related to the robot frame of reference.
 * The robot knows these properties and methods since they are internal decisions made
 * during robot design.
 */
class RobotFrame {
	static get WIDTH() { return 9.0; }
	static get LENGTH() { return 10.5; }
	static get AXLE_LENGTH() { return 8.4; }
	static get WHEEL_DIAMETER() { return 5.0; }
	static get WHEEL_CIRCUMFERENCE() { return 15.3938; }
	static get IMU() { return {x: 0.0, y: 0.0 }; }
	static get FRONT_LASER() { return {x: 0.0, y: 2.5 }; }
	static get RIGHT_LASER() { return {x: 3.0, y: -2.5 }; }
	static get LEFT_WHEEL() { return {x: -4.2, y: 0.0 }; }
	static get RIGHT_WHEEL() { return {x: 4.2, y: 0.0 }; }
	static get FRONT_LEFT() { return {x: -4.2, y: 3.7 }; }
	static get FRONT_RIGHT() { return {x: 4.2, y: 3.7 }; }
	static get BACK_LEFT() { return {x: -4.2, y: -7.1 }; }
	static get BACK_RIGHT() { return {x: 4.2, y: -7.1 }; }
	static get MAX_RPM() { return 100; }
	static get MAX_CM_PER_S() { return RobotFrame.MAX_RPM * RobotFrame.WHEEL_CIRCUMFERENCE / 60.0; }
	
	/**
	 * Rotates a position about an axes by theta, counterclockwise.
	 * @param  {!Dictionary} rel_pos - x, y position relative to the axis of rotation.
	 * @param  {number} d_theta - Angle to rotate. Counterclockwise is positive.
	 * @return {!Dictionary} The rotated position.
	 */
	static rotate(rel_pos, d_theta) {
		var cos = Math.cos(d_theta);
		var sin = Math.sin(d_theta);

		return {
			x: rel_pos.x * cos - rel_pos.y * sin,
			y: rel_pos.x * sin + rel_pos.y * cos
		};
	}

	/**
	 * Translates a position in field frame.
	 * @param  {!Position} pos - Starting x, y position.
	 * @param  {number} dx - Position to translate on x-axis. Right is positive.
	 * @param  {number} dy - Position to translate on y-axis. Up is positive.
	 * @return {!Position} The translated position.
	 */
	static translate(pos, dx, dy) {
		return {
			x: pos.x + dx,
			y: pos.y + dy
		};
	}


	/**
	 * Rotates and translates a position in field frame.
	 * @param  {!Position} rel_pos - x, y position relative to the axis of rotation.
	 * @param  {!State} state - Differential state (x, y, theta) in field frame.
	 * @return {!Position} Transformed position.
	 */
	static transform(rel_pos, state) {
		var pos = RobotFrame.rotate(rel_pos, state.theta);
		pos = RobotFrame.translate(pos, state.x, state.y);
		
		return pos;
	}



	static distance(pos, theta) {

		theta = reduceAngle(theta);

		// if angle is too small, prevent large calculations
		const THRESHOLD = Math.PI / 24;

		var cos = Math.cos(theta);
		var sin = Math.sin(theta);
		var abs_theta = Math.abs(theta);

		// relevant walls depending on quadrant of angle
		var vert_wall = (theta < 0 ? sandboxLengthCm : 0);
		var horiz_wall = (abs_theta < Math.PI / 2 ? sandboxWidthCm : 0);

		// distance to each wall, or Infinity if distance too large
		var dist_to_vert = (abs_theta > THRESHOLD ? Math.abs((vert_wall - pos.x) / sin) : Infinity);
		var dist_to_horiz = (Math.abs(abs_theta - Math.PI / 2) > THRESHOLD ? Math.abs((horiz_wall - pos.y) / cos) : Infinity);

		// both walls unreachable
		if (!isFinite(dist_to_vert) && !isFinite(dist_to_horiz)) {

			return Infinity;

		} else {

			// least distance
			return (dist_to_vert < dist_to_horiz ? dist_to_vert : dist_to_horiz);

		}
	}


		/**
	 * Respond to motor voltage input and update robot state.
	 */
	static drive(state, leftVoltageRatio, rightVoltageRatio, period) {

		// distance traveled by each wheel of robot in this timeslice
		var leftArc = leftVoltageRatio * period;
		var rightArc = rightVoltageRatio * period;

		// change of angle
		var d_theta = (rightArc - leftArc) / RobotFrame.AXLE_LENGTH;

		// robot is moving straight
		if (d_theta == 0) {

			// let initial position be axis of rotation, and rotate new position about this axis
			var rel_pos = {x: 0, y: leftArc };
			var new_pos = RobotFrame.transform(rel_pos, state);

			// update robot state
			state.x = new_pos.x;
			state.y = new_pos.y;

		// robot is turning
		} else {

			// For short times, we assume there is negligible acceleration, such that each wheel moves evenly.
			// Then, we can envision each turn as the wheels moving along the same arc angle along the
			// circumference of concentric circles. 'r_center' is the radius from the center of these circles to
			// the reference point at the center of the robot axle.
			var r_center = RobotFrame.AXLE_LENGTH * ((Math.min(leftArc, rightArc) / Math.abs(rightArc - leftArc)) + 0.5);
			
			// The concentric circles center at the right side of the robot if the left wheel goes further, and
			// vice versa. The right side is denoted by positive distance.
			var coeff = (leftArc > rightArc ? 1 : -1);
			
			// Since 'r_center' extends from the axle, the center of the circles is a transformation moving sideways
			// from robot frame. 'center' is the center of the concentric circles.
			var rel_pos = {x: coeff * r_center, y: 0 };
			var center = RobotFrame.transform(rel_pos, state);

			// We let 'center' be the axis of rotation so we find the robot path along the circle circumference.
			rel_pos = {x: state.x - center.x, y: state.y - center.y };
			var d_state = {x: center.x, y: center.y, theta: d_theta };
			var new_pos = RobotFrame.transform(rel_pos, d_state);

			// update robot state
			state.x = new_pos.x;
			state.y = new_pos.y;
			state.theta += d_theta;
			state.theta = reduceAngle(state.theta);

		}

		return state;

	}

}


/**
 * A collection of properties and methods related to the actual robot state.
 * The robot does not know these properties and methods since they relate to the
 * design of the field and physical results of actuation.
 */
class RobotState {

	constructor(startState) {
		// initial position is random, unless stated
		this.state = startState || this.randStartingPosition();

		// parts of the robot that have tracked positions
		this.imu = {};
		this.frontLaser = {};
		this.rightLaser = {};
		this.leftWheel = {};
		this.rightWheel = {};
		this.frontLeft = {};
		this.frontRight = {};
		this.backLeft = {};
		this.backRight = {};

		this.intervals = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
		this.interval_q_num = 0;
		this.interval_q_size = 10;

		// update parts positions at starting state
		this.update_state();

		// input signals
		this.leftSignal = 0;
		this.rightSignal = 0;
		this.leftVoltageRatio = 0;
		this.rightVoltageRatio = 0;

		// register event to store voltage state when robot senses it
		var robotState = this;
		$("#robot").on("actuateEvent", function(e, leftVoltageRatio, rightVoltageRatio) {
			
			robotState.leftVoltageRatio = leftVoltageRatio;
			robotState.rightVoltageRatio = rightVoltageRatio;

		});

		this.readState = function() {

			var now = Date.now();
			robotState.intervals[robotState.interval_q_num] = now - robotState.lastRead;
			robotState.interval_q_num += 1;
			if (robotState.interval_q_num == robotState.interval_q_size) { 
				robotState.interval_q_num = 0;
			}
			robotState.lastRead = now;

			// drive based on voltage input
			robotState.state = RobotFrame.drive(robotState.state, 
																					robotState.leftVoltageRatio, 
																					robotState.rightVoltageRatio,
																					RobotState.MAX_CM_IN_CPU_PERIOD);

			// update positions of robot parts
			robotState.update_state();

			// draw updated robot
			$(document).trigger("drawEvent", [ robotState.state, true ] );
		}

		// continuously update state
		this.stateUpdate = setInterval(this.readState, RobotState.CPU_PERIOD);
		
	}

	static get CPU_FREQ() { return 200; }
	static get CPU_PERIOD() { return 5.0; } // millisecond
	static get MAX_CM_IN_CPU_PERIOD() { return RobotFrame.MAX_CM_PER_S * RobotState.CPU_PERIOD / 1000; }


	update_state() {

		// rotate robot, then translate, according to actual state
		this.imu.pos = RobotFrame.transform(RobotFrame.IMU, this.state);
		this.frontLaser.pos = RobotFrame.transform(RobotFrame.FRONT_LASER, this.state);
		this.rightLaser.pos = RobotFrame.transform(RobotFrame.RIGHT_LASER, this.state);
		this.leftWheel.pos = RobotFrame.transform(RobotFrame.LEFT_WHEEL, this.state);
		this.rightWheel.pos = RobotFrame.transform(RobotFrame.RIGHT_WHEEL, this.state);
		this.frontLeft.pos = RobotFrame.transform(RobotFrame.FRONT_LEFT, this.state);
		this.frontRight.pos = RobotFrame.transform(RobotFrame.FRONT_RIGHT, this.state);
		this.backLeft.pos = RobotFrame.transform(RobotFrame.BACK_LEFT, this.state);
		this.backRight.pos = RobotFrame.transform(RobotFrame.BACK_RIGHT, this.state);
		
		// read input signals
		this.signal_controls();
	}


	/** 
	 * Prepare control signals and normalize.
	 * @return {!Controls} Left, right control signals.
	 */
	signal_controls() {
		var upKey = (upKeyPressed ? 1 : 0);
		var downKey = (downKeyPressed ? 1 : 0);
		var leftKey = (leftKeyPressed ? 1 : 0);
		var rightKey = (rightKeyPressed ? 1 : 0);
		var left = upKey - downKey + rightKey - leftKey;
		left = Math.sign(left) * Math.sqrt(Math.abs(left));
		var right = upKey - downKey + leftKey - rightKey;
		right = Math.sign(right) * Math.sqrt(Math.abs(right));
		this.leftSignal = left;
		this.rightSignal = right;
	};

	/**
	 * Generates random legal starting position.
	 * @return {!State} The state of the robot reference point in field frame (x, y, theta).
	 */
	randStartingPosition() {
		// generates random angle from -pi to pi
		var theta = (Math.random() * 2 * Math.PI) - Math.PI;
		
		// corners of robot rotated about origin
		var front_left = RobotFrame.rotate(RobotFrame.FRONT_LEFT, theta);
		var front_right = RobotFrame.rotate(RobotFrame.FRONT_RIGHT, theta);
		var back_left = RobotFrame.rotate(RobotFrame.BACK_LEFT, theta);
		var back_right = RobotFrame.rotate(RobotFrame.BACK_RIGHT, theta);

		// extremities of robot in field frame
		var min_x = Math.min(front_left.x, front_right.x, back_left.x, back_right.x);
		var max_x = Math.max(front_left.x, front_right.x, back_left.x, back_right.x);
		var min_y = Math.min(front_left.y, front_right.y, back_left.y, back_right.y);
		var max_y = Math.max(front_left.y, front_right.y, back_left.y, back_right.y);

		// generate random legal x, y position taking into account size of robot
		var x = Math.random() * (sandboxLengthCm - (max_x - min_x) - 2 * BUFFER) - min_x + BUFFER;
		var y = Math.random() * (sandboxWidthCm - (max_y - min_y) - 2 * BUFFER) - min_y + BUFFER;
		return {x: x, y: y, theta: theta};

	}

}


class Robot {
	
	constructor() {

		this.actual_state = new RobotState();

		// default input
		this.leftVoltageRatio = 0.0;
		this.rightVoltageRatio = 0.0;
		this.leftSignal = 0.0;
		this.rightSignal = 0.0;

		// parts of the robot that have tracked positions
		this.imu = {};
		this.frontLaser = {};
		this.rightLaser = {};
		this.leftWheel = {};
		this.rightWheel = {};
		this.frontLeft = {};
		this.frontRight = {};
		this.backLeft = {};
		this.backRight = {};
		this.state = {};
		this.field = {};

		this.calibrated = false;
		this.intervals = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
		this.interval_q_num = 0;
		this.interval_q_size = 10;

		// get reading from sensors
		this.sense();

		// get an initial sense of things
		this.calibrate();

		// sense and actuate every so often
		var robot = this;
		this.readState = function() {

			var now = Date.now()
			robot.intervals[robot.interval_q_num] = now - robot.lastRead;
			robot.interval_q_num += 1;
			if (robot.interval_q_num == robot.interval_q_size) { 
				robot.interval_q_num = 0;
			}
			robot.lastRead = now;

			if (robot.calibrated) {

				// drive based on control input
				robot.actuate();

				// update parts positions at starting state
				robot.update_state();

				// draw updated sensed position
				$(document).trigger("drawEvent", [ robot.state, false ] );

				// update sensor measurements based on new position
				robot.sense();

			}

		}

		var senseUpdate = setInterval(this.readState, Robot.SENSE_PERIOD);

		$("#robot").on("mirrorEvent", function(e) {
		
			if (robot.calibrated) {

				robot.state.x = sandboxLengthCm - robot.state.x;
				robot.state.y = sandboxWidthCm - robot.state.y;
				robot.field.orient = reduceAngle(robot.field.orient - Math.PI);
				robot.state.theta = reduceAngle(robot.state.theta - Math.PI);
				$(document).trigger("drawEvent", [ robot.state, false ]);

			}

		});

	}

	static get SENSE_PERIOD() { return 50.0; } // milliseconds
	static get SENSE_FREQ() { return 1000 / Robot.SENSE_PERIOD; }
	static get MAX_CM_IN_SENSE_PERIOD() { return RobotFrame.MAX_CM_PER_S * Robot.SENSE_PERIOD / 1000; }


	update_state() {

		// rotate robot, then translate, according to sensed state
		this.imu.pos = RobotFrame.transform(RobotFrame.IMU, this.state);
		this.frontLaser.pos = RobotFrame.transform(RobotFrame.FRONT_LASER, this.state);
		this.rightLaser.pos = RobotFrame.transform(RobotFrame.RIGHT_LASER, this.state);
		this.leftWheel.pos = RobotFrame.transform(RobotFrame.LEFT_WHEEL, this.state);
		this.rightWheel.pos = RobotFrame.transform(RobotFrame.RIGHT_WHEEL, this.state);
		this.frontLeft.pos = RobotFrame.transform(RobotFrame.FRONT_LEFT, this.state);
		this.frontRight.pos = RobotFrame.transform(RobotFrame.FRONT_RIGHT, this.state);
		this.backLeft.pos = RobotFrame.transform(RobotFrame.BACK_LEFT, this.state);
		this.backRight.pos = RobotFrame.transform(RobotFrame.BACK_RIGHT, this.state);
	
	}


	calibrate() {

		var robot = this;

		var angle = robot.imu.mag;
		var leastDistance = Infinity;
		var angleAtLeastDistance;
		var front_measure_one;
		var right_measure_one;
		var countCycles = 0;
		var trueAngleAtLeast;

		$("#status").show();

		var startingAngle = angle;
		console.log(startingAngle*180/Math.PI);

		// rotate an entire circle and find the smallest measurements to get the
		// orientation of the field -- to be used in a promise
		var promise = new Promise(function(resolve) {

			var calibration_first_interval = setInterval(function() {

				robot.actuate(-1, 1, 0.10);

				// update sensor measurements
				robot.sense();

				// angle wraps around
				if (angle > Math.PI / 2 && robot.imu.mag < -Math.PI / 2) {
					
					countCycles += 1;

				// buffer against noise, in case angle reverses at the cusp
				} else if (angle < -Math.PI / 2 && robot.imu.mag > Math.PI / 2) {

					countCycles -= 1;

				}

				// ensure that at least a full cycle is made
				if (countCycles == 1 && robot.imu.mag > startingAngle ) {
					console.log("angle: " + angle + ", imu.mag: " + robot.imu.mag);
					robot.actuate(0, 0, 0.0);
					clearInterval(calibration_first_interval);
					resolve(angleAtLeastDistance);
				}

				angle = robot.imu.mag;

				// wall with the least distance from the front laser must be perpendicular since it lies on axis
				// measure shortest distance
				if (robot.frontLaser.dist < leastDistance) {
					leastDistance = robot.frontLaser.dist;
					angleAtLeastDistance = angle;

					// measure field dimensions on one side
					front_measure_one = robot.frontLaser.dist + RobotFrame.FRONT_LASER.y;
					right_measure_one = robot.rightLaser.dist + RobotFrame.RIGHT_LASER.x;
				}


			}, Robot.SENSE_PERIOD);

		}).then(function(angleAtLeastDistance) {

			var half_circle = reduceAngle(angleAtLeastDistance + Math.PI);
			var epsilon = Math.PI;

			// measure the opposite side of the least distance angle
			return new Promise(function(resolve) {

				var calibration_second_interval = setInterval(function() {

					robot.actuate(-1, 1, 0.10);
					robot.sense();
					epsilon = Math.abs(robot.imu.mag - half_circle);

					if (epsilon < 0.005) { // ~0.58deg tolerance
						robot.actuate(0, 0, 0.0);
						clearInterval(calibration_second_interval);
						resolve();
					}

				}, RobotState.CPU_PERIOD);


			});

		}).then(function() {
		
			// turn to measure the other side
			var front_measure_two = robot.frontLaser.dist + RobotFrame.FRONT_LASER.y;
			var right_measure_two = robot.rightLaser.dist + RobotFrame.RIGHT_LASER.x;

			// get complete field dimensions
			var front_measure = front_measure_one + front_measure_two;
			var right_measure = right_measure_one + right_measure_two;
			var front_shorter = (front_measure < right_measure ? true : false);
			console.log(front_shorter);

			// the direction where shortest distance was measured turns out to be the
			// length of the field
			if (front_measure >= right_measure) {

				// assume robot points right at least distance, then the facing
				// dimension is the length, orientation faces up at +90deg, and x, y is
				// measured from behind, and right, respectively
				robot.field.length = front_measure;
				robot.field.width = right_measure;
				robot.field.orient = angleAtLeastDistance + Math.PI / 2;
				robot.state.x = front_measure_two;
				robot.state.y = right_measure_one;

			// the direction where shortest distance was measured is the width of the
			// field
			} else {

				// assume robot points up at least distance, then the facing dimension
				// of the field is the width, orientation faces up at 0deg, and x, y is
				// measured from the bottom and left, respectively
				robot.field.width = front_measure;
				robot.field.length = right_measure;
				robot.field.orient = angleAtLeastDistance;
				robot.state.x = right_measure_two;
				robot.state.y = front_measure_two;
			
			}

			// try to orient 0deg along width of field
			robot.state.theta = robot.imu.mag - robot.field.orient;
			console.log("Orientation: ", robot.field.orient*180/Math.PI);

			// hide notice on screen
			$("#status").hide();

			robot.calibrated = true;
			$("#shadow").show();
			$(document).trigger("drawEvent", [robot.state, false]);

		});

	}


	/**
	 * Update global voltage state in response to control signals by accelerating naturally.
	 * @param  {number} input - Control signal for one motor.
	 * @param  {number} voltageRatio - Percentage of max motor speed.
	 * @return {number} Updated motor speed.
	 */
	accelerate(input, voltageRatio, cap) {

		// default cap is 1
		if (!cap) cap = 1.0;

		// improve controls by syncing the sides faster after turn button released
		var slowest = Math.min(Math.abs(this.leftVoltageRatio), Math.abs(this.rightVoltageRatio));
		var diff_step = (Math.abs(voltageRatio) - slowest) / 3.0;

		// gradually decrease speed
		if (input == 0) {

			var direction = Math.sign(voltageRatio);
			voltageRatio -= ((0.15 + diff_step) * direction);

			// don't let motor reverse
			if (Math.sign(voltageRatio) != direction) voltageRatio = 0.0;

		// there is input, gradually increase speed
		} else {
			voltageRatio += (0.10 * input);

			// limit to maximum forward speed
			if (voltageRatio > cap) voltageRatio = cap;

			// limit to maximum reverse speed
			if (voltageRatio < -cap) voltageRatio = -cap;
		}

		return voltageRatio;
	}


	updateLeftSignal() {
		return this.actual_state.leftSignal;
	}

	updateRightSignal() {
		return this.actual_state.rightSignal;
	}

	updateLaserRange(pos, theta) {
		var dist = RobotFrame.distance(pos, theta);
		if (dist > LASER_MAX_RANGE) {
			dist = Infinity;
		}
		return dist;
	}

	updateFrontLaserRange() {
		var robotState = this.actual_state;
		return this.updateLaserRange(robotState.frontLaser.pos, robotState.state.theta);
	}

	updateRightLaserRange() {
		var robotState = this.actual_state;
		return this.updateLaserRange(robotState.rightLaser.pos, robotState.state.theta - Math.PI / 2);
	}

	updateMagnetometer() {
		var theta = this.actual_state.state.theta;
		return reduceAngle(theta - trueNorth);
	}


	/**
	 * Update all sensor measurements. Only the individual sensors should have access to the true states.
	 */
	sense() {

		// store input signals
		this.leftSignal = addNoise(this.updateLeftSignal(), 0.05);
		this.rightSignal = addNoise(this.updateRightSignal(), 0.05);

		// measure distance using lasers
		this.frontLaser.dist = addNoise(this.updateFrontLaserRange(), 0.08);
		this.rightLaser.dist = addNoise(this.updateRightLaserRange(), 0.08);
		//console.log("Theta: ", theta*180/Math.PI, ", RDist: ", this.rightLaser.dist);

		// magnetometer should be calibrated and converted into an angle in robot's code
		this.imu.mag = addNoise(this.updateMagnetometer(), 0.02);

	}


	fixSenseRatio() {

		function add(a, b) { return a + b; }

		// average time between each sense period over cpu period, since setInterval isn't accurate
		var ratio = this.intervals.reduce(add, 0) / this.actual_state.intervals.reduce(add, 0);

		// average number of times cpu interval has run in one sense interval
		return ratio * RobotState.MAX_CM_IN_CPU_PERIOD;

	}


	actuate(leftSgn, rightSgn, cap) {

		// maximum speed given
		if (!cap) cap = 1.0;

		// signals given
		if (!leftSgn) leftSgn = this.leftSignal;
		if (!rightSgn) rightSgn = this.rightSignal;

		// accelerate using previous voltage and current signal
		this.leftVoltageRatio = addNoise(this.accelerate(leftSgn, this.leftVoltageRatio, cap), 0.02);
		this.rightVoltageRatio = addNoise(this.accelerate(rightSgn, this.rightVoltageRatio, cap), 0.02);

		// pass to state
		$("#robot").trigger("actuateEvent", [ this.leftVoltageRatio, this.rightVoltageRatio ]);

		// update state estimation
		this.state = RobotFrame.drive(this.state, this.leftVoltageRatio, this.rightVoltageRatio,
																	this.fixSenseRatio());

		// check position based on geometric properties of lasers and angle
		var cons = this.constrain_position();
		$("#constraint").css("top", sandboxTopPx + sandboxWidthPx - scalePxPerCm * cons.max_y);
		$("#constraint").css("left", sandboxLeftPx + scalePxPerCm * cons.min_x);
		$("#constraint").css("height", scalePxPerCm * (cons.max_y - cons.min_y) + 3);
		$("#constraint").css("width", scalePxPerCm * (cons.max_x - cons.min_x) + 3);

	}


	constrain_position(noiseRatio) {

		var epsilon = 0.03; // 3% error
		var f = this.frontLaser.dist;
		var r = this.rightLaser.dist;
		var theta = this.state.theta;
		var sin = Math.abs(Math.sin(theta));
		var cos = Math.abs(Math.cos(theta));
		var tan = Math.abs(Math.tan(theta));
		var quadrant;
		var facing_wall_length;

		// constraints
		var cons = {};

		// facing quadrant
		if (theta <= 0 && theta > -Math.PI / 2) quadrant = 1;
		else if (theta <= Math.PI / 2 && theta > 0) quadrant = 2;
		else if (theta <= Math.PI && theta > Math.PI / 2) quadrant = 3;
		else quadrant = 4;

		if (quadrant == 1 || quadrant == 3) facing_wall_length = sandboxWidthCm;
		else facing_wall_length = sandboxLengthCm;

		// lasers project on opposite walls
		if (f*cos + r*sin >= facing_wall_length * (1 - epsilon)) {

			// facing right
			if (quadrant == 1) {

				cons.min_x = 0;
				cons.max_x = sandboxLengthCm - Math.max(f*sin, r*cos);
				cons.min_y = r*sin;
				cons.max_y = cons.min_y;

			// facing left
			} else if (quadrant == 3) {

				cons.min_x = Math.max(f*sin, r*cos);
				cons.max_x = sandboxLengthCm;
				cons.min_y = f*cos;
				cons.max_y = cons.min_y;

			// facing up
			} else if (quadrant == 2) {

				cons.min_x = r*sin;
				cons.max_x = cons.min_x;
				cons.min_y = 0;
				cons.max_y = sandboxWidthCm - Math.max(f*sin, r*cos);

			// facing down
			} else {

				cons.min_x = f*cos;
				cons.max_x = cons.min_x;
				cons.min_y = Math.max(f*sin, r*cos);
				cons.max_y = sandboxWidthCm;

			}

		// lasers project on same wall
		} else if ((f / r) - Math.tan(theta) <= epsilon) {

			// facing right
			if (quadrant == 1) {

				cons.min_x = sandboxLengthCm - f*cos;
				cons.max_x = cons.min_x;
				cons.min_y = r*cos;
				cons.max_y = f*sin;

			// facing left
			} else if (quadrant == 3) {

				cons.min_x = f*cos;
				cons.max_x = cons.min_x;
				cons.min_y = f*sin;
				cons.max_y = r*cos;

			// facing up
			} else if (quadrant == 2) {

				cons.min_x = f*sin;
				cons.max_x = r*cos;
				cons.min_y = sandboxWidthCm - f*cos;
				cons.max_y = cons.min_y;

			// facing down
			} else {

				cons.min_x = r*cos;
				cons.max_x = f*sin;
				cons.min_y = f*cos;
				cons.max_y = cons.min_y;

			}

		// lasers project on adjacent walls
		} else {

			// facing right
			if (quadrant == 1) {

				// front corner
				if (f*cos < r*sin) {

					cons.x_min = sandboxLengthCm - r*cos;
					cons.x_max = cons.x_min;
					cons.y_min = sandboxWidthCm - f*cos;
					cons.y_max = cons.y_min;

				// rear corner
				} else {

					cons.x_min = sandboxLengthCm - f*sin;
					cons.x_max = cons.x_min;
					cons.y_min = r*sin;
					cons.y_max = cons.y_min;

				}

			// facing left
			} else if (quadrant == 3) {

				// front corner
				if (f*cos < r*sin) {

					cons.x_min = r*cos;
					cons.x_max = cons.x_min;
					cons.y_min = f*cos;
					cons.y_max = cons.y_min;

				// rear corner
				} else {

					cons.x_min = f*sin;
					cons.x_max = cons.x_min;
					cons.y_min = sandboxWidthCm - r*sin;
					cons.y_max = cons.y_min;

				}

			// facing up
			} else if (quadrant == 2) {

				// front corner
				if (f*cos < r*sin) {

					cons.x_min = f*sin;
					cons.x_max = cons.x_min;
					cons.y_min = sandboxWidthCm - r*sin;
					cons.y_max = cons.y_min;

				// rear corner
				} else {

					cons.x_min = sandboxLengthCm - r*cos;
					cons.x_max = cons.x_min;
					cons.y_min = sandboxWidthCm - f*cos;
					cons.y_max = cons.y_min;

				}

			// facing down
			} else {

				// front corner
				if (f*cos < r*sin) {

					cons.x_min = sandboxLengthCm - f*sin;
					cons.x_max = cons.x_min;
					cons.y_min = r*sin;
					cons.y_max = cons.y_min;

				// rear corner
				} else {

					cons.x_min = r*cos;
					cons.x_max = cons.x_min;
					cons.y_min = f*cos;
					cons.y_max = cons.y_min;

				}

			}

		}

		// take into account possible noise
		cons.x_min *= (1 - noiseRatio);
		cons.x_max *= (1 + noiseRatio);
		cons.y_min *= (1 - noiseRatio);
		cons.y_max *= (1 + noiseRatio);

		return cons;

	}


	/**
	 * Determines whether a robot position is legal.
	 * Robot position is measured from the IMU.
	 * @return {Boolean} True if robot position is legal.
	 */
	get isLegalPos() {
		var min_x = Math.min(this.frontLeft.pos.x, this.frontRight.pos.x, this.backLeft.pos.x, this.backRight.x);
		var max_x = Math.max(this.frontLeft.pos.x, this.frontRight.pos.x, this.backLeft.pos.x, this.backRight.x);
		var min_y = Math.min(this.frontLeft.pos.y, this.frontRight.pos.y, this.backLeft.pos.y, this.backRight.y);
		var max_y = Math.max(this.frontLeft.pos.y, this.frontRight.pos.y, this.backLeft.pos.y, this.backRight.y);
		if (min_x <= 0 || min_y <= 0 || max_x >= sandboxLengthCm || max_y >= sandboxWidthCm) {
			return false;
		} else {
			return true;
		}
	}
}


/**
 * Standard Normal variate using Box-Muller transform.
 * @return {number} Random number in the unit variance Gaussian distribution.
 */
function gaussian() {
    var u = 0, v = 0;
    while(u === 0) u = Math.random();
    while(v === 0) v = Math.random();
    return Math.sqrt( -2.0 * Math.log( u ) ) * Math.cos( 2.0 * Math.PI * v );
}


function addNoise(input, ratio) {
	return input + (gaussian() * input * ratio);
}


function reduceAngle(theta) {
	if (theta > Math.PI)
		theta -= 2 * Math.PI;
	if (theta < -Math.PI)
		theta += 2 * Math.PI;
	return theta;
}


/**
 * Generates field and robot dimensions and stores necessary variables.
 */
function setDocSize() {

	// store browser dimensions
	docHeightPx = $(document).height();
	docWidthPx = $(document).width();

	// calculate sandbox width as a proportional of vertical side of browser
	sandboxWidthPx = docHeightPx * SANDBOX_HEIGHT_RATIO;

	// use sandbox ratio to calculate sandbox length
	var dimRatio = sandboxLengthCm / sandboxWidthCm;
	sandboxLengthPx = sandboxWidthPx * dimRatio;

	// draw sandbox on page
	$("#sandbox").css('width', sandboxLengthPx);
	$("#sandbox").css('height', sandboxWidthPx);

	// get margins of sandbox
	sandboxLeftPx = $("#sandbox").offset().left;
	sandboxTopPx = $("#sandbox").offset().top;

	// calculate pixel per metric scale
	scalePxPerCm = sandboxLengthPx / sandboxLengthCm;

	// calculate robot dimensions
	$("#robot").css('width', RobotFrame.WIDTH * scalePxPerCm);
	$("#robot").css('height', RobotFrame.LENGTH * scalePxPerCm);
	$("#shadow").css('width', RobotFrame.WIDTH * scalePxPerCm);
	$("#shadow").css('height', RobotFrame.LENGTH * scalePxPerCm);
	
}


/**
 * Transform robot metric position to pixel position and draw on sandbox.
 * @param  {!State} robot - Robot state to draw.
 * @param  {boolean} actual - The actual robot state, or sensed state if false.
 */
function drawRobot(state, actual) {

	var name = (actual ? "#robot" : "#shadow");
	
	// Robot pixel position uses pre-rotated SVG image dimensions, so it must scale the unrotated front left of
	// robot from the robot state, and scale it to pixels, taking into account the sandbox margin on the left.
	$(name).css('left', (state.x + RobotFrame.FRONT_LEFT.x) * scalePxPerCm + sandboxLeftPx);

	// Since field reference is from the bottom left, and pixel reference is from the top left, the robot state in
	// pixel reference is the vertical dimension of the sandbox minus the state from the bottom. We find the top left
	// of the robot from there and scale it to pixels, taking into account the sandbox margin on the top.
	$(name).css('top', ((sandboxWidthCm - state.y) - RobotFrame.FRONT_LEFT.y) * scalePxPerCm + sandboxTopPx);

	// Robot state theta is positive counterclockwise, but CSS transform is positive clockwise.
	$(name).css('transform', "rotate(" + (-state.theta) + "rad)");

}


$(document).ready(function() {

	// register event to draw robot on field
	$(document).on("drawEvent", function(e, state, actual) {
		drawRobot(state, actual);
	});

	// initialize field and robot size
	setDocSize();

	// randomly initialize true north
	//trueNorth = (Math.random() * 2 * Math.PI) - Math.PI; 
	trueNorth = 0;

	// randomly place robot
	var r = new Robot();


	
});

/** Resize elements when the window is resized. */
$(window).resize(function() {
	setDocSize()
});

$(document).click(function(e) {
	console.log("CM(", (e.pageX - sandboxLeftPx) / scalePxPerCm, ", ", 
		sandboxWidthCm - ((e.pageY - sandboxTopPx) / scalePxPerCm), ") => PX(", 
		e.pageX, ", ", e.pageY, ")");
});

/**
 * Detects whether control buttons have been pressed and records
 * results in global state.
 * @param  {!Event} e - e.keyCode represents the button pressed.
 */
$(document).keydown(function(e) {

	e = e || window.event;

	if (e.keyCode == '38') { upKeyPressed = true;	} 
	else if (e.keyCode == '40') { downKeyPressed = true; }
	else if (e.keyCode == '37') { leftKeyPressed = true; }
	else if (e.keyCode == '39') { rightKeyPressed = true; }
	else if (e.keyCode == '77') { $("#robot").trigger("mirrorEvent"); }

});

/**
 * Detects whether control buttons have been released and records
 * result in global state.
 * @param  {!Event} e - e.keyCode represents the button pressed.
 */
$(document).keyup(function(e) {

	e = e || window.event;

	if (e.keyCode == '38') { upKeyPressed = false;	} 
	else if (e.keyCode == '40') { downKeyPressed = false; }
	else if (e.keyCode == '37') { leftKeyPressed = false; }
	else if (e.keyCode == '39') { rightKeyPressed = false; }

});