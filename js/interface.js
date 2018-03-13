class HTTP_Interface {
	
	constructor() {
		this.flags = { sim_input: false, actual_input: false, sensor: false, calibrate: false };
		this.controls = "";
		this.sensors = {front: [], right: [], mag: [], gyro: [], accel: []};
		this.ajax = false;
		this.response = false;
	}

	send_controls(left_list, right_list) {
		var string = "";
		for (var i = 0; i < left_list.length; i += 1) {
			let left_sign = (left_list[i] >= 0 ? "+" : "-");
			let right_sign = (right_list[i] >= 0 ? "+" : "-");
			left_list[i] = String(Math.abs(left_list[i])).padStart(3, "0").slice(0, 3);
			right_list[i] = String(Math.abs(right_list[i])).padStart(3, "0").slice(0, 3);
			string += left_sign + left_list[i] + right_sign + right_list[i]
		}

		this.controls = "ctrl/" + string + "/";
		this.flags.sim_input = true;
		this.flags.actual_input = true;
	}


	// this will never be sent to the actual robot
	// prefer 'target' mechanism
	receive_controls(is_sim=true) {
		if (is_sim && this.flags.sim_input) {
			this.flags.sim_input = false;
			return this.controls;
		} else {
			return null;
		}
	}


	send_target(r, theta, is_sim) {
		this.controls = "tar/" + String(r) + "/" + String(theta) + "/";
		
		if (is_sim == true) {
			this.flags.sim_input = true;
		} else {
			this.flags.actual_input = true;
		}
	}

	// actual input would call ajax to get input
	receive_target(is_sim) {
		if (is_sim && this.flags.sim_input) {
			this.flags.sim_input = false;
			return this.target;
		} else {
			return null;
		}
	}


	send_sensor_data(dict, replace=true) {
		if (replace == true) {
			this.sensors = dict;
		} else {
			this.sensors.front.concat(dict.front);
			this.sensors.right.concat(dict.right);
			this.sensors.mag.concat(dict.mag);
			this.sensors.gyro.concat(dict.gyro);
			this.sensors.accel.concat(dict.accel);
		}
		this.flags.sensors = true;
	}


	receive_sensor_data() {
		if (this.flags.sensors = true) {
			let oldest = {
				front : this.sensors.front.shift(),
				right : this.sensors.right.shift(),
				mag   : this.sensors.mag.shift(),
				gyro  : this.sensors.gyro.shift(),
				accel : this.sensors.accel.shift()
			};

			if (this.sensors.front.length == 0) {
				this.flags.sensors = false;
			}
			return oldest;
		} else {
			return null;
		}
	}


	receive_calibration_data() {
		if (this.flags.calibration == true) {
			return {
				width: this.sensors.width,
				length: this.sensors.length,
				orient: this.sensors.orient,
				Q: this.sensors.Q,
				R: this.sensors.R
			};
		} else {
			return null;
		}
	}


	clear_sensor_data() {
		this.sensors =  {front: [], right: [], mag: [], gyro: [], accel: []};
	}

	send_calibrate_signal() {
		this.flags.calibrate = true;
		this.controls = "cal/";
	}

	ajax_controls() {

		var payload = JSON.stringify({
			controls: this.controls
		});

		this.ajax = true;
		this.response = false;

		fetch('/simulation/jsonhandler/', {
			method: 'POST',
			body: payload,
			header: {
				"X-CSRFToken" : "CSRF_COOKIE",
				"Content-Type" : "application/json"
			},
			credentials: 'include',
			mode: 'cors',
			cache: 'default',
		}).then(function(response) {
			if (response.status !== 200) {
				console.log("There was a problem with ajax. " +
					response.status);
				return;
			}

			// not sure the format of 'data'
			response.json().then(function(data) {
				this.send_sensor_data(data, false);
				this.response = true;
				this.ajax = false;
			});
		});

	}
}

class FrontEnd_Interface {
	
	constructor() {

		this.flags = {
			mirror : false,
			simulate : false,
			up : false,
			down : false,
			left : false,
			right : false,
			point : false,
			calibrate: false 
		};

		this.target = new Position();
		this.simul_off = false;

	}


	set_flag(name) {
		this.flags[name] = true;
	}


	unset_flag(name) {
		this.flags[name] = false;
	}


	receive_inputs() {

		var up = this.flags.up ? 1 : 0;
		var down = this.flags.down ? 1 : 0;
		var left = this.flags.left ? 1 : 0;
		var right = this.flags.right ? 1 : 0;

		return { 
			left  : up - down + 0.5*(right - left),
			right : up - down + 0.5*(left - right) 
		};
	}


	receive_options(name) {

		// invalid flags for this call
		if (name == "up" || name == "down" || 
				name == "left" || name == "right") {
					return null;
		}

		var val = this.flags[name];
		this.flags[name] = false;
	
		return val;
	}


	simulation_off() {
		this.simul_off = true;
	}


	send_click(x, y) {
		this.flags['point'] = true;
		this.target.x = x;
		this.target.y = y;
	}

	receive_click() {
		if (this.flags['point'] == true) {
			return this.target;
		} else {
			return null;
		}
		this.flags['point'] = false;
	}
}


http_IF = new HTTP_Interface();
fend_IF = new FrontEnd_Interface();