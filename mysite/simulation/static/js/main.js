$(document).ready(function() {

	// robot state
	var robot = new RobotState();

	// randomly place simulation
	var simbot = new Simulation();

	sandbox.draw();
	draw_robot(simbot.state, false);

	// setup event loop
	var sim_update = setInterval(function() { simulator(simbot); }, simbot.period);
	var state_update = setInterval(function() { state_loop(robot); }, robot.period);
	var helper_update = setInterval(function() { helper_loop(robot, simbot); }, 1000);

});


function helper_loop(robot, simbot) {

	// waste less cpu
	if (!simbot.active) {
		simbot.period = 10000;
		robot.calibrated = false;
	}

	// end of calibration
	if (robot.calibrated) {
		$("#status").hide();
		$("#robot").show();
		$("#sandbox").show();

	}

}
