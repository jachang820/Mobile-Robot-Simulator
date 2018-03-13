class Box {
	constructor(width=80, length=100) {
		this.width_cm = width;
		this.length_cm = length;
		this.seen = false;
	}

	get width_px() {
		return this.width_cm * this.scale_px_per_cm;
	}

	set width_px(px) {
		this.width_cm = px / this.scale_px_per_cm;
	}

	get length_px() {
		return this.length_cm * this.scale_px_per_cm;
	}

	set length_px(px) {
		this.length_cm = px / this.scale_px_per_cm; 
	}

	/**
 	* Generates field and robot dimensions and stores necessary variables.
 	*/
	draw(width_cm=null, length_cm=null) {

		if (width_cm == null || length_cm == null) {
			var width_cm = this.width_cm;
			var length_cm = this.length_cm;
		}

		// store browser dimensions
		var doc_height_px = $(window).height();
		var doc_width_px = $(window).width();

		// calculate sandbox width as a proportional of vertical side of browser
		var width_px = doc_height_px * 0.95;

		// use sandbox ratio to calculate sandbox length
		var dim_ratio = length_cm / width_cm;
		var length_px = width_px * dim_ratio;

		// draw sandbox on page
		$("#sandbox").css('width', length_px);
		$("#sandbox").css('height', width_px);
		$("#sandbox").offset({top: 10, left: 10});

		// get margins of sandbox
		this.left_px = $("#sandbox").offset().left;
		this.top_px = $("#sandbox").offset().top;

		// calculate pixel per metric scale
		this.scale_px_per_cm = length_px / length_cm;

		// calculate robot dimensions
		$("#robot").css('width', Robot.WIDTH * this.scale_px_per_cm);
		$("#robot").css('height', Robot.LENGTH * this.scale_px_per_cm);
		$("#shadow").css('width', Robot.WIDTH * this.scale_px_per_cm);
		$("#shadow").css('height', Robot.LENGTH * this.scale_px_per_cm);

		this.width_cm = width_cm;
		this.length_cm = length_cm;
	}

}



/**
 * Transform robot metric position to pixel position and draw on sandbox.
 * @param  {!State} robot - Robot state to draw.
 * @param  {boolean} actual - The actual robot state, or simulated state if false.
 */
function draw_robot(state, actual) {

	var name = (actual ? "#robot" : "#shadow");
	
	// Robot pixel position uses pre-rotated SVG image dimensions, so it must scale the unrotated front left of
	// robot from the robot state, and scale it to pixels, taking into account the sandbox margin on the left.
	$(name).css('left', (state.x + Robot.FRONT_LEFT.x) * sandbox.scale_px_per_cm + sandbox.left_px);

	// Since field reference is from the bottom left, and pixel reference is from the top left, the robot state in
	// pixel reference is the vertical dimension of the sandbox minus the state from the bottom. We find the top left
	// of the robot from there and scale it to pixels, taking into account the sandbox margin on the top.
	$(name).css('top', ((sandbox.width_cm - state.y) - Robot.FRONT_LEFT.y) * sandbox.scale_px_per_cm + sandbox.top_px);

	// Robot state theta is positive counterclockwise, but CSS transform is positive clockwise.
	$(name).css('transform', "rotate(" + (-state.theta) + "rad)");

}


/** Resize elements when the window is resized. */
$(window).resize(function() {
	if (sandbox.seen) {
		sandbox.draw();
	}
});

$(document).click(function(e) {

	e = e || window.event;

	var x_cm = (e.pageX - sandbox.left_px) / sandbox.scale_px_per_cm;
	var y_cm = sandbox.width_cm - (e.pageY - sandbox.top_px) / sandbox.scale_px_per_cm;

	if (x_cm > 0 && x_cm < sandbox.length_cm && 
			y_cm > 0 && y_cm < sandbox.width_cm) {
				fend_IF.send_click(x_cm, y_cm);
	}

});

/**
 * Detects whether control buttons have been pressed and records
 * results in global state.
 * @param  {!Event} e - e.keyCode represents the button pressed.
 */
$(document).keydown(function(e) {

	e = e || window.event;

	if (e.keyCode == '38') { fend_IF.set_flag("up"); } 
	else if (e.keyCode == '40') { fend_IF.set_flag("down"); }
	else if (e.keyCode == '37') { fend_IF.set_flag("left"); }
	else if (e.keyCode == '39') { fend_IF.set_flag("right"); }
	else if (e.keyCode == '77') { fend_IF.set_flag("mirror"); }
	else if (e.keyCode == '83') { 
		fend_IF.set_flag("simulate");
		$("#shadow").hide();
	}
	else if (e.keyCode == '67') { fend_IF.set_flag("calibrate"); }

});

/**
 * Detects whether control buttons have been released and records
 * result in global state.
 * @param  {!Event} e - e.keyCode represents the button pressed.
 */
$(document).keyup(function(e) {

	e = e || window.event;

	if (e.keyCode == '38') { fend_IF.unset_flag("up");	} 
	else if (e.keyCode == '40') { fend_IF.unset_flag("down"); }
	else if (e.keyCode == '37') { fend_IF.unset_flag("left"); }
	else if (e.keyCode == '39') { fend_IF.unset_flag("right"); }

});



sandbox = new Box();