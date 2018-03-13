from keypoller import KeyPoller

class HTTP_Interface:
	
	# True if new data
	flags = {"input" : False, "sensor" : False }

	# inputs
	controls = ""

	# sensors
	sensors = {}

	def send_controls(left_list, right_list):

		# Convert input into parameter string
		string = ""
		for i in range(len(left_list)):
			string += left_list[i] + "," + right_list[i]
		HTTP_Interface.controls = "?ctrl=" + string[:-1]

		# Set flag
		flags['input'] = True

	def receive_controls():

		# Get inputs if new
		if flags['input'] == True:
			flags['input'] == False
			return HTTP_Interface.controls

		else:
			return None

	def send_sensor_data(dict):

		# Save data from converted json
		HTTP_Interface.sensors = dict;

		# Set flag
		flags['sensors'] = True

	def receive_sensor_data():

		# Get sensors if new
		if flags['sensors'] == True:
			flags['sensors'] = False
			return HTTP_Interface.sensors

		else:
			return None


class FrontEnd_Interface:

	# True if new data
	flags = { "mirror" : False,
						"control" : False,
						"simulate" : False,
						"up" : False,
						"down" : False,
						"left" : False,
						"right" : False,
						"calibrate" : False }

	def set_flag(name):
		flags[name] = True

	def end_calibration():
		flags['calibrate'] = False

	def recieve_inputs():
		# collect data
		up = 1 if flags['up'] else 0
		down = 1 if flags['down'] else 0
		left = 1 if flags['left'] else 0
		right = 1 if flags['right'] else 0
		
		# reset flags
		flags['up'] = False
		flags['down'] = False
		flags['left'] = False
		flags['right'] = False

		return { "left" : up - down + 0.5*(right - left),
						 "right" : up - down + 0.5*(left - right) }

	def recieve_options(name):
		val = flags[name]
		if not name == "calibrate":
			flags[name] = False
		return val

	def listen():
		c = KeyPoller().poll()
		if not c is None:
			if c == "c":
				flags['control'] = True
			elif c == "s":
				flags['simulate'] = True
			elif c == "m":
				flags['mirror'] = True
			else:
				print(c)
		