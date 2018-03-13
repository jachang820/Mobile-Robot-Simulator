# Mobile Robot Simulation

A mobile robot is equipped with perpendicular laser sensors on the front and right side, an IMU (Invensense MPU-9250), and a ESP-9266 control unit. The control unit has WiFi capability to either connect to a local network or act as an access point, meaning the controls will eventually be passed to it via http requests.

The simulation is for a general rectangular field with randomized compass and randomized starting position (since we're operating under the assumption that we can't exactly control the position of the field). 

## About the simulator

The simulator runs on a Javascript/JQuery front-end, where it performs all its calculations. We will eventually get to the point where we offload important calculations to a Django backend, which will then send and receive http requests to the actual robot.

The simulator keeps an internal state based on the controls:
* UP - move forward
* DOWN - move backward
* LEFT - turn left (forward for right motor and backward for left)
* RIGHT - turn right (forward for left motor and backward for right)
* M - mirror the sensed state (currently, the state estimation can't tell the difference between top right and bottom left)
* C - run calibration
* S - switch off simulation

It also attempts to estimate state using only simulated sensor data. The sensors access the actual state but then inserts some Gaussian noise. The robot, then, only accesses the returned sensor data, inputs (with noise), and assumed dimensions and properties about the robot and constraints with the system.

## Simulation to Actual

The main differences between the simulation and actual trials will be that noise in the actual might be a bit more unpredictable, and also that there will be expected latency in the http requests. The simulation will account for the latency by interpolating past states.

The actual robot must also contain code to set up the WiFi access point and calibrate the sensors and prepare sensor data, so that they are in a readable form. It must then send a hash of sensor data over interval t and expect a response of controls back from the server. This will be coded in C++ with the Arduino IDE.

Instead of polling the robot every 5ms, we could attempt longer intervals by planning a motion pathway and the controls to get there, and sending it to the robot, and the robot then keeps a history of sensor data to pass back to the server. The server performs state estimation, calculates the error, then adjusts the controls to send back to the robot. These iterations repeat. 

But the hope is that once we have a solid software simulator, we can switch the inputs to take commands from the robot instead of generating them locally, and send the controls back to the robot, and it should work decently. So if the simulator is properly prepared, the migration should be swift.

## Authors
**Jonathan Chang** - [jachang820](https://github.com/jachang820)
**Michael Warren**
**Nick Bruce**