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

It also attempts to estimate state using only simulated sensor data. The sensors access the actual state but then inserts some Gaussian noise. The robot, then, only accesses the returned sensor data, inputs (with noise), and assumed dimensions and properties about the robot and constraints with the system.

## Simulation to Actual

The main differences between the simulation and actual trials will be that noise in the actual might be a bit more unpredictable, and also that there will be expected latency in the http requests. The simulation will account for the latency by interpolating past states.

The actual robot must also contain code to set up the WiFi access point and calibrate the sensors and prepare sensor data, so that they are in a readable form. It must then send a hash of sensor data over interval t and expect a response of controls back from the server. This will be coded in C++ with the Arduino IDE.

Instead of polling the robot every 5ms, we could attempt longer intervals by planning a motion pathway and the controls to get there, and sending it to the robot, and the robot then keeps a history of sensor data to pass back to the server. The server performs state estimation, calculates the error, then adjusts the controls to send back to the robot. These iterations repeat. It may help to use rudimentary machine learning to mimize the error between the estimated state and actual state and update controls accordingly.

But the hope is that once we have a solid software simulator, we can switch the inputs to take commands from the robot instead of generating them locally, and send the controls back to the robot, and it should work decently. So if the simulator is properly prepared, the migration should be swift.

## Current state of the simulator

The simulator currently uses various geometric properties of the input and laser sensors to estimate state. Since the sensors are noisy, and the sense interval is 5-10 times slower than the actual state updates, the state estimation is weak. But don't worry, we will improve it with the Extended Kalman Filter and by improving these geometric algorithms (I have a few ideas).

## Getting started

Since it doesn't currently require a server, you can run it in the browser by clicking on 'index.html'. The robot in the simulator will turn around for 1.5 circles to calibrate and learn its surroundings, which include an estimation of the length and width of the box, and its orientation relative to magnetic North. It uses this information to estimate its starting state. You might have to press 'M' in case its estimate is mirrored.

While the starting estimation is decent, it falls apart when you move (and the higher the speed is, the more it diverges). This will be fixed to some extent, but the problem is harder than it first seemed.

## To dos
1. Update the driving algorithm to take into account the change of angular velocity (via the gyroscope), and change in input voltage per each sense interval, t. The driving algorithm currently only calculates path with input voltages and t. The algorithm should divide the interval into 1ms increments and estimate each intermediate state to estimate the path.
2. Keep a running average of 10 or 20 past sensor values to construct a moving average. This slows response time, but greatly reduces the effect of noise.
3. Implement a Kalman filter on the state.
4. Simulate http request delays in the simulator (which will make it worse).
5. Construct a basic server (we have the shell for how to do this from the previous project).
6. Write the Arduino code in the robot.
7. Implement motion planning / path estimation, which is the same process I described in #1, except from the current state toward a desired state, instead of from a past state to the current state.
8. (If necessary) implement a linear regression model that will learn state estimation errors over time and decrease control error.

Numbers 1, 3, 8 are the hardest. Number 7 is hard, except it will basically reuse a lot of 1. However, the hardest part, as always, will be the troubleshooting. Even if we plan everything perfectly, something always goes wrong.

## Authors
**Jonathan Chang** - [jachang820](https://github.com/jachang820)