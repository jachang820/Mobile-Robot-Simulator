#include <ESP8266WiFi.h>
#include <Servo.h>
#include <string.h>
#include <Arduino.h>
#include <vector>
#include <BasicLinearAlgebra.h>
#include <Wire.h>
#include <VL53L0X.h>

#define LEFT_SERVO                  D0
#define RIGHT_SERVO                 D1

#define SDA_PORT                    14
#define SCL_PORT                    12

#define MPU9250_ADDRESS             0x68
#define MAG_ADDRESS                 0x0C

#define GYRO_FULL_SCALE_250_DPS     0x00  
#define GYRO_FULL_SCALE_500_DPS     0x08
#define GYRO_FULL_SCALE_1000_DPS    0x10
#define GYRO_FULL_SCALE_2000_DPS    0x18

#define ACC_FULL_SCALE_2_G          0x00  
#define ACC_FULL_SCALE_4_G          0x08
#define ACC_FULL_SCALE_8_G          0x10
#define ACC_FULL_SCALE_16_G         0x18

#define ENV_WIDTH                   608
#define ENV_HEIGHT                  401
#define ENV_PHI                     PI / 2

#define LIN_VEL                     120.5
#define ANGULAR_VEL                 2.618

int count = 0;

// Wifi settings
const char * SSID = "Linksys00292";
const char * PASSWORD = "1fbjgxdtpv";

WiFiServer server(80);

void setup() {

  Serial.begin(9600);
  Serial.println("Begin!");
  
  // login to existing wifi network as station
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
  }

  // report status
  Serial.print("Connected to ");
  Serial.println(SSID);
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  server.begin();

  Wire.begin(SDA_PORT,SCL_PORT);
  Serial.begin(115200);
  setupPins();

  currentState(0) = ENV_WIDTH / 2;
  currentState(1) = ENV_HEIGHT / 2;
  currentState(2) = 0;
}

void loop() {
  
  // check to see if server is still running
  WiFiClient client = server.available();
  if (!client) {
    return;
  }

  // read first line of request
  String request = client.readStringUntil('\r');
  if (request.length() == 0) {
    return;
  }

  // tokenize the request string to get the key
  char * str = new char [request.length()+1];
  strcpy (str, request.c_str());
  char * pch = strtok(str, "/");
  char * result = new char[1024];

  // skip the first token (GET)
  pch = strtok(NULL, "/");
  if (pch == "cal") {
    
    float theta = atof(strtok(NULL, "/"));
    float dist = atof(strtok(NULL, "/"));
    
    // run calibration routine
    result = calibrate(theta, dist);
    
  } else if (pch == "tar") {

    // run motion planning routine
    result = motionplanning();
    
  }

  // respond
  client.flush();
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/json");
  client.println("Connection: Keep-Alive");
  client.println("");
  client.print(result);
    
  }
}

char * calibrate() {

  return;
}

char * motionplanning(float theta, float dist) {
  rrt();
  return;
}


void rrt() {

  BLA::Matrix<3, 1> goalState = readGoalState();
  //goalState = getGoalStateFromServer();

  // Create the RRT: assuming currentState is the state set up in setup()
  std::vector<vertex> tree = createRRT(ENV_WIDTH, ENV_HEIGHT, currentState, goalState, 20);
  Serial.print("Generated the RRT with size: ");
  Serial.println(tree.size());
  
  // Find the path
  std::vector<short> path = getPath(tree);

  Serial.print("Found the path of size: ");
  Serial.println(path.size());

  // Drive along that path
  int pathSize = path.size();
  for (int i = 0; i < pathSize; i++)
  {
    vertex currentVertex = tree[path[i]];
    short x = currentVertex.x;
    short y = currentVertex.y;

    driveToPosition(x, y);
  }

  // Rotate the car into our goal state angle
  rotateTo(goalState[2]);
 
//  BLA::Matrix<3, 1> input = { 90, 90, 100 };
//  BLA::Matrix<4, 1> actualSensorValues = readSensorValues();
//  currentState = estimator.estimateState(currentState, input, actualSensorValues);

//  while(!Serial.available())
//    delay(1000);
//
//  while(Serial.available())
//    Serial.read();
//  
//  std::vector<BLA::Matrix<3, 1>> inputList;
////  inputList.push_back({ 180, 0, 1500 }); // Go forwad
////  inputList.push_back({ 180, 180, 450 }); // Rotate 90 degrees CW
////  inputList.push_back({ 180, 0, 1500 }); // Go forward
////  inputList.push_back({ 0, 0, 550 }); // Rotate 90 CCW 
////  inputList.push_back({180, 0, 1500 });
//
//  inputList.push_back({ 180, 0, 1500 }); // Go forwad
//  inputList.push_back({ 0, 0, 450 }); // Rotate 90 degrees CCW
//  inputList.push_back({ 180, 0, 1500 }); // Go forward
//  inputList.push_back({ 0, 0, 450 }); // Rotate 90 CCW 
//  inputList.push_back({180, 0, 1500 }); // Go forward
//
//  int numInputs = inputList.size();
//  for(int i = 0; i < numInputs; i++)
//  {
//    BLA::Matrix<3, 1> currentInput = inputList[i];
//    applyInput(currentInput);
//    delay(150);
//  }
  
  
  delay(3000);
}

void setupPins()
{
  // set up the servos
  servo_left.attach(LEFT_SERVO);
  servo_right.attach(RIGHT_SERVO);
  Serial.println("Servos set up...");
  Serial.flush();

  delay(1000);

  // Set up the range sensors
  pinMode(D3, OUTPUT);
  pinMode(D4, OUTPUT);
  digitalWrite(D7, LOW);
  digitalWrite(D8, LOW);

  delay(500);

  digitalWrite(D3, HIGH);
  delay(150);
  
  front_sensor.init(true);
  delay(100);
  front_sensor.setAddress((uint8_t)22);

  digitalWrite(D4, HIGH);
  delay(150);
  right_sensor.init(true);
  delay(100);
  right_sensor.setAddress((uint8_t)25);

  Serial.println("Range sensors set up...");
  Serial.flush();

  delay(1000);
  
  // setup magnetometer
  I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
  delay(500);
  Serial.println("Magnetometer set up...");

  // confirm sensor detections
  Serial.println ("I2C scanner. Scanning ...");
  byte count = 0;

  for (byte i = 1; i < 120; i++)
  {

    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0)
    {
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);
      Serial.println (")");
      count++;
      delay (1);  // maybe unneeded?.
    } // end of good response
  } // end of for loop
  Serial.println ("Done.");
  Serial.print ("Found ");
  Serial.print (count, DEC);
  Serial.println (" device(s).");

  delay(1000);
}

void setupServer()
{
  Serial.print("Server is setup at addr: ");
  //Serial.println(IPAddress);
}

void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}

std::vector<vertex> createRRT(const int width, const int height, const BLA::Matrix<3, 1>& initState, const BLA::Matrix<3, 1>& goalState, const int threshold)
{
  std::vector<vertex> tree;
  
  // The maximum distance to increase a tree branch
  short delta = 50;
  
  vertex startVertex = { initState(0), initState(1), -1 };
  tree.push_back(startVertex);

  float newVertexDist = (initState(0) - goalState(0)) * (initState(0) - goalState(0)) + (initState(1) - goalState(1)) * (initState(1) - goalState(1));
  while (newVertexDist > threshold * threshold)
  {
      // Generate a random vertex within the boundaries of the environment   
      vertex rv = { random(50, width - 50), random(50, height - 50), -1 };

      // Find the closest vertex to this new random vertex
      int closestDist = (width * width) + (height * height) + 10000;
      short closestVertexID = -1;
      for (int i = 0; i < tree.size(); i++)
      {
          vertex v = tree[i];
          
          int distSquared = (rv.x - v.x) * (rv.x - v.x) + (rv.y - v.y) * (rv.y - v.y);
          if(distSquared < closestDist)
          {
            closestDist = distSquared;
            closestVertexID = i;
          }
      }

      // If the closest dist is within delta squared, just add the new random vertex to tree
      // otherwise, add a vertex of delta length towards rand vertex
      vertex newVertex = { rv.x, rv.y, closestVertexID };
      if(closestDist > delta * delta)
      {
        vertex cv = tree[closestVertexID];
        float angle = atan2( (rv.y - cv.y) , (rv.x - cv.x) );
        newVertex.x = cv.x + delta * cos(angle);
        newVertex.y = cv.y + delta * sin(angle);
      }

      tree.push_back(newVertex);
      newVertexDist = (newVertex.x - goalState(0)) * (newVertex.x - goalState(0)) + (newVertex.y - goalState(1)) * (newVertex.y - goalState(1));
  }

  return tree;
}


std::vector<short> getPath(const std::vector<vertex>& tree)
{
  std::vector<short> path;
  path.insert(path.begin(), tree.size() - 1);
  
  short parentID = tree[tree.size() - 1].parentID;
  while (parentID != -1)
  {
    path.insert(path.begin(), parentID);
    parentID = tree[parentID].parentID;
  }

  return path;
}

BLA::Matrix<4, 1> readSensorValues()
{
  int16_t values[3];
  readMagnetometerValues(values);

  BLA::Matrix<4, 1> sensorValues = { readRangeSensor(true), readRangeSensor(false), values[0], values[1] };
  return sensorValues;
}

void readMagnetometerValues(int16_t* values)
{
  // Request first magnetometer single measurement
  I2CwriteByte(MAG_ADDRESS,0x0A,0x01);
  
  // Read register Status 1 and wait for the DRDY: Data Ready
  uint8_t ST1;
  do
  {
    I2Cread(MAG_ADDRESS,0x02,1,&ST1);
  }
  while (!(ST1&0x01));

  // Read magnetometer data  
  uint8_t Mag[7];  
  I2Cread(MAG_ADDRESS,0x03,7,Mag);

  // Create 16 bits values from 8 bits data  
  // Magnetometer
  values[0] = (Mag[1]<<8 | Mag[0]) - bias[0];
  values[1] = (Mag[3]<<8 | Mag[2]) - bias[1];
  values[2] = (Mag[5]<<8 | Mag[4]) - bias[2];
}

short readRangeSensor(bool forward)
{
  return forward ? front_sensor.readRangeSingleMillimeters() : right_sensor.readRangeSingleMillimeters() - 30;
}

void driveToPostition(int x, int y)
{
  float range = 10;
  
  float dx = x - currentState(0);
  float dy = y - currentState(1);
  float dist = sqrt(dx * dx + dy * dy);
  
  while(dist > range)
  {
      // for every iteration: 
      // 1.) Rotate if off course (decision made in rotate function)
      // 2.) Approximate the needed duration to reach the destination driving forward
      // 3.) Apply the forward input for the set amount of time.
      // 4.) Read the sensors and update the current state with the state estimator.

      float desiredAngle = atan2( y - currentState(1), x - currentState(0) );
      rotateTo(desiredAngle);
      
      float timeNeeded = 1000 * dist / LIN_VEL;
      BLA::Matrix<3, 1> forwardInput = { 180, 0, timeNeeded };
      applyInput(forwardInput);

      BLA::Matrix<4, 1> csv = readSensorValues();
      currentState = estimator.estimateState(currentState, forwardInput, csv);
  }
}

BLA::Matrix<3, 1> readGoalState()
{
  int x = 100;
  int y = 100;
  int theta = 0;
  
  while(!Serial.available())
    delay(1000);

  x = readSerialInt();
  y = readSerialInt();
  theta = readSerialInt();
  
  return { x, y, theta };
}

int readSerialInt()
{
  String inString = "";
  
  while (Serial.available() > 0) 
  {
    int inChar = Serial.read();
    if(inChar == '\n' || inChar == ' ')
      break;
      
    if (isDigit(inChar)) 
      inString += (char)inChar;
  }

  return inString.toInt();  
}

void rotateTo(float angle)
{
  float currentAngle = currentState(2);
  float range = 5 * PI / 180;

  float diff = abs(currentAngle - angle);
  while(diff > range)
  {
    bool ccw = angle > currentAngle;
    BLA::Matrix<3, 1> rotationInput;

    if(ccw) 
    { 
      rotationInput(0) = 0; 
      rotationInput(1) = 0; 
    }
    else    
    { 
      rotationInput(0) = 180;
      rotationInput(1) = 180;
    }

    float timeNeeded = 1000 * diff / ANGULAR_VEL;
    rotationInput(2) = timeNeeded;
    applyInput(rotationInput);

    BLA::Matrix<4, 1> csv = readSensorValues();
    currentState = estimator.estimateState(currentState, rotationInput, csv);

    diff = abs(currentState(2) - angle);
  }
}

void applyInput(const BLA::Matrix<3, 1>& input)
{
  servo_left.write(input(0));
  servo_right.write(input(1));
  
  delay(input(2));

  servo_left.write(90);
  servo_right.write(90);
}


// STATE ESTIMATOR FUNCTIONS
BLA::Matrix<3, 1> StateEstimator::estimateState(const BLA::Matrix<3, 1>& cps, const BLA::Matrix<3, 1>& input, const BLA::Matrix<4, 1>& actualSensorValues)
{
  // predict state estimate
  BLA::Matrix<3, 1> state_pred = f(cps, input);
  
  // predict covariance estimate
  float vel = 0;
  if(input(0) == 180 && input(1) == 0)
    vel = LIN_VEL;
  else if(input(0) == 0 && input(1) == 180)
    vel = -LIN_VEL;

  float theta = state_pred(2);
  float dt = input(2);
  BLA::Matrix<3, 3> F = { 0, 0, -vel * sin(theta) * dt,
                          0, 0,  vel * cos(theta) * dt,
                          0, 0,           0             };
 
  BLA::Matrix<3, 3> P_pred = F * (P * ~F) + Q;

  // measurement residual
  BLA::Matrix<4, 1> sensor_pred = h(state_pred);
  BLA::Matrix<4, 1> error = actualSensorValues - sensor_pred;
  
  // residual covariance
  float x = state_pred(0);
  float y = state_pred(1);  
  float t = input(3);
  
  float x_prime = x - 25*cos(theta);
  float y_prime = y - 25*sin(theta);
  float phi = ENV_PHI;
  float W = ENV_WIDTH;
  float H = ENV_HEIGHT;
  float Mx_max = 32;
  float My_max = 31;
  
  float d_f_top_wall = (H-y)/(sin(theta)) - 25;           //front sensor 
  float d_r_top_wall = -(H-y_prime) / cos(theta) - 30;
  
  float d_f_right_wall = (ENV_WIDTH-x)/(cos(theta)) - 25;
  float d_r_right_wall = (ENV_WIDTH-x_prime)/(sin(theta)) - 30;
  
  float d_f_left_wall = (-1*x/(cos(theta))) - 25;
  float d_r_left_wall = (-x_prime/sin(theta)) - 30;
  
  float d_f_bottom_wall = (-1*y/(sin(theta))) - 25;
  float d_r_bottom_wall = (y_prime/cos(theta)) -30;

  bool  DF_top = false;                                              //determining if the sensor is facing top or right wall
  bool  DF_right = false; 
  bool  DF_left = false; 
  bool  DF_bottom = false; 
  
  bool  DR_top = false;
  bool  DR_right = false; 
  bool  DR_left = false; 
  bool  DR_bottom = false; 
  
  if(theta >= 0 && theta <= PI/2){
    DF_top = d_f_top_wall <= d_f_right_wall;             // determining if the sensor is facing top or right 
    DF_right =  d_f_top_wall > d_f_right_wall;
    
    DR_right = d_r_right_wall <= d_r_bottom_wall;
    DR_bottom = d_r_bottom_wall < d_r_right_wall;
  }
  
  if(theta >= PI/2 && theta <= PI){
    DF_top = d_f_top_wall <= d_f_left_wall; 
    DF_left = d_f_left_wall < d_f_top_wall;
    
    DR_top = d_r_top_wall <= d_r_right_wall;
    DR_right = d_r_top_wall > d_r_right_wall; 
    
  }
  
  if(theta >= PI && theta <= 3*PI/2){
    DF_bottom = d_f_bottom_wall <= d_f_left_wall; 
    DF_left = d_f_left_wall < d_f_bottom_wall;
    
    DR_top = d_r_top_wall <= d_r_left_wall;
    DR_left = d_r_left_wall < d_r_top_wall; 
    
  }
  
  if(theta >= 3*PI/2 && theta <= 2*PI){
    DF_bottom = d_f_bottom_wall <= d_f_right_wall; 
    DF_right = d_f_right_wall < d_f_bottom_wall;
    
    DR_bottom = d_r_bottom_wall <= d_r_left_wall;
    DR_left = d_r_left_wall < d_r_bottom_wall; 
    
    }

    float H11 = 0;
    float H12 = 0;
    float H13 = 0;

    float H21 = 0;
    float H22 = 0;
    float H23 = 0;
    
    if(DF_top)
    {
      H11 = 0;
      H12 = - 1 / sin(theta);
      H13 = -(H - y) * cos(theta) / (sin(theta) * sin(theta));
    }
    else if(DF_right)
    {
      H11 = -1 / cos(theta);
      H12 = 0;
      H13 = (W - x) * sin(theta) / (cos(theta) * cos(theta));
    }
    else if(DF_bottom)
    {
      H11 = 0;
      H12 = - 1 / sin(theta);
      H13 = y * cos(theta) / (sin(theta) * sin(theta));
    }
    else if(DF_left)
    {
      H11 = -1 / cos(theta);
      H12 = 0;
      H13 = -x * sin(theta) / (cos(theta) * cos(theta));
    }

    if(DR_top)
    {
      H21 = 0;
      H22 = 1 / cos(theta);
      H23 = ( -(H - y ) * sin(theta) - 25) / (cos(theta) * cos(theta));
    }
    else if(DR_right)
    {
      H21 = -1 / sin(theta);
      H22 = 0;
      H23 = ( -(W - x) * cos(theta) - 25) / (sin(theta) * sin(theta));
    }
    else if(DR_bottom)
    {
      H21 = 0;
      H22 = 1 / cos(theta);
      H23 = (y * sin(theta) - 25) / (cos(theta) * cos(theta));
    }
    else if(DR_left)
    {
        H21 = - 1 / sin(theta);
        H22 = 0;
        H23 = (x * cos(theta) - 25) / (sin(theta) * sin(theta));
    }

    float H33 = -Mx_max * cos(theta - phi);
    float H43 = -My_max * sin(theta - phi);
    
    BLA::Matrix<4,3> jacobian = { H11, H12, H13,
                                  H21, H22, H23,
                                  0  , 0  , H33,
                                  0  , 0  , H43 };

    BLA::Matrix<4, 3> Hk = jacobian;
    BLA::Matrix<4, 4> S = Hk * (P_pred * ~Hk) + R;

    // near optimal Kalman gain
    BLA::Matrix<3, 4> K = P_pred * (~Hk * S.Inverse());
  
    // updated state estimate
    BLA::Matrix<3, 1> estimatedState = state_pred + K * error;
  
    if(estimatedState(2) >= 2*PI)
      estimatedState(2) -= 2*PI;
    else if(estimatedState(2) < 0)
      estimatedState(2) += 2*PI;
  
    // updated covariance estimate
    P = (I - K * Hk) * P_pred;
    return estimatedState;
}

BLA::Matrix<3, 1> StateEstimator::f(const BLA::Matrix<3, 1>& currentState, const BLA::Matrix<3, 1>& input)
{ 
  float dt = input(2) / 1000;
  float vel = 0;
  if(input(0) == 180 && input(1) == 0)
    vel = LIN_VEL;
  else if(input(0) == 0 && input(1) == 180)
    vel = -LIN_VEL;

  float newX = currentState(0) + vel*cos(currentState(2)) * dt;
  float newY = currentState(1) + vel*sin(currentState(2)) * dt;

  float newTheta = currentState(2);
  if(input(0) == input(1))
  {
    if(input(0) == 0)
      newTheta += dt * ANGULAR_VEL;
    else if(input(0) == 180)
      newTheta -= dt * ANGULAR_VEL;
  }

  BLA::Matrix<3, 1> nps = { newX, newY, newTheta };
  return nps;
}

BLA::Matrix<4, 1> StateEstimator::h(const BLA::Matrix<3, 1>& predictedState)
{
  const int W = ENV_WIDTH;
  const int H = ENV_HEIGHT;
  const float PHI = ENV_PHI;
  
  float x = predictedState(0);
  float y = predictedState(1);
  float theta = predictedState(2);

  float dr = 0, df = 0;
  
  float t_t = (H - y) / sin(theta);
  float t_r = (W - x) / cos(theta);
  float t_b = - y / sin(theta);
  float t_l = - x / cos(theta);

  float t_t2 = -(H - ( y - 25*sin(theta) )) / cos(theta);
  float t_r2 = (W - ( x - 25*cos(theta) )) / sin(theta);
  float t_b2 =  ( y - 25*sin(theta) ) / cos(theta);
  float t_l2 =  -( x - 25*cos(theta) ) / sin(theta);

  if (theta >= 0 && theta < PI/2)
  {
      df = min( t_t, t_r ) - 25;
      dr = min( t_b2, t_r2 ) - 30;
  }
  else if (theta >= PI/2 && theta < PI)
  {
      df = min(t_t, t_l) - 25;
      dr = min(t_t2, t_r2) - 30;
  }
  else if (theta >= PI && theta < 3*PI/2)
  {
      df = min(t_l, t_b) - 25;
      dr = min(t_t2, t_l2) - 30;
  }
  else if (theta >= 3*PI/2 && theta < 2*PI)
  {
      df = min(t_b, t_r) - 25;
      dr = min(t_l2, t_b2) - 30;
  }

  float mx =  32*sin(theta - PHI);
  float my =  31*cos(theta - PHI);

  BLA::Matrix<4, 1> psv = { df, dr, mx, my };
  return psv;
}