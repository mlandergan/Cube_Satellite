/*
  Written by Mark Landergan
  WPI RBE student 2019
*/


#include <Wire.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Declare sensor and motor objects
Adafruit_BNO055 bno = Adafruit_BNO055(55);
Servo motorX, motorY, motorZ; // Create as much as Servoobject you want. You can controll 2 or more Servos at the same time

// Declare constants and variables
const int escInitValue = 2000; // max uS value to arm ESC
const int stopMotors = 1060;
const int throttle = 1150 ;

float elapsedTime, time, timePrev;

float PIDX, PIDY, PIDZ, pwmX, pwmY, pwmZ;
float errorX, errorY, errorZ, previous_errorX, previous_errorY, previous_errorZ;

float x_pid_p = 0;
float x_pid_i = 0;
float x_pid_d = 0;
float y_pid_p = 0;
float y_pid_i = 0;
float y_pid_d = 0;
float z_pid_p = 0;
float z_pid_i = 0;
float z_pid_d = 0;
const float desiredAngleXY = 0; //This is the angle in which we want the balance to stay steady
float desiredAngleZ = 0; // will be read in from serial

// PID CONSTANTS
// constants for motorX
const double x_kp = 1; //3.55
const double x_ki = 0; //0.003
const double x_kd = 0; //2.05

// constants for motorY
const double y_kp = 3.55; //3.55
const double y_ki = 0.005; //0.003
const double y_kd = 2.05; //2.05

// constants for motorZ
const double z_kp = 3.55; //3.55
const double z_ki = 0.005; //0.003
const double z_kd = 2.05; //2.05


void setup(void)
{
  Serial.begin(9600);
  motorX.attach(9); // Brushless motor for x axis connected to digital pin 9
  motorY.attach(10); // Brushless motor for y axis connected to digital pin 10
  motorZ.attach(11); // Brushless motor for z axis connected to digital pin 11

  if (!bno.begin()) // Initialise the sensor
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  //motorX.writeMicroseconds(escInitValue); // set the motors to 0 RPM to start
  //delay(3000);

  bno.setExtCrystalUse(true);
  motorX.writeMicroseconds(stopMotors); // set the motors to 0 RPM to start
  motorY.writeMicroseconds(stopMotors);
  motorZ.writeMicroseconds(stopMotors);
  delay(1000);

}

/*
  Takes in a sensor event contatining IMU data
  Displays the floating point data to serial monitor
*/
void displayOrientation(sensors_event_t event) {
  Serial.print("X: ");
  Serial.print(event.orientation.z, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.x, 4);
  Serial.println("");
  delay(100);
}

void loop(void)
{
  /*
     The timeStep is the time that elapsed since the previous loop.
     This is the value that we will use in the formulas as "elapsedTime"
     in seconds. We work in ms so we haveto divide the value by 1000
     to obtain seconds
  */
  timePrev = time;  // the previous time is stored before the actual time read
  time = millis();  // actual time read
  elapsedTime = (time - timePrev) / 1000;

  // IMU reading stored in a sensor event
  sensors_event_t event;
  bno.getEvent(&event); // Get a new sensor event
  displayOrientation(event);

  /* PID calculations
     first calculate the error between current orientation reading and desiered angles
     x and y have a goal orientation of 0 degrees
     z has a variable goal orientation read in via user input
  */
  errorX = event.orientation.z - desiredAngleXY; // event.orientation.x is a float may need to cast, using z field due to way IMU is mounted
  Serial.print("Error in X: ");
  Serial.println(errorX);
  errorY = event.orientation.y - desiredAngleXY;
  errorZ = event.orientation.x - desiredAngleZ; // x and z frames switched, using x field due to way IMU is mounted

  /* Proportional term calculation
     proportional value of the PID is just a proportional constant
     multiplied by the error
  */
  x_pid_p = x_kp * errorX;
  y_pid_p = y_kp * errorY;
  z_pid_p = z_kp * errorZ;

  /* Integral term calculation
     Should only come into play if we are close to the desired positon but want to fine tune the
     error. If we are within 3 degrees of error than we will use this integral term. Also will help
     avoid integral wind up.
  */
  if (-3 < errorX < 3)
  {
    x_pid_i = x_pid_i + (x_ki * errorX);
  }
  if (-3 < errorY < 3)
  {
    y_pid_i = y_pid_i + (y_ki * errorY);
  }
  if (-3 < errorZ < 3)
  {
    z_pid_i = z_pid_i + (z_ki * errorZ);
  }

  /* errorWithinDegree takes in an error term (float)and a range term (int)
     will only

  */

  }

/* Derivative term
  The derivative term flattens the error trajectory into a horizontal line, damping the force applied
*/
x_pid_d = x_kd * ((errorX - previous_errorX) / elapsedTime);
  y_pid_d = y_kd * ((errorY - previous_errorY) / elapsedTime);
  z_pid_d = z_kd * ((errorZ - previous_errorZ) / elapsedTime);

  /* PID values
    the sume of the proportional, integral and derivative terms
  */
  PIDX = x_pid_p + x_pid_i + x_pid_d;
  Serial.print("PID X: ");
  Serial.println(PIDX);
  PIDY = y_pid_p + y_pid_i + y_pid_d;
  PIDZ = z_pid_p + z_pid_i + z_pid_d;

  // constrain the PID
  if (PIDX < -1000)
  {
    PIDX = -20;
  }
  if (PIDX > 1000)
  {
    PIDX = 20;
  }

  if (PIDY < -1000)
  {
    PIDY = -1000;
  }
  if (PIDY > 1000)
  {
    PIDY = 1000;
  }

  if (PIDZ < -1000)
  {
    PIDZ = -1000;
  }
  if (PIDZ > 1000)
  {
    PIDZ = 1000;
  }


  /* Motor output
    Calculate the motor speeds necessary to achieve the desired setpoints
  */
  pwmX = throttle + PIDX;
  pwmY = throttle + PIDY;
  pwmZ = throttle + PIDZ;

  // constrain values so they are within a safe motor speed
  pwmX = constrain(pwmX, 1060, 1200);
  pwmY = constrain(pwmY, 1060, 1200);
  pwmZ = constrain(pwmZ, 1060, 1200);

  // print output for debugging purposes
  Serial.print("Motor output: ");
  Serial.println(pwmX);

  // write calculated PWM values to the motors
  motorX.writeMicroseconds(pwmX);
  motorY.writeMicroseconds(pwmY);
  motorZ.writeMicroseconds(pwmZ);

  previous_errorX = errorX;
  previous_errorY = errorY;
  previous_errorZ = errorZ;


}
