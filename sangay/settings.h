//Settings file for the spice rack and motor

// Motion Profile Variables
float strokeMM = 500.0;     // stroke [mm] 431.8
float pulleyRadius = 24.41;  // radius of the pulley [mm]
float maxSpeedMM = 80.0;     // max speed of rack [mm/s]
float homeSpeedMM = 30.0; // homing speed [mm/s]
float accelMM = 150.0;        // acceleration of rack [mm/s^2]
float PPR = 1440.0;  // number of pulses of encoder per rev

// Set PID Controller Settings for Position Control
float Kp = 7800.0; // proportional gain [V / m]
float Ki = 200.0; // integral gain [V / (m*s)]
float Kd = 0.0; // derivative gain [V * s / m]
float pulseKp, pulseKi, pulseKd; // pulse conversion declarations

// CONSTANTS
const unsigned int sampleTime = 30; //sample time for derivative measurements [ms]
const unsigned int debounceDelay = 50;  // the debounce time; increase if the output flickers
const int error = 5; // error [pulses] allowable for position control
const unsigned int limitTime = 100; // time to move into the limit switch [ms]
const int homeStep = 3; // distance to travel each homing step/loop
const int lightPosition = 0.2 * PPR * 2 * strokeMM / (pulleyRadius * 2 * 3.14); // position to turn on lights [pulses]

// PINS
const int dirPin = 4;  //pin to enable (high) driver
const int PWMpin = 10; //pin to set pwm on driver for up
const int buttonPin = 5;  //pushbutton signal in
const int limitSwitchPin = 6; //limitSwitch signal in
const int encoderApin = 3;  //Best Performance: both pins have interrupt capability
const int encoderBpin = 2;  //Best Performance: both pins have interrupt capability
const int lightPin = 7; //pin for the LED mosfet
const int brakePin = 9; //pin for the motor brake MOSFET

// Calculate variables in units of enconder pulses. Note - gear ratio was removed
float stroke = PPR * 2 * strokeMM / (pulleyRadius * 2 * 3.14);  // stroke [pulses]
float maxSpeed = PPR * 2 * maxSpeedMM / (pulleyRadius * 2 * 3.14 * 1000); // max speed [pulses/ms]
float homeSpeed = PPR * 2 * homeSpeedMM / (pulleyRadius * 2 * 3.14 * 1000); // max speed [pulses/ms]
float accel = PPR * 2 * accelMM / (pulleyRadius * 2 * 3.14 * 1000 * 1000);  // acceleration [pulses/ms^2]
float accelTime = maxSpeed / accel; //time to accelerate and decelerate from stop [ms]
float homeAccelTime = homeSpeed / accel;
float accelDistance = 0.5 * accel * accelTime * accelTime; //the minimum distance to accelerate to max speed [pulses]
float homeAccelDistance = 0.5 * accel * homeAccelTime * homeAccelTime;
