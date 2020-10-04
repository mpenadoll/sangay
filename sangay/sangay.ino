/* Sangay (Spice Rack Controller)
 *  Sangay is an active stratovolcano in central Ecuador. The eruption that started in 1934 is still ongoing.
 *  Elevation: 17,388ft
*/

// VARIABLES
bool go = false; // the state of the drive system (go or stop)
bool homed = false;  // has the stepper been properly homed
int limitSwitch = LOW;  // the state of the limit switch
int8_t dir = 1; //the current state direction of the drive system (up is HIGH)
int buttonState = HIGH; // the current reading from the input pin
long currentPosition; //the current position [pulses]

float currentSpeed; //the current speed (average) [pulses / ms]
const int numReadings = 4; //number of readings for speed moving average
int readIndex; //index to update the readings
float speedReadings[numReadings]; // array for speed moving average
float speedTotal; // sum of speed readings

long profilePositions[4]; //{x0, x1, x2, x3} x0 is the start position, and x3 is the end position [pulses]
unsigned int profileTimes[4]; //{t0, t1, t2, t3} t0 is the start time, and t3 is the end time [ms]
bool debugPrint = false;

// FSM STATES
enum state_enum {STOPPED, HOMING, MOVING}; //declare the states as an enum
state_enum state = STOPPED; // create the state variable of type state_enum
String stateNames[3] = {"STOPPED", "HOMING", "MOVING"}; // names of states for printing

// Import Libraries
#include <Encoder.h>
#include "settings.h"
#include "PIDcontroller.h"
#include "motorDriver.h"
#include "profileBuilder.h"

// Initialize Encoder
Encoder encoder(encoderApin, encoderBpin);

void setup()
{
  Serial.begin(9600);
  
  // put your setup code here, to run once:
  setGains();
  encoder.write(0);
  
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(limitSwitchPin, INPUT);
  pinMode(PWMpin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(lightPin, OUTPUT);
  pinMode(brakePin, OUTPUT);
  pinMode(degenPWMpin, OUTPUT);

  //---------------------------------------------- Set PWM frequency for D9 & D10 ------------------------------
   
  //TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
  TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000100;    // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz
  
  updateSensors();
  Serial.print("Stroke [mm]: ");
  Serial.println(strokeMM,4);
  Serial.print("Stroke [pulses]: ");
  Serial.println(stroke,4);
  Serial.print("Max Speed [mm/s]: ");
  Serial.println(maxSpeedMM,4);
  Serial.print("Max Speed [pulses/ms]: ");
  Serial.println(maxSpeed,4);
  Serial.print("Acceleration [mm/s^2]: ");
  Serial.println(accelMM,4);
  Serial.print("Acceleration [pulses/ms^2]: ");
  Serial.println(accel,4);
  Serial.println("-------------------");
  Serial.print("Kp: ");
  Serial.println(Kp,4);
  Serial.print("pulseKp: ");
  Serial.println(pulseKp,4);
  Serial.print("Ki: ");
  Serial.println(Ki,4);
  Serial.print("pulseKi: ");
  Serial.println(pulseKi,6);
  Serial.print("Kd: ");
  Serial.println(Kd,4);
  Serial.print("pulseKd: ");
  Serial.println(pulseKd,4);
  Serial.println("-------------------");
  Serial.println("SENSORS");
  Serial.print("Current Position: ");
  Serial.println(currentPosition);
  Serial.print("Current Speed: ");
  Serial.println(currentSpeed);
  Serial.print("Limit Switch: ");
  Serial.println(limitSwitch);
  Serial.print("Go: ");
  Serial.println(go);
  Serial.println("-------------------");
  Serial.println("READY");
}

void updateSensors()
{
  // update button and limit switches
  int reading = digitalRead(buttonPin);  // read the state of the switch into a local variable
  limitSwitch = digitalRead(limitSwitchPin);

  // get the time now
  unsigned int now = millis();
  static unsigned int lastTime = now - sampleTime;

  // read encoder and calculate the speed
  long newPosition = encoder.read();
  static long lastPosition = newPosition;
  // only calculate the speed if time elapsed has been more than set sample time
  if (now - lastTime >= sampleTime)
  {
    speedTotal -= speedReadings[readIndex];
    speedReadings[readIndex] = (newPosition - lastPosition)/(float)(now - lastTime);
    speedTotal += speedReadings[readIndex];

    readIndex += 1;
    if (readIndex >= numReadings) readIndex = 0;

    currentSpeed = speedTotal / numReadings;

    // save static variables for next round
    lastTime = now;
    lastPosition = newPosition;
  }
  currentPosition = newPosition;

  static int lastButtonState = HIGH; // the previous reading from the input pin
  static unsigned int lastDebounceTime = 0;  // the last time the output pin was toggled
  // reset the debouncing timer if reading has changed
  if (reading != lastButtonState) lastDebounceTime = now;

  if ((now - lastDebounceTime) > debounceDelay && reading != buttonState)
  {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, and the button state has changed
    buttonState = reading;
    if (buttonState == LOW)
    {
      go = !go;       // change system state to go
      Serial.print("GO: ");
      Serial.println(go);
    }
  }
  lastButtonState = reading; // save the reading. Next time through the loop, it'll be the lastButtonState
}

void moveTo(long setpoint, long target)
{
  static long milliVolts = 0;
  unsigned int now = millis();
  static unsigned int lastTime = now - sampleTime;
  if (now - lastTime >= sampleTime)
  {
    milliVolts = computePID(setpoint, currentPosition);
    motorDriver(milliVolts, target - currentPosition);
    lastTime = now;

//    static unsigned int lastPrintTime = now;
//    if (now - lastPrintTime >= 500)
//    {
//      Serial.print("mV: ");
//      Serial.println(milliVolts);
//      Serial.print("setpoint: ");
//      Serial.println(setpoint);
//      Serial.print("current pos: ");
//      Serial.println(currentPosition);
//      lastPrintTime = now;
//    }
  }
}

long stop()
{
  float topSpeed = stopProfile();
  long target = profilePositions[3];
  while (motorState != BRAKE)
  {
    long setpoint = integrateProfile(topSpeed);
    moveTo(setpoint, target);
    updateSensors();
  }
  return target;
}


void loop()
{
  updateSensors();

  static long target = 0; // finishing position of a profile
  static long setpoint = 0; // next step along a profile
  static bool homeFlag = false; // flag for setting 0 when hitting limit switch
  static float topSpeed = 0; // speed to transfer from profile builder to integrator

  //if the position is greater than the lighting up position, turn on the LED strip
  //otherwise, turn it off
  if (currentPosition > lightPosition)
  {
    digitalWrite(lightPin, HIGH);
  }
  else
  {
    digitalWrite(lightPin, LOW);
  }

  unsigned int now = millis();
  static unsigned int lastPrintTime = now;
  if (now - lastPrintTime >= 700 && state == HOMING)
  {
    Serial.print("setpoint: ");
    Serial.println(setpoint);
    Serial.print("current pos: ");
    Serial.println(currentPosition);
    Serial.print("current speed: ");
    Serial.println(currentSpeed);
//    debugPrint = true;
    lastPrintTime = now;
  }

  // {STOPPED, HOMING, MOVING_UP, MOVING_DOWN}
  switch (state)
  {
    // -------------------------------
    case STOPPED:

      // if motorstate not stop
      // set target as soon as possible
      // build a half triangle profile
      // moveto target
      if (motorState != BRAKE)
      {
        target = stop();
        Serial.println("stop() done");
        setpoint = target;
        go = false;
      }
      else moveTo(setpoint, target);

      if (go)
      { 
        if (!homed)
        {
          state = HOMING;
          target = -stroke;
          topSpeed = buildProfile(target, homeSpeed);
        }
        else if (dir == 1)
        {
          state = MOVING;
          target = stroke;
          topSpeed = buildProfile(target, maxSpeed);
        }
        else if (dir == -1)
        {
          state = MOVING;
          target = 0;
          topSpeed = buildProfile(target, maxSpeed);
        }
        Serial.println(stateNames[state]);
        Serial.print("Top Speed: ");
        Serial.println(topSpeed);
        printProfile();
      }

      break;
    // -------------------------------
    case HOMING:

      // if limit switch, move up until not limit for set distance
      if (limitSwitch && !homeFlag)
      {
        stop();
        target = stroke;
        topSpeed = buildProfile(target, limitSpeed);
        homeFlag = true;
      }
      else if (!limitSwitch && homeFlag)
      {
        long homeMarker = currentPosition;
        stop();
        encoder.write(homeOffset + (currentPosition - homeMarker));
        for (int i = 0; i <= numReadings; i++)
        {
          Serial.println("updated sensors i times");
          updateSensors();
          delay(sampleTime);
        }
        homed = true;
        homeFlag = false;
        Serial.println("Homed = true");
        target = 0;
        topSpeed = buildProfile(target, limitSpeed);
        Serial.print("topSpeed: ");
        Serial.println(topSpeed);
        printProfile();
      }
  
      setpoint = integrateProfile(topSpeed);
      moveTo(setpoint, target);

      if (motorState == BRAKE && abs(currentPosition - target) <= maxError && homed) go = false;

      if (!go)
      {
        homeFlag = false;
        dir = 1;
        state = STOPPED;
        Serial.println(stateNames[state]);
        target = currentPosition;
        setpoint = target;
      }

      break;
    // -------------------------------
    case MOVING:

      if (limitSwitch)
      {
        homed = false;
        Serial.println("Homed = false");
        go = false;
      }
      else
      {
        setpoint = integrateProfile(topSpeed);
        moveTo(setpoint, target);
        if (motorState == BRAKE && abs(currentPosition - target) <= maxError) go = false;
      }

      if (!go)
      {
        dir = -dir;
        state = STOPPED;
        Serial.println(stateNames[state]);
        target = currentPosition;
        setpoint = target;
      }

      break;
    // -------------------------------
  }
}
