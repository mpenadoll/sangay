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

// FSM STATES
enum state_enum {STOPPED, HOMING, MOVING, ERROR}; //declare the states as an enum
state_enum state = STOPPED; // create the state variable of type state_enum
String stateNames[4] = {"STOPPED", "HOMING", "MOVING", "ERROR"}; // names of states for printing
int8_t errorCode = 0; // code for the error type

// Import Libraries
#include <Encoder.h>
#include "settings.h"
#include "PIDcontroller.h"
#include "motorDriver.h"
#include "profileBuilder.h"
#include "button.h"
#include "voltMeter.h"

// Initialize Encoder
Encoder encoder(encoderApin, encoderBpin);

// Initialize Buttons
toggleButton goButton(goButtonPin, debounceDelay);
momentaryButton inchUpButton(inchUpButtonPin, debounceDelay);
momentaryButton inchDownButton(inchDownButtonPin, debounceDelay);
voltMeter Vs(VoutSpin, R1, R2);
voltMeter Vk1(VoutK1pin, R1, R2);
voltMeter Vk2(VoutK2pin, R1, R2);

void setup()
{
  Serial.begin(9600);
  
  // put your setup code here, to run once:
  setGains();
  encoder.write(0);
  
  pinMode(limitSwitchPin, INPUT);
  pinMode(PWMpin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(lightPin, OUTPUT);
  pinMode(brakePin, OUTPUT);
  pinMode(dK1pin, OUTPUT);
  pinMode(dK2pin, OUTPUT);

  digitalWrite(lightPin, LOW);
  digitalWrite(brakePin, LOW);
  delay(100);

  //---------------------------------------------- Set PWM frequency for D9 & D10 ------------------------------
   
  //TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
  TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000100;    // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz
  
  go = goButton.updateButton(go);
  inchUpButton.updateButton();
  inchDownButton.updateButton();

  for (int i = 0; i <= numReadings; i++)
  {
    updateEncoder();
    delay(sampleTime);
  }

  safetyCheck();

  Serial.print("PulsePerMM: ");
  Serial.println(pulsePerMM,4);
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
  Serial.print("Max Error [pulses]: ");
  Serial.println(maxError);
  Serial.print("Home Offset [pulses]: ");
  Serial.println(homeOffset);
  Serial.print("Min Degen Speed [pulses / ms]: ");
  Serial.println(minDegenSpeed,5);
  Serial.print("Min Speed [pulses / ms]: ");
  Serial.println(minSpeed,5);
  Serial.print("Stop Fudge [pulses]: ");
  Serial.println(stopFudge);
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
  Serial.println(stateNames[state]);
}

void updateEncoder()
{
  // updates the encoder and the limit switch for motion.
  // make sure to only call this function once per sample period

  limitSwitch = digitalRead(limitSwitchPin);

  // read encoder and calculate the speed
  long newPosition = encoder.read();
  static long lastPosition = newPosition;

  speedTotal -= speedReadings[readIndex];
  speedReadings[readIndex] = (newPosition - lastPosition)/(float)sampleTime;
  speedTotal += speedReadings[readIndex];

  readIndex += 1;
  if (readIndex >= numReadings) readIndex = 0;

  currentSpeed = speedTotal / numReadings;

  // reset static variable and update global variable
  lastPosition = newPosition;
  currentPosition = newPosition;
}

void moveTo(long setpoint)
{
  static long milliVolts = 0;
  unsigned int now = millis();
  static unsigned int lastTime = now - sampleTime;

  digitalWrite(brakePin, HIGH); // disengage the brake
  
  if (now - lastTime >= sampleTime)
  {
    updateEncoder();
    milliVolts = computePID(setpoint, currentPosition);
    motorDriver(milliVolts);
    lastTime = now;

    // float Vsupply = Vs.readVoltage();
    // if (Vsupply > 25 || Vsupply < 22) Serial.println(Vsupply);
  }
}

long quickStop()
{
  long setpoint = currentPosition;
  while (abs(currentSpeed) > minSpeed) moveTo(setpoint);
  return setpoint;
}

long stop()
{
  float topSpeed = stopProfile();
  long target = profilePositions[3];
  while (abs(currentPosition - target) > maxError || abs(currentSpeed) > minSpeed)
  {
    long setpoint = integrateProfile(topSpeed);
    moveTo(setpoint);
  }
  return target;
}

void inching(int step)
{
  long setpoint = currentPosition + step;
  while (abs(currentPosition - setpoint) > maxError) moveTo(setpoint);
}

void safetyCheck()
{
  Serial.println("SAFETY CHECK");
  // SAFETY RELAY CHECK START
  digitalWrite(lightPin, HIGH); // turn on LEDs to burn off voltage
  
  digitalWrite(dK1pin, HIGH);
  digitalWrite(dK2pin, HIGH); // turn on both safety relays
  delay(500);
  Serial.print("Vs: ");
  Serial.println(Vs.readVoltage());
  Serial.print("Vk1: ");
  Serial.println(Vk1.readVoltage());
  Serial.print("Vk2: ");
  Serial.println(Vk2.readVoltage());
  if (abs(Vs.readVoltage() - supplyVoltage) > voltageTolerance || abs(Vk1.readVoltage() - supplyVoltage) > voltageTolerance || abs(Vk2.readVoltage() - supplyVoltage) > voltageTolerance)
  {
    Serial.println("ERROR");
    state = ERROR;
    if (Vs.readVoltage() < voltageTolerance)
    {
      errorCode = 1;
    }
    else if (Vk1.readVoltage() < voltageTolerance)
    {
      errorCode = 3;
    }
    else if (Vk2.readVoltage() < voltageTolerance)
    {
      errorCode = 2;
    }
  }
  digitalWrite(dK1pin, HIGH); // turn on relay 1
  digitalWrite(dK2pin, LOW); // turn off relay 2
  delay(2000);
  if (state != ERROR && Vk2.readVoltage() > voltageTolerance)
  {
    Serial.println("ERROR");
    state = ERROR;
    errorCode = 4;
  }
  digitalWrite(dK1pin, LOW); // turn off relay 1
  digitalWrite(dK2pin, LOW); // turn off relay 2
  delay(500);
  if (state != ERROR && Vk1.readVoltage() > voltageTolerance)
  {
    Serial.println("ERROR");
    state = ERROR;
    errorCode = 5;
  }
  Serial.print("Error Code: ");
  Serial.println(errorCode);

  if (errorCode <= 3)
  {
    digitalWrite(dK1pin, HIGH);
    digitalWrite(dK2pin, HIGH); // turn on both safety relays
  }
  else if (errorCode == 4)
  {
    digitalWrite(dK1pin, HIGH); // turn on relay 1
    digitalWrite(dK2pin, LOW); // turn off relay 2
  }
  else if (errorCode == 5)
  {
    digitalWrite(dK1pin, LOW); // turn off relay 1
    digitalWrite(dK2pin, LOW); // turn off relay 2
  }
  delay(500);

  digitalWrite(lightPin, LOW); // turn off LEDs after test
  //SAFETY RELAY TEST END
}

void loop()
{
  go = goButton.updateButton(go);
  
  static long target = 0; // finishing position of a profile
  static long setpoint = 0; // next step along a profile
  static float topSpeed = 0; // speed to transfer from profile builder to integrator


  unsigned int now = millis(); //timer for ALT and debug printing
  // ALT Testing >>
  // static unsigned int lastTime = now - 10000;
  // if (now - lastTime > 10000)
  // {
  //   go = true;
  //   lastTime = now;
  // }
  // ALT Testing <<
  // Debug Printing >>
  static unsigned int lastPrintTime = now;
  if (now - lastPrintTime >= 1500 && abs(currentSpeed) > 0)
  {
//    Serial.print("setpoint: ");
//    Serial.println(setpoint);
//    Serial.print("current pos: ");
//    Serial.println(currentPosition);
//    Serial.print("current speed: ");
    // Serial.println(currentSpeed);
    lastPrintTime = now;
  }
  // Debug Printing <<

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

  // FSM
  switch (state)
  {
    // -------------------------------
    case STOPPED:

      // ensure system is not moving when STOPPED
      if (abs(currentSpeed) > minSpeed)
      {
        Serial.println("stopping");
        stop();
        Serial.print("stop() done, position: ");
        Serial.println(currentPosition);
      }
      else
      {
        digitalWrite(brakePin, LOW); //engage brake;
        motorDriver(0);
      }

      if (inchUpButton.updateButton() || inchDownButton.updateButton())
      {
        Serial.println("INCHING...");
        while (inchUpButton.updateButton()) inching(2*maxError);
        while (inchDownButton.updateButton()) inching(-2*maxError);
        quickStop();
      }

      // when "go" set target, state, and build profile
      if (go)
      { 
        if (!homed)
        {
          state = HOMING;
          target = -stroke*1.1;
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

      setpoint = integrateProfile(topSpeed);
      moveTo(setpoint);

      // when limit switch activated, stop, build profile, and trip flag
      // then move up and set home once limit switch deactivated
      if (limitSwitch)
      {
        stop();
        while (limitSwitch) inching(2*maxError);
        // quickStop();
        encoder.write(homeOffset);
        for (int i = 0; i <= numReadings; i++)
        {
          updateEncoder();
          delay(sampleTime);
        }
        while (abs(currentPosition) > maxError) inching(2*maxError);
        quickStop();

        homed = true;
        Serial.println("Homed = true");

        go = false;
      }

      if (!go)
      {
        dir = 1;
        state = STOPPED;
        Serial.println(stateNames[state]);
      }

      break;
    // -------------------------------
    case MOVING:

      // error if the limit switch is activated at any time
      if (limitSwitch)
      {
        homed = false;
        Serial.println("Homed = false");
        go = false;
      }
      else
      {
        setpoint = integrateProfile(topSpeed);
        moveTo(setpoint);
        if (abs(currentPosition - target) <= maxError && abs(currentSpeed) <= minSpeed) go = false;
      }

      if (!go)
      {
        dir = -dir;
        state = STOPPED;
        Serial.println(stateNames[state]);
      }

      break;
    // -------------------------------
    case ERROR:
      Serial.print("Vs: ");
      Serial.println(Vs.readVoltage());
      Serial.print("Vk1: ");
      Serial.println(Vk1.readVoltage());
      Serial.print("Vk2: ");
      Serial.println(Vk2.readVoltage());
      Serial.print("Error Code: ");
      Serial.print(errorCode);
      Serial.print(" | ");
      if (errorCode == 0) Serial.println("ERROR: undefined");
      else if (errorCode == 1) Serial.println("ERROR: Supply Off");
      else if (errorCode == 2) Serial.println("ERROR: K2 Relay Open");
      else if (errorCode == 3) Serial.println("ERROR: K1 Relay Open");
      else if (errorCode == 4) Serial.println("ERROR: K2 Relay Closed");
      else if (errorCode == 5) Serial.println("ERROR: K1 Relay Closed");
      Serial.println("-------------------");
      delay(2000);
      
      break;
  }
}
