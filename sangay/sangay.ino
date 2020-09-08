// Spice Rack Controller
#include <Encoder.h>
#include "settings.h"

// VARIABLES
bool go = false; // the state of the drive system (go or stop)
bool homed = false;  // has the stepper been properly homed
int limitSwitch = LOW;  // the state of the limit switch
int8_t dir = 1; //the current state direction of the drive system (up is HIGH)
int8_t lastDir = 1; //the last direction of the system, to check if it switched
int buttonState = HIGH; // the current reading from the input pin
int currentPosition; //the current position [pulses]
float currentSpeed; //the current speed (average) [pulses / ms]
int profilePositions[4]; //{x0, x1, x2, x3} x0 is the start position, and x3 is the end position [pulses]
unsigned int profileTimes[4]; //{t0, t1, t2, t3} t0 is the start time, and t3 is the end time [ms]
bool integrateStart = true; // initializes the start of an integration profile
int posDir = LOW; //positive direction of motor driver dir pin

// Initialize Encoder
Encoder encoder(encoderApin, encoderBpin);

void setup() {
  Serial.begin(9600);
  
  // put your setup code here, to run once:
  setGains();
  encoder.write(0);
  profilePositions[3] = -stroke;
  
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
  Serial.print("Acceleration Time [ms]: ");
  Serial.println(accelTime,4);
  Serial.print("Acceleration Distance [pulses]: ");
  Serial.println(accelDistance,4);
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

static inline int8_t sgn(float val) {
 if (val < 0) return -1;
 if (val==0) return 0;
 return 1;
}

void updateSensors(){
  // update button and limit switches
  int reading = digitalRead(buttonPin);  // read the state of the switch into a local variable
  limitSwitch = digitalRead(limitSwitchPin);

  // get the time now
  unsigned int now = millis();
  static unsigned int lastTime = now - sampleTime;

  // read encoder and calculate the speed
  int newPosition = encoder.read();
  static int lastPosition = newPosition;
  // only calculate the speed if time elapsed has been more than set sample time
  if (now - lastTime >= sampleTime){
    currentSpeed = (newPosition - lastPosition)/(float)(now - lastTime);
    // save static variables for next round
    lastTime = now;
    lastPosition = newPosition;
  }
  currentPosition = newPosition;

  static int lastButtonState = HIGH; // the previous reading from the input pin
  static unsigned int lastDebounceTime = 0;  // the last time the output pin was toggled
  // reset the debouncing timer if reading has changed
  if (reading != lastButtonState) lastDebounceTime = now;

  if ((now - lastDebounceTime) > debounceDelay && reading != buttonState) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, and the button state has changed
    buttonState = reading;
    if (buttonState == LOW) {
      go = !go;       // change system state to go
      Serial.print("GO: ");
      Serial.println(go);
      if (!go && homed) dir = -dir; // reverse directions if motion stopped with button, and homed
    }
  }
  lastButtonState = reading; // save the reading. Next time through the loop, it'll be the lastButtonState
}

void setPosition(int newPosition){
  motorDriver(0, sgn(currentSpeed));
  encoder.write(newPosition);
  while (currentSpeed != 0){
    updateSensors();
    delay(1);
  }
  encoder.write(newPosition);
  updateSensors();
}

void stopNow(){
  Serial.println("STOPPING");
  int8_t stopDir = sgn(currentSpeed);
  unsigned int startTime = millis();
  int startPosition = currentPosition;
  if (stopDir == 1) startPosition += 1.4*currentSpeed*sampleTime; // add fudge factor to make stopping smooth
  else startPosition += 1.0*currentSpeed*sampleTime;
  float startSpeed = abs(currentSpeed);
  int distance = accel * (startSpeed/accel) * (startSpeed/accel) / 2;
  int endPosition = startPosition + stopDir*distance;
  int posSetpoint = startPosition; // initialize setpoint [pulses]
  float milliVolts;
  unsigned int now = startTime;
  unsigned int lastTime = now;
  integrateStart = true;
  while (abs(posSetpoint - endPosition) > error){
    if (now - lastTime >= sampleTime){
      int deltaT = now - startTime; // calculate the time change from begginning of stop
      posSetpoint = startPosition + stopDir*(startSpeed*deltaT - accel*deltaT*deltaT/2); // [pulses]
      milliVolts = computePID(posSetpoint, currentPosition);
      motorDriver(milliVolts, sgn(currentSpeed));
      // save static variables for next round
      lastTime = now;
    }
    updateSensors();
    now = millis();
  }
  while (currentSpeed != 0) {
    if (now - lastTime >= sampleTime){
      milliVolts = computePID(endPosition, currentPosition);
      motorDriver(milliVolts, sgn(currentSpeed));
      lastTime = now;
    }
    updateSensors();
    now = millis();
  }
  motorDriver(0, sgn(currentSpeed));
  Serial.println("DONE STOPPING");
}

void homeNow(){
  Serial.println("HOMING");
  if (currentSpeed != 0) stopNow();
  setPosition(0);
  buildProfile();
  printProfile();
  int posSetpoint;
  float milliVolts;
  dir = -1;
  integrateStart = true;
  unsigned int now = millis();
  unsigned int lastTime = now - sampleTime;
  while (!limitSwitch && go){
    if (now - lastTime >= sampleTime){
      posSetpoint = integrateProfile(); // [pulses]
      milliVolts = computePID(posSetpoint, currentPosition);
      motorDriver(milliVolts, sgn(currentSpeed));
      // save static variables for next round
      lastTime = now;
    }
    updateSensors();
    now = millis();
  }
  if (currentSpeed != 0) stopNow();
  now = millis();
  lastTime = now - sampleTime;  
  posSetpoint = currentPosition;
  integrateStart = true;
  while (limitSwitch && go){
    if (now - lastTime >= sampleTime){
      posSetpoint += homeStep;
      milliVolts = computePID(posSetpoint, currentPosition);
      motorDriver(milliVolts, sgn(currentSpeed));
      lastTime = now;
    }
    updateSensors();
    now = millis();
  }
  unsigned int startTime = now;
  while ((now - startTime) < limitTime && go){
    if (now - lastTime >= sampleTime){
      posSetpoint += homeStep;
      milliVolts = computePID(posSetpoint, currentPosition);
      motorDriver(milliVolts, sgn(currentSpeed));
      lastTime = now;
    }
    updateSensors();
    now = millis();
  }
  motorDriver(0, sgn(currentSpeed));
  lastTime = now - sampleTime;
  integrateStart = true;
  while (!limitSwitch && go){
    if (now - lastTime >= sampleTime){
      posSetpoint -= homeStep;
      milliVolts = computePID(posSetpoint, currentPosition);
      motorDriver(milliVolts, sgn(currentSpeed));
      lastTime = now;
    }
    updateSensors();
    now = millis();
  }
  startTime = now;
  while ((now - startTime) < limitTime && go){
    if (now - lastTime >= sampleTime){
      posSetpoint -= homeStep;
      milliVolts = computePID(posSetpoint, currentPosition);
      motorDriver(milliVolts, sgn(currentSpeed));
      lastTime = now;
    }
    updateSensors();
    now = millis();
  }
  if (currentSpeed != 0) stopNow();
  if (limitSwitch && go) {
    setPosition(0);
    homed = true;
  }
  go = false;
  dir = 1;
  lastDir = -1;
  Serial.println("DONE HOMING");
}

void loop() {

  //check if system thinks it is at 0 but is not at home
  if (abs(currentPosition) < error && !limitSwitch && homed) {
    homed = false;
    Serial.println("HOME FAILED (OUTSIDE LIMIT)");
  }

  //check if system is at home and doesn't know it, and travelling downwards
  if (limitSwitch && abs(currentPosition) > error && dir == -1 && homed) {
    homed = false;
    Serial.println("HOME FAILED (INSIDE LIMIT)");
  }

  //check if target has been reached
  if (abs(currentPosition - profilePositions[3]) < error && currentSpeed == 0){
    dir = -dir;
    go = false;
    Serial.println("TARGET REACHED");
  }

  //if the direction has changed, update the target and build the profile
  if (dir != lastDir){
    if (dir == 1) profilePositions[3] = stroke;
    else profilePositions[3] = 0;
    buildProfile();
    lastDir = dir;
    integrateStart = true;
    Serial.println("PROFILE BUILT");
    printProfile();
  }

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
  
  updateSensors();
  
  if (go && homed) {
    //Serial.println("RUNNING");
    unsigned int now = millis();
    static unsigned int lastTime = now - sampleTime;
    if (now - lastTime >= sampleTime){
      int posSetpoint = integrateProfile(); // [pulses]
      float milliVolts = computePID(posSetpoint, currentPosition);
      motorDriver(milliVolts, sgn(currentSpeed));
      // save static variables for next round
      lastTime = now;
    }
  }
  else if (go && !homed) homeNow();
  else {
    if (currentSpeed != 0) stopNow();
    motorDriver(0, sgn(currentSpeed));
  }
}
