//Settings file for the spice rack and motor
//* indicates motor specific settings (for test setup)

// Motion Profile Variables
float strokeMM = 498.5; //* stroke [mm] 431.8
float pulleyRadius = 24.41; // radius of the pulley [mm]
float maxSpeedMM = 80.0; //* max speed of rack [mm/s]
float accelMM = 100.0; //* acceleration of rack [mm/s^2]
float PPR = 1440.0; // number of pulses of encoder per rev

// convert to [pulse] units. Note - gear ratio not included because encoder is on output shaft
float pulsePerMM = PPR * 2 / (pulleyRadius * 2 * 3.14);  // pulses per mm for use in settings
float stroke = strokeMM * pulsePerMM;  // stroke [pulses]
float maxSpeed = maxSpeedMM * pulsePerMM / 1000.0; // max speed [pulses/ms]
float homeSpeed = 0.4 * maxSpeed;
float accel = accelMM * pulsePerMM / (1000.0 * 1000.0);  // acceleration [pulses/ms^2]

// Set PID Controller Settings for Position Control
float Kp = 7800.0; //* proportional gain [V / m]
float Ki = 0.0; //* integral gain [V / (m*s)]
float Kd = 0.0; //* derivative gain [V * s / m]

// CONSTANTS
const unsigned int sampleTime = 10; // sample time for derivative measurements [ms]
const unsigned int debounceDelay = 50;  // the debounce time [ms] increase if the output flickers
const int maxError = 0.3 * pulsePerMM; // error [pulses] allowable for position control
const int homeOffset = -1.5 * pulsePerMM; // distance between limit and 0, negative number [pulses]
const long lightPosition = 0.2 * stroke; // position to turn on lights [pulses]
const long supplyVoltage = 24000; // system voltage, long due to wrapping of milliVolts [mV]
const int voltageTolerance = 2000; // noise voltage for error detection [mV]
const float minDegenSpeed = 0.1 * pulsePerMM / 1000; // minimum speed that degen is effective [pulses / ms]
const int minPWM = 255 * 1.05 / 24; // minimum PWM value that motor will move [analog]
const float minSpeed = 0.1 * pulsePerMM / 1000; // minimum speed at which the brake can be applied [pulses / ms]
const int stopFudge = 1.6 * pulsePerMM; // fudge factor speed multiplier for stopping [pulses]
const int posDir = HIGH; // direction to set motorDriver positively (note, also swap encA and B pins)

// VOLTMETERS
const float R1 = 47000.00; // resistance of R1 (47K) 
const float R2 = 10000.00; // resistance of R2 (10K) 

// PINS
const int dirPin = 4;  //pin to enable (high) driver
const int PWMpin = 10; //pin to set pwm on driver for up
const int goButtonPin = 5;  // momentary button for GO signal
const int inchUpButtonPin = 8; // momentary button for inching up
const int inchDownButtonPin = 12; // momentary button for inching down
const int limitSwitchPin = 6; //limitSwitch signal in
const int encoderApin = 3;  //Best Performance: both pins have interrupt capability
const int encoderBpin = 2;  //Best Performance: both pins have interrupt capability
const int lightPin = 7; //pin for the LED mosfet
const int brakePin = 11; //pin for the motor brake MOSFET
const int dK1pin = A3; // digital pin for safety relay 1 mosfet
const int dK2pin = A4; // digital pin for safety relay 2 mosfet
const int VoutSpin = A0; // analog input pin for reading Supply voltage
const int VoutK1pin = A1; // analog input pin for reading K1 relay voltage
const int VoutK2pin = A2; // analog input pin for reading K2 relay voltage


// SUPPORTING FUNCTIONS
static inline int8_t sgn(float val)
{
 if (val < 0) return -1;
 if (val==0) return 0;
 return 1;
}
