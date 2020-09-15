/* Motor Driver
Controls an H-Bridge BDC driver, motor brake MOSFET, and degenerative resistor MOSFET.
Motor brake (for holding position)
Degenerative braking (for slowing motor)
inputs:
    1. milliVolts, float, from PID controller [mV]
    2. motorSpeed, float, [pulses / ms]
*/

// FSM STATES
enum motorState_enum {STOP, PWR_MOVE, DEGEN}; //declare the states as an enum
motorState_enum motorState = STOP; // create the state variable of type state_enum
String motorStateNames[4] = {"STOP", "PWR_MOVE", "DEGEN"}; // names of states for printing

void motorDriver (float milliVolts, float motorSpeed)
{
  // constrain the voltage command to system's supplyVoltage
  milliVolts = constrain(milliVolts, -supplyVoltage, supplyVoltage);

  switch (motorState)
  {
    // -------------------------------
    case STOP:

      analogWrite(PWMpin, 0); //set PWM for motor driver
      analogWrite(degenPWMpin, 255); //set degen PWM for resistors
      digitalWrite(brakePin, LOW); //engage brake

      if (milliVolts != 0)
      {
        motorState = PWR_MOVE;
      }
    
      break;
    // -------------------------------
    case PWR_MOVE:

      if (sgn(milliVolts) != sgn(motorSpeed) && motorSpeed > minDegenSpeed) motorState = DEGEN;

      else
      {
        digitalWrite(brakePin, HIGH); //disengage brake
        analogWrite(degenPWMpin, 0); //set degen PWM for resistors
        digitalWrite(dirPin, sgn(milliVolts)); //set direction for motor driver
        analogWrite(PWMpin, map(abs(milliVolts),0,supplyVoltage,0,255)); //set PWM for motor driver
      }
      
      if (milliVolts == 0 && motorSpeed == 0) motorState = STOP;

      break;
    // -------------------------------
    case DEGEN:

      digitalWrite(brakePin, HIGH); //disengage brake
      analogWrite(PWMpin, 0); //set PWM for motor driver
      analogWrite(degenPWMpin, map(abs(milliVolts),0,supplyVoltage,0,255)); //set degen PWM for resistors

      if (sgn(milliVolts) == sgn(motorSpeed) || motorSpeed < minDegenSpeed) motorState = PWR_MOVE;

      break;
    // -------------------------------
    default:
      motorState = STOP;
      break;
    // -------------------------------
  }
}
