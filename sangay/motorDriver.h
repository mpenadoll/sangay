/* Motor Driver
Controls an H-Bridge BDC driver, motor brake MOSFET, and degenerative resistor MOSFET.
Motor brake (for holding position)
Degenerative braking (for slowing motor)
inputs:
    1. milliVolts, float, from PID controller [mV]
    2. error, long [pulses]
    3. currentSpeed, float, global [pulses / ms]
*/

// FSM STATES
enum motorState_enum {PWR_MOVE, DEGEN}; //declare the states as an enum
motorState_enum motorState = DEGEN; // create the state variable of type state_enum
String motorStateNames[2] = {"PWR_MOVE", "DEGEN"}; // names of states for printing

void motorDriver (long milliVolts)
{
  // constrain the voltage command to system's supplyVoltage
  milliVolts = constrain(milliVolts, -supplyVoltage, supplyVoltage);

  switch (motorState)
  {
    // -------------------------------
    case PWR_MOVE:

      if (sgn(milliVolts) == 1) digitalWrite(dirPin, posDir); //set direction for motor driver
      else digitalWrite(dirPin, !posDir);
      analogWrite(PWMpin, map(abs(milliVolts),0,supplyVoltage,minPWM,255)); //set PWM for motor driver

//      Serial.println(map(abs(milliVolts),0,supplyVoltage,minPWM,255));

      if ((sgn(milliVolts) != sgn(currentSpeed) && currentSpeed > minDegenSpeed) || milliVolts == 0 || Vin > 26)
      {
        motorState = DEGEN;
        Serial.println(motorStateNames[motorState]);
      }

      break;
    // -------------------------------
    case DEGEN:

      analogWrite(PWMpin, 0); //set PWM for motor drivers

      if ((sgn(milliVolts) == sgn(currentSpeed) || currentSpeed < minDegenSpeed) && milliVolts != 0 && Vin <= 26)
      {
        motorState = PWR_MOVE;
        Serial.println(motorStateNames[motorState]);
      }

      break;
    // -------------------------------
    default:
      motorState = DEGEN;
      break;
    // -------------------------------
  }
}
