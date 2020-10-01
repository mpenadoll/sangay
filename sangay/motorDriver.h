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
enum motorState_enum {BRAKE, PWR_MOVE, DEGEN}; //declare the states as an enum
motorState_enum motorState = BRAKE; // create the state variable of type state_enum
String motorStateNames[3] = {"BRAKE", "PWR_MOVE", "DEGEN"}; // names of states for printing

void motorDriver (int milliVolts, long error)
{
  // constrain the voltage command to system's supplyVoltage
  milliVolts = constrain(milliVolts, -supplyVoltage, supplyVoltage);
  error = abs(error); 
  
  if (debugPrint)
  {
    Serial.print("constrained mV: ");
    Serial.println(milliVolts);
    Serial.print("mapped mV: ");
    Serial.println(map(abs(milliVolts),0,supplyVoltage,minPWM,255));
    Serial.print("motor speed: ");
    Serial.println(currentSpeed,5);
    debugPrint = false;
  }

  switch (motorState)
  {
    // -------------------------------
    case BRAKE:

      analogWrite(PWMpin, 0); //set PWM for motor driver
      analogWrite(degenPWMpin, 255); //set degen PWM for resistors
      digitalWrite(brakePin, LOW); //engage brake

      if (error > maxError)
      {
        motorState = PWR_MOVE;
        Serial.println(motorStateNames[motorState]);
      }
    
      break;
    // -------------------------------
    case PWR_MOVE:

      if (sgn(milliVolts) != sgn(currentSpeed) && currentSpeed > minDegenSpeed) motorState = DEGEN;

      else
      {
        digitalWrite(brakePin, HIGH); //disengage brake
        analogWrite(degenPWMpin, 0); //set degen PWM for resistors
        if (sgn(milliVolts) == 1) digitalWrite(dirPin, LOW); //set direction for motor driver
        else digitalWrite(dirPin, HIGH);
        analogWrite(PWMpin, map(abs(milliVolts),0,supplyVoltage,minPWM,255)); //set PWM for motor driver
      }
      
      if (error <= maxError && currentSpeed <= minSpeed)
      {
        motorState = BRAKE;
        Serial.println(motorStateNames[motorState]);
      }

      break;
    // -------------------------------
    case DEGEN:

      digitalWrite(brakePin, HIGH); //disengage brake
      analogWrite(PWMpin, 0); //set PWM for motor driver
      analogWrite(degenPWMpin, map(abs(milliVolts),0,supplyVoltage,0,255)); //set degen PWM for resistors

      if (sgn(milliVolts) == sgn(currentSpeed) || currentSpeed < minDegenSpeed)
      {
        motorState = PWR_MOVE;
        Serial.println(motorStateNames[motorState]);
      }

      break;
    // -------------------------------
    default:
      motorState = BRAKE;
      break;
    // -------------------------------
  }
}
