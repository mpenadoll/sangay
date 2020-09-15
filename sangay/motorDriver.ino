/* Motor Driver
Controls an H-Bridge BDC driver, motor brake MOSFET, and degenerative resistor MOSFET.
Motor brake (for holding position)
Degenerative braking (for slowing motor)
inputs:
    1. milliVolts, float, from PID controller
    2. motorDir, int8_t, sign of the current speed
*/

void motorDriver (float milliVolts, int8_t motorDir)
{
  // constrain the voltage command to system's supplyVoltage
  milliVolts = constrain(milliVolts, -supplyVoltage, supplyVoltage);

  //case 1: stopped w/ brake
  //turn off motor and apply brake
  if (motorDir == 0 && milliVolts == 0)
  {
    analogWrite(PWMpin, 0); //set PWM for motor driver
    analogWrite(degenPWMpin, 0); //set degen PWM for resistors
    digitalWrite(brakePin, LOW); //engage brake
  }

  //case 2: stopped w/ power
  //release brake, and move motor negatively
  else if (motorDir == 0 && milliVolts < 0)
  {
    digitalWrite(brakePin, HIGH); //disengage brake
    analogWrite(degenPWMpin, 0); //set degen PWM for resistors
    digitalWrite(dirPin, !posDir); //set direction for motor driver
    analogWrite(PWMpin, map(abs(milliVolts),0,supplyVoltage,15,255)); //set PWM for motor driver
  }

  //case 3: stopped w/ power
  //release brake, and move motor positively
  else if (motorDir == 0 && milliVolts > 0)
  {
    digitalWrite(brakePin, HIGH); //disengage brake
    analogWrite(degenPWMpin, 0); //set degen PWM for resistors
    digitalWrite(dirPin, posDir); //set direction for motor driver
    analogWrite(PWMpin, map(abs(milliVolts),0,supplyVoltage,15,255)); //set PWM for motor driver
  }

  //case 4: lowering w/out power
  //release brake, and let motor coast negatively
  else if (motorDir < 0 && milliVolts == 0)
  {
    digitalWrite(brakePin, HIGH); //disengage brake
    analogWrite(degenPWMpin, 0); //set degen PWM for resistors
    digitalWrite(dirPin, !posDir); //set direction for motor driver
    analogWrite(PWMpin, 0); //set PWM for motor driver
  }

  //case 5: lowering w/ power
  //release brake, and move motor negatively
  else if (motorDir < 0 && milliVolts < 0)
  {
    digitalWrite(brakePin, HIGH); //disengage brake
    analogWrite(degenPWMpin, 0); //set degen PWM for resistors
    digitalWrite(dirPin, !posDir); //set direction for motor driver
    analogWrite(PWMpin, map(abs(milliVolts),0,supplyVoltage,15,255)); //set PWM for motor driver
  }

  //case 6: lowering w/ degen
  //release brake, and degen motor negatively
  else if (motorDir < 0 && milliVolts > 0)
  {
    digitalWrite(brakePin, HIGH); //disengage brake
    digitalWrite(dirPin, !posDir); //set direction for motor driver
    analogWrite(PWMpin, 0); //set PWM for motor driver
    analogWrite(degenPWMpin, map(abs(milliVolts),0,supplyVoltage,15,255)); //set degen PWM for resistors
  }

  //case 7: lifting w/out power
  //release brake, and let motor coast positively
  else if (motorDir > 0 && milliVolts == 0)
  {
    digitalWrite(brakePin, HIGH); //disengage brake
    analogWrite(degenPWMpin, 0); //set degen PWM for resistors
    digitalWrite(dirPin, posDir); //set direction for motor driver
    analogWrite(PWMpin, 0); //set PWM for motor driver
  }

  //case 8: lifting w/ degen
  //release brake, and degen motor positively
  else if (motorDir > 0 && milliVolts < 0)
  {
    digitalWrite(brakePin, HIGH); //disengage brake
    digitalWrite(dirPin, posDir); //set direction for motor driver
    analogWrite(PWMpin, 0); //set PWM for motor driver
    analogWrite(degenPWMpin, map(abs(milliVolts),0,supplyVoltage,15,255)); //set degen PWM for resistors
  }

  //case 9: lifting w/ power
  //release brake, and move motor positively
  else if (motorDir > 0 && milliVolts > 0)
  {
    digitalWrite(brakePin, HIGH); //disengage brake
    analogWrite(degenPWMpin, 0); //set degen PWM for resistors
    digitalWrite(dirPin, posDir); //set direction for motor driver
    analogWrite(PWMpin, map(abs(milliVolts),0,supplyVoltage,15,255)); //set PWM for motor driver
  }

  //error
  //turn off motor and apply brake
  else
  {
    analogWrite(PWMpin, 0); //set PWM for motor driver
    analogWrite(degenPWMpin, 0); //set degen PWM for resistors
    digitalWrite(brakePin, LOW); //engage brake
    Serial.println("ERROR: MOTOR DRIVER, ELSE");
  }
}
