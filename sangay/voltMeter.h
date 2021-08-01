/* Volt Meter
voltMeter creates a voltmeter to measure voltages

input - none
output - mV into the voltage divider
*/

class voltMeter
{
  // class member variables
  float Vout; // voltage out of the voltage divider
  float Vin; // voltage in of the voltage divider (of interest)
  float R1; // resistance of R1
  float R2; // resistance of R2
  int voltPin;

  //constructor
  public:
  voltMeter(int pin, float resistor1, float resistor2)
  {
    voltPin = pin; 
    pinMode(voltPin, INPUT); //assigning the input port for voltmeter
    R1 = resistor1;
    R2 = resistor2;
  }

  float readVoltage()
  {
    Vout = (analogRead(voltPin) * 500.0) / 1024.00; // formula for calculating voltage out from analog reading [mV]
    Vin = Vout * ( (R1+R2) / R2 ); // formula for calculating voltage in [mV]
    if (Vin < 50) //condition
    {
      Vin = 0;//statement to quash undesired reading !
    }
    return Vin; // voltage of interest [mV]
  }

};