/* Button
toggleButton creates a button that toggles the output.

By using the output as an input (lastToggle) you can change the toggle
outside of the button functions (on a trigger, for example).
*/

class toggleButton
{
  // class member variables
  int lastButtonState;
  unsigned int lastDebounceTime;
  int buttonState;
  unsigned int debounceDelay;
  int buttonPin;
  bool toggle;
  int reading;

  // constructor
  public:
  toggleButton(int pin, unsigned int delayTime)
  {
    buttonPin = pin;
    pinMode(buttonPin, INPUT_PULLUP);
    debounceDelay = delayTime;

    buttonState = LOW;
    lastDebounceTime = 0;
    lastButtonState = buttonState;

    toggle = false;
  }

  bool updateButton(bool currentToggle)
  {
    toggle = currentToggle;

    reading = digitalRead(buttonPin);  // read the state of the switch into a local variable

    // get the time now
    unsigned int now = millis();
    
    // reset the debouncing timer if reading has changed
    if (reading != lastButtonState) lastDebounceTime = now;

    if ((now - lastDebounceTime) > debounceDelay && reading != buttonState)
    {
      // whatever the reading is at, it's been there for longer than the debounce
      // delay, and the button state has changed
      buttonState = reading;
      if (buttonState == LOW)
      {
        toggle = !toggle; // toggle
//        Serial.println(toggle);
      }
    }

    lastButtonState = reading; // save the reading. Next time through the loop, it'll be the lastButtonState

    return toggle;
  }
};

class momentaryButton
{
  // class member variables
  int lastButtonState;
  unsigned int lastDebounceTime;
  int buttonState;
  unsigned int debounceDelay;
  int buttonPin;
  int reading;

  // constructor
  public:
  momentaryButton(int pin, unsigned int delayTime)
  {
    buttonPin = pin;
    pinMode(buttonPin, INPUT_PULLUP);
    debounceDelay = delayTime;

    buttonState = LOW;
    lastDebounceTime = 0;
    lastButtonState = buttonState;

  }

  bool updateButton()
  {

    reading = digitalRead(buttonPin);  // read the state of the switch into a local variable

    // get the time now
    unsigned int now = millis();
    
    // reset the debouncing timer if reading has changed
    if (reading != lastButtonState) lastDebounceTime = now;

    if ((now - lastDebounceTime) > debounceDelay && reading != buttonState)
    {
      // whatever the reading is at, it's been there for longer than the debounce
      // delay, and the button state has changed
      buttonState = reading;
    }

    lastButtonState = reading; // save the reading. Next time through the loop, it'll be the lastButtonState

    return !buttonState; // not button state because true = HIGH, false = LOW
  }
};
