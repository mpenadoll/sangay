/* Profile Builder
 * builds a trapezoidal or triangular profile
 * when called, integrates along that profile and returns target setpoint
 * inputs:
 * 1. target, end position of the profile [pulses]
 * 2. speed, expected top speed of the profile [pulses/ms]
 * output:
 * 1. topSpeed, top speed of the profile in case expected "speed" not reached [pulses/ms]
*/
bool integrateStart = true; // initializes the start of an integration profile

float buildProfile(long target, float speed)
{
  // tell the integrator it's time to start over
  integrateStart = true;
  
  // Function to calculate the position at times of the trapezoidal OR triangular profile
  // Note that profilePosition[3] is the target point, and should be set to the input

  // starting point t0, x0
  profileTimes[0] = 0; //set t0 to time 0 [ms]
  profilePositions[0] = currentPosition; //set x0 to current position [pulses]

  // set end point to target
  profilePositions[3] = target;

  float topSpeed = speed; // set top speed to max speed, and change it later if necessary
  float accelTime = speed / accel; //time to accelerate and decelerate from stop [ms]
  float accelDistance = 0.5 * accel * accelTime * accelTime; //the minimum distance to accelerate to max speed [pulses]
  long distance = profilePositions[3] - profilePositions[0]; //distance to travel for profile [pulses]

  Serial.print("topSpeed: ");
  Serial.println(topSpeed);
  Serial.print("accelTime: ");
  Serial.println(accelTime);
  Serial.print("accelDistance: ");
  Serial.println(accelDistance);
  Serial.print("distance: ");
  Serial.println(distance);
  
  // Trapezoidal Profile Calculations
  if (abs(distance) >= 2*accelDistance)
  {
    // End of Acceleration Point t1 / x1
    profileTimes[1] = profileTimes[0] + accelTime; //time at beginning of table top [ms]
    profilePositions[1] = profilePositions[0] + sgn(distance)*accelDistance; //position at beginning of table top [pulses]
  
    // End of table top point t2 / x2
    profilePositions[2] = profilePositions[3] - sgn(distance)*accelDistance; //position at end of table top [pulses]
    profileTimes[2] = profileTimes[1] + abs(profilePositions[2] - profilePositions[1]) / speed; //time at end of table top [ms]
  
    // End of profile at target t3 / x3
    profileTimes[3] = profileTimes[2] + accelTime; //time at the end of the profile [ms]
  }
  // Triangular Profile Calculations
  else
  {
    // End of Acceleration Point t1 / x1
    profilePositions[1] = profilePositions[0] + distance/2; //position at top of triangle [pulses]
    profileTimes[1] = profileTimes[0] + sqrt(abs(distance)/accel); //time at top of triangle [ms]
    
    // Position at the top is also t2 / x2
    profilePositions[2] = profilePositions[1]; //position at end of table top [pulses]
    profileTimes[2] = profileTimes[1]; //time at end of table top [ms]
  
    // End of profile at target t3 / x3
    profileTimes[3] = profileTimes[0]+ 2*(profileTimes[1]-profileTimes[0]); //time at the end of the profile [ms]

    topSpeed = accel*(profileTimes[1] - profileTimes[0]);
  }
  return topSpeed;
}

float stopProfile()
{
  // tell the integrator it's time to start over
  integrateStart = true;
  
  // starting point t0, x0
  profileTimes[0] = 0; //set t0 to time 0 [ms]
  profilePositions[0] = currentPosition + currentSpeed*stopFudge; //set x0 to current position [pulses]
  Serial.print("Stop Fudge: ");
  Serial.println(currentSpeed*stopFudge);

  // set point 1 and 2 equal to point 0
  profilePositions[1] = profilePositions[0];
  profileTimes[1] = profileTimes[0];

  profilePositions[2] = profilePositions[1];
  profileTimes[2] = profileTimes[1];

  float topSpeed = abs(currentSpeed);

  profilePositions[3] = profilePositions[2] + sgn(currentSpeed)*topSpeed*topSpeed/(2*accel);
  profileTimes[3] = profileTimes[2] + topSpeed/accel;
  
  return topSpeed;
}

long integrateProfile(float topSpeed)
{
  // Integrates the profile and returns a new position setpoint for the controller
  unsigned int now = millis(); //what time is it?
  static unsigned int startTime = now;
  if (integrateStart)
  {
    startTime = now;
    Serial.println("Integrate Started");
    integrateStart = false;
  }
  unsigned int duration = now - startTime; //time since start of profile
//  Serial.println(duration);
  
  int8_t dir = sgn(profilePositions[3] - profilePositions[0]);

  if (duration <= (profileTimes[1] - profileTimes[0]))
  {
//    Serial.println(profilePositions[0] + dir*accel*duration*duration/2);
    return profilePositions[0] + dir*accel*duration*duration/2; // distance [pulses]
  }
  else if (duration <= (profileTimes[2] - profileTimes[0]))
  {
    return profilePositions[1] + dir*topSpeed*(duration - profileTimes[1]); // distance [pulses]
  }
  else if (duration <= (profileTimes[3] - profileTimes[0]))
  {
    // float topSpeed = accel * (profileTimes[1] - profileTimes[0]); // calculate the top speed in case != speed
    int deltaT = duration - profileTimes[2]; // calculate the time change from end of table top / triangle
    return profilePositions[2] + dir*(topSpeed*deltaT - accel*deltaT*deltaT/2); // distance [pulses]
  }
  else {
    return profilePositions[3]; //return the current position
  }
}

void printProfile()
{
  Serial.print("Profile Positions [0]: ");
  Serial.print(profilePositions[0]);
  Serial.println();
  Serial.print("Profile Positions [1]: ");
  Serial.print(profilePositions[1]);
  Serial.println();
  Serial.print("Profile Positions [2]: ");
  Serial.print(profilePositions[2]);
  Serial.println();
  Serial.print("Profile Positions [3]: ");
  Serial.print(profilePositions[3]);
  Serial.println();
  Serial.print("Profile Times [0]: ");
  Serial.print(profileTimes[0]);
  Serial.println();
  Serial.print("Profile Times [1]: ");
  Serial.print(profileTimes[1]);
  Serial.println();
  Serial.print("Profile Times [2]: ");
  Serial.print(profileTimes[2]);
  Serial.println();
  Serial.print("Profile Times [3]: ");
  Serial.print(profileTimes[3]);
  Serial.println();
}
