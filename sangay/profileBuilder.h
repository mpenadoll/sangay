/* Profile Builder
 * builds a trapezoidal or triangular profile
 * when called, integrates along that profile and returns target setpoint
*/

void buildProfile(){
  // Function to calculate the position at times of the trapezoidal OR triangular profile
  // Note that profilePosition[3] is the target point, and should already be set unless !homed
  
  if (!homed){
    // create honming specific profile
    profilePositions[0] = currentPosition;
    profilePositions[3] = -stroke;
    profileTimes[0] = 0; //set t0 to time 0 [ms]
    profileTimes[1] = profileTimes[0] + homeAccelTime; //time at beginning of table top [ms]
    profilePositions[1] = profilePositions[0] - homeAccelDistance; //position at beginning of table top [pulses]
    // End of table top point t2 / x2
    profilePositions[2] = profilePositions[3] + homeAccelDistance; //position at end of table top [pulses]
    profileTimes[2] = profileTimes[1] + abs(profilePositions[2] - profilePositions[1]) / homeSpeed; //time at end of table top [ms]
    // End of profile at target t3 / x3
    profileTimes[3] = profileTimes[2] + homeAccelTime; //time at the end of the profile [ms]
  }

  else {
    // Starting Point t0 / x0
    profileTimes[0] = 0; //set t0 to time 0 [ms]
    profilePositions[0] = currentPosition; //set start position to the current position [pulses]
    int distance = profilePositions[3] - profilePositions[0]; //distance to travel for profile [pulses]
    
    // Trapezoidal Profile Calculations
    if (abs(distance) >= 2*accelDistance){
      // End of Acceleration Point t1 / x1
      profileTimes[1] = profileTimes[0] + accelTime; //time at beginning of table top [ms]
      profilePositions[1] = profilePositions[0] + sgn(distance)*accelDistance; //position at beginning of table top [pulses]
    
      // End of table top point t2 / x2
      profilePositions[2] = profilePositions[3] - sgn(distance)*accelDistance; //position at end of table top [pulses]
      profileTimes[2] = profileTimes[1] + abs(profilePositions[2] - profilePositions[1]) / maxSpeed; //time at end of table top [ms]
    
      // End of profile at target t3 / x3
      profileTimes[3] = profileTimes[2] + accelTime; //time at the end of the profile [ms]
    }
    // Triangular Profile Calculations
    else {
      // End of Acceleration Point t1 / x1
      profilePositions[1] = profilePositions[0] + sgn(distance)*distance/2; //position at top of triangle [pulses]
      profileTimes[1] = profileTimes[0] + sqrt(abs(distance)/accel); //time at top of triangle [ms]
      
      // Position at the top is also t2 / x2
      profilePositions[2] = profilePositions[1]; //position at end of table top [pulses]
      profileTimes[2] = profileTimes[1]; //time at end of table top [ms]
    
      // End of profile at target t3 / x3
      profileTimes[3] = profileTimes[0]+ 2*(profileTimes[1]-profileTimes[0]); //time at the end of the profile [ms]
    }
  }
}

int integrateProfile(){
  // Integrates the profile and returns a new position setpoint for the controller
  unsigned int now = millis(); //what time is it?
  static unsigned int startTime = now;
  if (integrateStart) {
    startTime = now;
//    Serial.println("Integrate Started: ");
//    Serial.println(startTime);
//    Serial.print("DIR: ");
//    Serial.println(dir);
  }
  unsigned int duration = now - startTime; //time since start of profile
//  Serial.println(duration);

  if (duration <= (profileTimes[1] - profileTimes[0])){
//    Serial.println(profilePositions[0] + dir*accel*duration*duration/2);
    return profilePositions[0] + dir*accel*duration*duration/2; // distance [pulses]
  }
  else if (duration <= (profileTimes[2] - profileTimes[0])){
    if (!homed) return profilePositions[1] + dir*homeSpeed*(duration - profileTimes[1]); // distance [pulses]
    return profilePositions[1] + dir*maxSpeed*(duration - profileTimes[1]); // distance [pulses]
  }
  else if (duration <= (profileTimes[3] - profileTimes[0])){
    float topSpeed = accel * (profileTimes[1] - profileTimes[0]); // calculate the top speed in case != maxSpeed
    int deltaT = duration - profileTimes[2]; // calculate the time change from end of table top / triangle
    return profilePositions[2] + dir*(topSpeed*deltaT - accel*deltaT*deltaT/2); // distance [pulses]
  }
  else {
    return profilePositions[3]; //return the current position
  }
}

void printProfile(){
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
