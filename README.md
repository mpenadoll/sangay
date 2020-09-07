# sangay
repository for the Spice Rack (Quiet Systems LLC)

The _sangay_ folder contains the main Arduino files that make up the system controller.

_sangay.ino_ has the loop() function with the general logic.

_settings.h_ has all the settings that can be changed for the appropriate system.

_PIDcontroller.ino_ is the PID controller loop that outputs milliVolts for the motor. It also has the appropriate conversions for the P,I,and D gains (from SI to mV and encoder pulses).

_profileBuilder.ino_ has the logic to create trapezoidal velocity profiles, and integrate the position from the profile.