
#include "rccopter.h"

// Speify target
// RCCopter - Remote Control (joystick hardware)
// Copter - Onboard Control (copter hardware)
Copter copter;

void setup() {
  if (DEBUG_ON)
    Serial.begin(115200);
  copter.begin();
  
  if (DEBUG_ON)
    Serial.println("Setup Complete");
  
}

void loop() {
  copter.loop();
  delay(10);
}

