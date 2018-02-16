
#include "rccopter.h"

// Speify target
// RCCopter - Remote Control (joystick hardware)
// Copter - Onboard Control (copter hardware)
RCCopter copter;

void setup() {
  copter.begin();
  Serial.println("Setup Complete");
}

void loop() {
  copter.loop();
  delay(50);
}

