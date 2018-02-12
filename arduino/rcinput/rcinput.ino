
#include "rcinput.h"
#include "rccopter.h"

//
//Joystick lj(9, 0, 1);
//
//
//Joystick rj(10, 2, 3);
////RCCopter copter;

Copter copter;

void setup() {
  Serial.begin(57600);
  copter.connect();
  Serial.println("Setup Complete");
}

void loop() {
  copter.loop();
}

