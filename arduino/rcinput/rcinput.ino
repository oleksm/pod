
#include "rcinput.h"
#include "rccopter.h"


Joystick lj(1, 0, 9);
Joystick rj(2, 3, 10);
RCCopter copter;

void setup() {
  Serial.begin(57600);
  copter.connect();
  Serial.println("Setup Complete");
}

void loop() {
  Serial.println("Loop");
  lj.update();
  rj.update();
  uint16_t throttle = lj.getUp();
  copter.setThrottle(throttle);

  delay(200);
}

