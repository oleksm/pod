#include <Event.h>
#include <Timer.h>
#include <FlexiTimer2.h>
#include <Wire.h>

#include "Chassis.h"

// Sensors Measuments per Second
const int PHOTO_MPS = 2000;

// Photo Cells Pins
const byte PHOTO11 = A2;
const byte PHOTO12 = A3;
const byte PHOTO21 = A6;
const byte PHOTO22 = A7;

// Pseudo parallel running
Timer timer;

Chassis chassis;

bool blink;

// total readings on each sensor
long readings = 0;

void setup()
{
  Serial.begin(115200);
  pinMode(PHOTO11, INPUT);
  pinMode(PHOTO12, INPUT);
  pinMode(PHOTO21, INPUT);
  pinMode(PHOTO22, INPUT);

  chassis.setup();
  timer.every(200, updateMotorsStats);
  timer.every(ADJUST_NAVIGATION_TIMER + 20, adjustNavigation);
  timer.every(1000, updateBlink);

  readings = 0;
  FlexiTimer2::set(1, 1.0 / PHOTO_MPS, readSensors);
  FlexiTimer2::start();

  // Blink 5 times to indicate chassis is ready
  updateBlink();
  delay(100);
  updateBlink();
  delay(100);
  updateBlink();
  delay(100);
  updateBlink();
  delay(100);
  updateBlink();
  delay(100);
  updateBlink();
  delay(100);
  updateBlink();
  delay(100);
  updateBlink();
  delay(100);
  updateBlink();
  delay(100);
}

/*
 * READ SENSORS
 */
void readSensors() {
  ++readings;
  byte v11 = analogRead(PHOTO11);
  byte v12 = analogRead(PHOTO12);
  byte v21 = analogRead(PHOTO21);
  byte v22 = analogRead(PHOTO22);
  chassis.update_photo_state(v11, v12, v21, v22);
}

void updateBlink() {
  if (blink) {
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  }
  else {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  }
  blink = !blink;
}
  
void loop()
{
  timer.update();
}

void adjustNavigation() {
  FlexiTimer2::stop();
  chassis.adjust_navigation();
  FlexiTimer2::start();
}

void updateMotorsStats() {
  FlexiTimer2::stop();
  chassis.update_motors_stats(millis());
  FlexiTimer2::start();  
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveWireEvent(int howMany) {
  FlexiTimer2::stop();
  chassis.recieve_wire_event();
  FlexiTimer2::start();
}


