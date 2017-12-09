#include "Chassis.h"

// Arduino pin numbers
const int SW_pin = 2; // digital pin connected to switch output
const int X_pin = 0; // analog pin connected to X output
const int Y_pin = 1; // analog pin connected to Y output


RemoteChassis chassis;

float kp;
float ki;
float kd;

long rotations = 1000;

void setup() {
  Serial.println("Starting up");
  chassis.setup();
  Serial.begin(115200);

  pinMode(SW_pin, INPUT);
  digitalWrite(SW_pin, HIGH);

  kp = 0.3;
  ki = 0.3;
  kd = 0.3;
  
  Serial.println("Connected to Remote Chassis.");
}

void loop() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    switch(c) {
      case 'p':
      case 'P':
        kp = Serial.parseFloat();
        break;
      case 'i':
      case 'I':
        ki = Serial.parseFloat();
        break;
      case 'd':
      case 'D':
        kd = Serial.parseFloat();
        break;
    }
    chassis.set_pid(kp, ki, kd);
    Serial.print("Set ");
    Serial.print(c);
    Serial.print(" - New kp: ");
    Serial.print(kp);
    Serial.print(", ki: ");
    Serial.print(ki);
    Serial.print(", kd: ");
    Serial.println(kd);
  }

  int sw = digitalRead(SW_pin);
  int x = analogRead(X_pin);
  int y = analogRead(Y_pin);
//  Serial.print("Rotations: ");
//  Serial.print(rotations);
//  Serial.print("\n");
//  Serial.print("Switch:  ");
//  Serial.print(sw);
//  Serial.print("\n");
//  Serial.print("X-axis: ");
//  Serial.print(x);
//  Serial.print("\n");
//  Serial.print("Y-axis: ");
//  Serial.println(y);
//  Serial.print("\n\n");
  delay(500);

  // Left
  if (y > 1000) rotations -=100;
  // Right
  if (y < 100) rotations +=100;
  // Up
  if (x < 100) rotations +=1000;
  // Down
  if (x > 1000) rotations -=1000;
  // Trigger
  if (sw == 0) {
    chassis.move_rot(rotations);
  }
}

