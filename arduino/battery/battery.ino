
/*
  for arduino uno
  testing sketch for the ACS712 current sensor module

  PARAMETERS:
  vcc: 4.5 - 5.5 v
  icc: 13 - 15 ma
  zero current output voltage: vcc × 0.5 (2.5 v)

  SENSITIVITY:
  +- 5 a: 185 mv/a (+-13513 mamper @ 5v)
  +-20 a: 100 mv/a (+-25000 mamper @ 5v)
  +-30 a:  66 mv/a (+-37878 mamper @ 5v)

  theoretical resolution for 5v analog pin:
  (arduino resolution: 4.8828 mv)
   5a: 26.4ma
  20a: 48.8ma
  30a: 74.0ma

  error @ 25c: +-1.5 %

  PIN MAP:
  module -> arduino
  vcc    -> 5v
  gnd    -> gnd
  out    -> analog pin (a0)
*/

#include "TM1651.h"
#define CLK 3  //pins definitions for TM1651 and can be changed to other ports       
#define DIO 2
#define VOLT A1
#define BUTTON 3
#define ACS          A0
#define SENSITIVITY  145    // acs712 model SENSITIVITY in mv
#define SAMPLING    100.0  // set samples before averaging

int button_state = 0;
float amps = 0.0;
float volts = 0.0;
int charging = 0;

TM1651 display(CLK,DIO);
int display_state = -1;

void setup() {
  display.init();
  pinMode(ACS, INPUT);
  Serial.begin(9600);
}
void loop()
{
  update_button();
  update_amps();
  update_voltage();
  update_display();
  update_telemetry();
  delay(200);
}

// DISPLAY
void update_display() {
  if (display_state != button_state) {
    if (button_state == 1) {
      int level = map(volts * 100, 680, 780, 1, 5);
      display.displayLevel(level);
      if (charging == 1) {
          display.frame(FRAME_ON);
      }
    }
    else {
      display.clearDisplay();
    }
    display_state = button_state;
  }
}

// BUTTON
void update_button()
{
  button_state = charging == 1 ? 1 : analogRead(BUTTON) > 512 ? 1 : 0;
}

// AMPS
void update_amps()
{
  float avgAmps;
  for (int i = 0; i < SAMPLING; i++) {
    int rawACS = analogRead(ACS);
    float mv = rawACS / 1023.0 * 5000.0;
    float rawAmps = (mv - 2500.0) / SENSITIVITY;  // set sensor calibration here
    avgAmps += rawAmps;
  }
  amps = avgAmps / SAMPLING;
  charging = amps < -0.5;
}

// VOLTS
void update_voltage() {
  float tmp = ((float)analogRead(VOLT)) / 4.092;  
  volts = tmp / 10;
}

// TELEMETRY
void update_telemetry() {
  char amps_text[30];
  char volts_text[30];

  dtostrf(amps, 10, 10, amps_text);
  dtostrf(volts, 10, 10, volts_text);

  char text[62];
  snprintf(text, 62, "%s,%s", amps_text, volts_text);
  Serial.println(text);
}

