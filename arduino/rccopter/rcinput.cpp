

#include "rccopter.h"
#include "rcinput.h"

RCInput::RCInput()
{
}

void RCInput::begin()
{
  left.begin(9, 0, 1);
  right.begin(10, 3, 2);
  blue.begin(5, 80);
  yellow.begin(4, 150);
  red.begin(3, 80);
  // buizzer 6
  b_connect.begin(A6, 930);
  b_disarm.begin(A6, 509);
  b_home.begin(A6, 677);
  b_menu.begin(A6, 852);
}

uint16_t vr2servo(uint16_t vr)
{
  if (vr >= J_MID)
  {
    return map(vr, J_MID, J_MAX, SERVO_MID, SERVO_MAX);
  } 
  return map(vr, J_MIN, J_MID, SERVO_MIN, SERVO_MID);
}

void RCInput::refresh(void* cptr)
{
  RCCopter *copter = (RCCopter*)cptr;
  if (left.refresh() | right.refresh())
  {
    // LEFT VRY - Throttle
    if (left.isVrYChanged())
    {
      uint16_t throttle = vr2servo(left.getVrY());
      copter->setThrottle(throttle);
    }
    // LEFT VRX - ROLL
    if (left.isVrXChanged())
    {
      uint16_t roll = vr2servo(left.getVrX());
      copter->setRoll(roll);
    }
    // RIGHT VRY - PITCH
    if (right.isVrYChanged())
    {
      uint16_t pitch = vr2servo(right.getVrY());
      copter->setPitch(pitch);
    }
    // RIGHT VRX - YAW
    if (right.isVrXChanged())
    {
      uint16_t yaw = vr2servo(right.getVrX());
      copter->setYaw(yaw);
    }
  }

  if (b_connect.isPressed())
  {   
    if (copter->isConnected())
      copter->disconnect();
    else
      copter->connect();
  }

  if (b_disarm.isPressed() && copter->isArmed())
  {
    copter->disarm();
  }

  if (b_home.isPressed() && copter->isFlying())
  {
    copter->returnHome();
  }

  blue.set(copter->isConnected());
  yellow.set(copter->isArmed());
  red.set(copter->isError());
  //TODO: Add menu code when my new cheap chinese screen delivered
}

//=============== JOYSTICK ==================

uint16_t filter_noise(int v)
{
  if (abs(J_MID - v) <= J_NOISE_TRESHOLD)
    return J_MID;
  if (abs(J_MAX - v) <= J_NOISE_TRESHOLD)
    return J_MAX;
  if (abs(J_MIN - v) <= J_NOISE_TRESHOLD)
    return J_MIN;
  return v;
}

Joystick::Joystick()
{
  
}

void Joystick::begin(int sw, int vrx, int vry)
{
  sw_pin = sw;
  // Oops I messed up with joystick positioning on my controller
  vrx_pin = vry;
  vry_pin = vrx;
  
  pinMode(sw_pin, INPUT);
  digitalWrite(sw_pin, HIGH);
  refresh();
}



bool Joystick::refresh()
{
  int new_sw = !digitalRead(sw_pin);
  // A little bit more transformation to fix my messed joystick positioning
  int new_vrx = filter_noise(abs(1024 - analogRead(vrx_pin)));
  int new_vry = filter_noise(abs(1024 - analogRead(vry_pin)));

  if (is_sw_changed = new_sw != sw)
    sw = new_sw;
  if (is_vrx_changed = new_vrx != vrx)
    vrx = new_vrx;
  if (is_vry_changed = new_vry != vry)
    vry = new_vry;

  return is_sw_changed || is_vrx_changed || is_vry_changed;
}

bool Joystick::isSw()
{
  return sw;
}

uint16_t Joystick::getVrX()
{
  return vrx;
}

uint16_t Joystick::getVrY()
{
  return vry;
}

bool Joystick::isSwChanged()
{
  return is_sw_changed;
}
bool Joystick::isVrXChanged()
{
  return is_vrx_changed;
}
bool Joystick::isVrYChanged()
{
  return is_vry_changed;
}

    
void Joystick::printDetails()
{
  Serial.print("sw: ");
  Serial.print(sw);

  Serial.print(" vrx: ");
  Serial.print(vrx);
  Serial.print(" vry: ");
  Serial.println(vry);
}


// ====================== LED ========================

LED::LED()
{
}

void LED::begin(int pin, int intensity)
{
  this->pin = pin;
  this->intensity = intensity;
  pinMode(pin, OUTPUT);
  value = LOW;
}

void LED::set(bool light)
{
  uint8_t v = light ? intensity : LOW;
  if (v != value)
  {
    analogWrite(pin, v);
    value = v;
  }
}

// ====================== ANALOG BUTTON =====================

AnalogButton::AnalogButton()
{
  state = 0;
}

void AnalogButton::begin(int pin, uint16_t high)
{
    this->pin = pin;
    this->high = high;
    pinMode(pin, INPUT);
}

bool AnalogButton::isPressed()
{
  int v = analogRead(pin);
  bool pressed = v > high - 20 && v < high + 20;
  if (pressed != state)
  {
    state = pressed;
    return pressed;
  }
  return 0;
}


// ======================= Buzzer ==========================

void Buzzer::begin(int pin)
{
  this->pin = pin;
  pinMode(pin, OUTPUT);
  play(MELODY_POWER_ON);
}

void Buzzer::play(uint8_t melody)
{
//  switch (melody)
//  {
//    case MELODY_POWER_ON:
//      tone(pin, 800, 100);
//      delay(100);
//      tone(pin, 800, 100);
//      delay(100);
//      tone(pin, 800, 100);
//      break;
//    case MELODY_ERROR_RADIO:
//      tone(pin, 1000, 300);
//      delay(50);
//      tone(pin, 500, 300);
//      break;
//  }
}

// ===================== Timer ============

void Timer::set(uint8_t id, long recurrence)
{
  this->recurrence[id] = recurrence;
  timer[id] = millis();
}

bool Timer::trigger(uint8_t id)
{
  long ms = millis();
  long elapsed = ms - timer[id];
  if (elapsed >= recurrence[id])
  {
    timer[id] =  ms;
    return 1;
  }
  return 0;
}




