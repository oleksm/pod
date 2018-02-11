

#include "rcinput.h"

Joystick::Joystick(int vrx, int vry, int sw)
{
  sw_pin = sw;
  vrx_pin = vrx;
  vry_pin = vry;

  pinMode(sw_pin, INPUT);
  digitalWrite(sw_pin, HIGH);
}

jstate_t* Joystick::getState()
{
  return &state;
}

void Joystick::setState(jstate_t state)
{
  this->state = state;
}


void Joystick::update()
{
  int sw = digitalRead(sw_pin);
  int vrx = analogRead(vrx_pin);
  int vry = analogRead(vry_pin);

  int x = abs(vrx - 512);
  if ( x < J_NOISE_FILTER)
  {
    x = 0;
  }
  if (x > 512 - J_NOISE_FILTER)
  {
    x = 512;
  }
  
  int y = abs(vry - 512);
  if ( y < J_NOISE_FILTER)
  {
    y = 0;
  }
  if (y > 512 - J_NOISE_FILTER)
  {
    y = 512;
  }

  byte xflag = (x == 0 || vrx <= 512 ? 1 : 0) << 1;
  byte yflag = (y == 0 || vry <= 512 ? 1 : 0) << 2;

  state.flags = sw | xflag | yflag;
  state.x = (byte)map(x, 0, 512, 0, 255);
  state.y = (byte)map(y, 0, 512, 0, 255);
}

bool Joystick::isSw() 
{
  return (state.flags & B00000001) == 1;
}

byte Joystick::getUp()
{
  return (state.flags & B00000010) == 1 ? state.y : 0;
}

byte Joystick::getDown()
{
  return (state.flags & B00000010) == 0 ? state.y : 0;
}

byte Joystick::getLeft()
{
  return (state.flags & B00000100) == 1 ? state.x : 0;
}

byte Joystick::getRight()
{
  return (state.flags & B00000100) == 0 ? state.x : 0;
}

void Joystick::print()
{
  Serial.print(state.flags, BIN);
  Serial.print(" ");
  Serial.print(state.x);
  Serial.print(" ");
  Serial.print(state.y);
}


