
#ifndef __RCINPUT_H__
#define __RCINPUT_H__

#include <Arduino.h>

#define J_NOISE_FILTER 11
#define J_MASK_SW B00000001
#define J_MASK_LR B00000010
#define J_MASK_UD B00000010

union jstate_t {
  struct {
    byte x;
    byte y;
    byte flags;
  };
  byte data[3];
};

class Joystick
{
  public:
    Joystick(int sw, int vrx, int vry);
    jstate_t* getState();
    void setState(jstate_t state);
    void update();
    bool isSw();
    byte getUp();
    byte getDown();
    byte getLeft();
    byte getRight();
    void print();
  private:
    jstate_t state;
    byte sw_pin;
    byte vrx_pin;
    byte vry_pin;
};


#endif

