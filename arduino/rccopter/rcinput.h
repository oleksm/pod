
#ifndef __RCINPUT_H__
#define __RCINPUT_H__

#include <Arduino.h>

#define J_MIN 0
#define J_MID 512
#define J_MAX 1024

#define J_NOISE_TRESHOLD 12

#define SERVO_MIN 1020
#define SERVO_MID 1500
#define SERVO_MAX 2000

class Joystick
{
  public:
    Joystick();
    void begin(int sw, int vrx, int vry);
    bool refresh();
    bool isSw();
    uint16_t getVrX();
    uint16_t getVrY();
    bool isSwChanged();
    bool isVrXChanged();
    bool isVrYChanged();
    void printDetails();
  private:
    bool sw;
    uint16_t vrx;
    uint16_t vry;
    bool is_vrx_changed;
    bool is_vry_changed;
    bool is_sw_changed;
    int sw_pin;
    int vrx_pin;
    int vry_pin;
};

class LED
{
  public:
    LED();
    void begin(int pin, int intensity);
    void set(bool light);
  private:
    uint8_t pin;
    uint8_t intensity;
    uint8_t value; 
};

class AnalogButton
{
  public:
    AnalogButton();
    void begin(int pin, uint16_t high);
    bool isPressed();
  private:
    int pin;
    uint16_t high;
    bool state;
};

#define MELODY_POWER_ON 0
#define MELODY_ERROR_RADIO 1

class Buzzer
{
  public:
    void begin(int pin);
    void play(uint8_t melody);
  private:
    uint8_t pin;
};


class RCInput
{
  public:
    RCInput();
    void begin();
    void refresh(void* copter);
  private:
    LED blue;
    LED yellow;
    LED red;
    AnalogButton b_connect;
    AnalogButton b_disarm;
    AnalogButton b_home;
    AnalogButton b_menu;    
    Joystick left;
    Joystick right;
};

#define TIMER_SIZE 1

class Timer
{
  public:
    void set(uint8_t id, long recurrence);
    bool trigger(uint8_t id);
  private:
    long timer[TIMER_SIZE];
    long recurrence[TIMER_SIZE];
};



#endif

