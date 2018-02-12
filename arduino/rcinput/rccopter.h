
#ifndef __RCCOPTER_H__
#define __RCCOPTER_H__

#include <Arduino.h>
#include <Servo.h>

#include "rcradio.h"

#define AUX_LOW 0
#define AUX_high 255
#define AUX_MID 127

#define SERVO_MIN 1020
#define SERVO_MID 1500
#define SERVO_MAX 2000

// Receiver Outbound pins
#define COPTER_PIN_THROTTLE 3
#define COPTER_PIN_YAW 9
#define COPTER_PIN_PITCH 6
#define COPTER_PIN_ROLL 5
#define COPTER_PIN_AUX1 10
#define COPTER_PIN_AUX2 4

#define CMD_PING 1
#define CMD_PONG 1
#define CMD_ARM 3
#define CMD_DISARM 4
#define CMD_HOME 5
#define CMD_THROTTLE 6
#define CMD_YAW 7
#define CMD_PITCH 8
#define CMD_ROLL 9
#define CMD_AUX1 10
#define CMD_AUX2 11

class RCCopter
{
  public:
    RCCopter();
    void connect();
    void arm();
    void disarm();
    void returnHome();
    void ping();
    void setThrottle(uint16_t throttle);
    void setYaw(uint16_t yaw);
    void setPitch(uint16_t pitch);
    void setRoll(uint16_t roll);
    void setAux1(uint16_t aux1);
    void setAux2(uint16_t aux2);
    
  private:
    RCRadio *radio;
    uint8_t throttle;
    uint8_t yaw;
    uint8_t pitch;
    uint8_t roll;
    uint8_t aux1;
    uint8_t aux2;
};

class Copter
{
  public:
    Copter();
    void connect();
    void arm();
    void disarm();
    void returnHome();
    void pong();
    void setThrottle(uint16_t throttle);
    void setYaw(uint16_t yaw);
    void setPitch(uint16_t pitch);
    void setRoll(uint16_t roll);
    void setAux1(uint16_t aux1);
    void setAux2(uint16_t aux2);
    void loop();
  private:
    RCRadio *radio;
    Servo s_throttle;
    Servo s_yaw;
    Servo s_pitch;
    Servo s_roll;
    Servo s_aux1;
    Servo s_aux2;
};

void rccopter_receive(void* copter, uint8_t cmd, uint16_t data);
void copter_receive(void* copter, uint8_t cmd, uint16_t data);


#endif

