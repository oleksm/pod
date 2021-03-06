
#include "multiwii.h"
#include "rcinput.h"

Multiwii::Multiwii()
{
  connected = 0;
}

void Multiwii::connect()
{
  msp.begin(Serial);
}

bool Multiwii::refresh()
{
  uint16_t i2cErrorCounter = status.i2cErrorCounter;
  bool res = msp.request(MSP_STATUS, &status, sizeof(status));
  error = i2cErrorCounter != status.i2cErrorCounter;
  return res;
}

boolean Multiwii::isError()
{
  return error;
}

boolean Multiwii::isArmed()
{
  // MSP_MODE_ARM BIT 0
  return status.flightModeFlags & 1;
}

bool Multiwii::disarm()
{
  msp_rc_t rc;
  if (msp.request(MSP_RC, &rc, sizeof(rc)))
  {
    // THROTTLE
    rc.channelValue[3] = SERVO_MIN;
    // YAW
    rc.channelValue[2] = SERVO_MIN;
     
    if (msp.request(MSP_SET_RAW_RC, &rc, sizeof(rc)))
    {
      delay(300);
      if (refresh() && !isArmed())
      {
        delay(300);
             // THROTTLE
        rc.channelValue[3] = SERVO_MID;
        // YAW
        rc.channelValue[2] = SERVO_MID;
        if (msp.request(MSP_SET_RAW_RC, &rc, sizeof(rc)))
        {
          delay(300);
          return refresh() && !isArmed();
        }
      }
    }
  }
  return 0;
}
