#ifndef __MULTIWII_H__
#define __MULTIWII_H__

#include <MSP.h>
#include <Arduino.h>


class Multiwii
{
  public:
    Multiwii();
    void connect();
    bool refresh();
    bool isError();
    bool isArmed();
    bool disarm();
  private:
    MSP msp;
    msp_status_t status;
    bool error;
    bool connected;
};

#endif
