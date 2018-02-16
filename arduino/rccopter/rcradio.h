
#ifndef __RCRADIO_H__
#define __RCRADIO_H__

#include <Arduino.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

#define TRANSMITTER_PIN_CE 7
#define TRANSMITTER_PIN_CSN 8
#define RECEIVER_PIN_CE 8
#define RECEIVER_PIN_CSN 7

#define RECEIVE_CALLBACK_FUNCTION void (*receive_callback)(void* copter, uint8_t cmd, uint16_t data)


#define TRANSMITTER_ADDRESS 0x85a4883bbc63
#define RECEIVER_ADDRESS 0xb0e267f98e70

class RCRadio
{
  public:
    RCRadio(int ce, int csn);
    void begin(uint64_t send_address, uint64_t receive_address, RECEIVE_CALLBACK_FUNCTION, void* copter);
    bool send(uint8_t cmd, uint16_t data);
    void receive();
  private:
    RF24 radio;
    RECEIVE_CALLBACK_FUNCTION;
    void* copter;
};

#endif
