
#include "rcradio.h"


RCRadio::RCRadio(int ce, int csn, uint64_t send_address, uint64_t receive_address, RECEIVE_CALLBACK_FUNCTION)
:
  radio(ce, csn)
{
  Serial.println("Initializing RCRadio");
  radio.begin();
  radio.setDataRate( RF24_250KBPS );
  radio.setRetries(3,5); // delay, count
  radio.setAutoAck(1);
  radio.openWritingPipe(send_address);
  radio.openReadingPipe(1, receive_address);
  radio.startListening();
  // RECEIVE_CALLBACK_FUNCTION
  this->receive_callback = receive_callback;
}

bool RCRadio::send(uint8_t cmd, uint16_t data)
{
  data |= cmd << 12;
  radio.stopListening();
  long ms = millis();
  bool success = radio.write( &data, sizeof(data));
  long ms1 = millis();
  radio.startListening();
  Serial.print("Radio transmission status: ");
  Serial.print(success);
  Serial.print(" ms: ");
  Serial.print(ms1 - ms);
  Serial.print(" data: ");
  Serial.println(data, HEX);
  return success;
}

void RCRadio::receive()
{
  uint16_t data;
  if ( radio.available() ) {
    radio.read( &data, sizeof(uint16_t));

    uint16_t cmd = data >> 12;
    data &= 0x0FFF;
    receive_callback(cmd, data);
  }
}

