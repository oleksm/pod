
#include "rccopter.h"


RCCopter::RCCopter()
:
  radio(TRANSMITTER_PIN_CE, TRANSMITTER_PIN_CSN)
{
}

void RCCopter::begin()
{
  timer.set(TIMER_PING, 2000);
  radio.begin(TRANSMITTER_ADDRESS, RECEIVER_ADDRESS, *rccopter_receive, this);
  input.begin();
  buzzer.begin(6);
  status = 0;
  Serial.begin(57600);
}
bool RCCopter::connect()
{
  if (!radio.send(CMD_CONNECT, 0))
    buzzer.play(MELODY_ERROR_RADIO);
}

void RCCopter::disconnect()
{
  if (!radio.send(CMD_DISCONNECT, 0))
    buzzer.play(MELODY_ERROR_RADIO);
}

void RCCopter::disarm()
{
  if(!radio.send(CMD_DISARM, 0))
    buzzer.play(MELODY_ERROR_RADIO);
}

void RCCopter::returnHome()
{
  if(!radio.send(CMD_HOME, 0))
    buzzer.play(MELODY_ERROR_RADIO);
}

void RCCopter::ping()
{
  if(!radio.send(CMD_PING, 0))
    buzzer.play(MELODY_ERROR_RADIO);
}

void RCCopter::setThrottle(uint16_t throttle)
{
  if(!radio.send(CMD_THROTTLE, throttle))
    buzzer.play(MELODY_ERROR_RADIO);
}

void RCCopter::setYaw(uint16_t yaw)
{
  if(radio.send(CMD_YAW, yaw))
    buzzer.play(MELODY_ERROR_RADIO);
}

void RCCopter::setPitch(uint16_t pitch)
{
  if(!radio.send(CMD_PITCH, pitch))
    buzzer.play(MELODY_ERROR_RADIO);
}

void RCCopter::setRoll(uint16_t roll)
{
  if(radio.send(CMD_ROLL, roll))
    buzzer.play(MELODY_ERROR_RADIO);
}

void RCCopter::setAux1(uint16_t aux1)
{
  if(!radio.send(CMD_AUX1, aux1))
    buzzer.play(MELODY_ERROR_RADIO);
}

void RCCopter::setAux2(uint16_t aux2)
{
  if(!radio.send(CMD_AUX2, aux2))
    buzzer.play(MELODY_ERROR_RADIO);
}

bool RCCopter::isConnected()
{
  return status & B00000001;
}

bool RCCopter::isArmed()
{
  return status & B00000010;
}

bool RCCopter::isError()
{
  return status & B00000100;
}

bool RCCopter::isFlying()
{
    return status & B00001000;
}

void RCCopter::pong(uint8_t status)
{
    this->status = status;
    Serial.print("Status: ");
    Serial.println(status, BIN);
}

void RCCopter::loop()
{
  radio.receive();
  input.refresh(this);

  if (timer.trigger(TIMER_PING))
  {
    ping();
  }
}

void rccopter_receive(void* cptr, uint8_t cmd, uint16_t data)
{
  RCCopter *copter = (RCCopter*)cptr;
  
  Serial.print ("Copter Received cmd: ");
  Serial.print(cmd);
  Serial.print(" data: ");
  Serial.println(data);

  switch(cmd)
  {
  case CMD_PONG:
    copter->pong(data);
    break;
  default:
    Serial.print("Copter received unknown command: ");
    Serial.println(cmd);
    break;
  }
}

//=========================== Copter =========================


Copter::Copter()
:
  radio(RECEIVER_PIN_CE, RECEIVER_PIN_CSN)
{
  connected = 0;
}

void Copter::begin()
{
  radio.begin(RECEIVER_ADDRESS,TRANSMITTER_ADDRESS, *copter_receive, this);
}

bool Copter::connect()
{
  if (!connected)
  {
    Serial.begin(115200);
    s_throttle.attach(COPTER_PIN_THROTTLE);
    s_yaw.attach(COPTER_PIN_YAW);
    s_pitch.attach(COPTER_PIN_PITCH);
    s_roll.attach(COPTER_PIN_ROLL);
    s_aux1.attach(COPTER_PIN_AUX1);
    s_aux2.attach(COPTER_PIN_AUX2);
    multiwii.connect();
    connected = 1;
  }
  return connected;
}

void Copter::disconnect()
{
  if (connected)
  {
    disarm();
    s_throttle.detach();
    s_yaw.detach();
    s_pitch.detach();
    s_roll.detach();
    s_aux1.detach();
    s_aux2.detach();
    Serial.end();
    connected = 0;
  }
}

void Copter::disarm()
{
  bool res = 0;
  if (connected)
  {
    if (!multiwii.disarm())
    {
      setThrottle(SERVO_MIN);
      setYaw(SERVO_MIN);
      delay(300);
      setYaw(SERVO_MID);
      setThrottle(SERVO_MID);
    }
  }
}

void Copter::returnHome()
{
  // TODO: How to RETURN home properly?
  disarm();
}

void Copter::pong()
{
 
  uint8_t status = 0;
  if (connected)
  {
    multiwii.refresh();
    status = multiwii.isError() << 2 | multiwii.isArmed() << 1 | 1;
  }
  radio.send(CMD_PONG, status);
}

void Copter::setThrottle(uint16_t throttle)
{
  s_throttle.writeMicroseconds(throttle);
}

void Copter::setYaw(uint16_t yaw)
{
  s_yaw.writeMicroseconds(yaw);
}

void Copter::setPitch(uint16_t pitch)
{
  s_pitch.writeMicroseconds(pitch);
}

void Copter::setRoll(uint16_t roll)
{
  s_roll.writeMicroseconds(roll);
}

void Copter::setAux1(uint16_t aux1)
{
  s_aux1.writeMicroseconds(aux1);
}

void Copter::setAux2(uint16_t aux2)
{
  s_aux2.writeMicroseconds(aux2);
}

void Copter::loop()
{
  radio.receive();
}

void copter_receive(void* cptr, uint8_t cmd, uint16_t data)
{
  Copter *copter = (Copter*)cptr;

  if (DEBUG_ON)
  {
    Serial.print("Copter Received cmd: ");
    Serial.print(cmd);
    Serial.print(" data: ");
    Serial.println(data);    
  }

  switch(cmd)
  {
  case CMD_PING:
    break;
  case CMD_DISARM:
    copter->disarm();
    break;
  case CMD_CONNECT:
    copter->connect();
    break;
  case CMD_DISCONNECT:
    copter->disconnect();
    break;
  case CMD_HOME:
    copter->returnHome();
    break;
  case CMD_THROTTLE:
    copter->setThrottle(data);
    break;
  case CMD_YAW:
    copter->setYaw(data);
    break;
  case CMD_PITCH:
    copter->setPitch(data);
    break;
  case CMD_ROLL:
    copter->setRoll(data);
    break;    
  case CMD_AUX1:
    copter->setAux1(data);
    break;
  case CMD_AUX2:
    copter->setAux2(data);
    break;
  default:
    if (DEBUG_ON)
    {
      Serial.print("Copter received unknown command: ");
      Serial.println(cmd);      
    }
    break;
  }
  copter->pong();
}

