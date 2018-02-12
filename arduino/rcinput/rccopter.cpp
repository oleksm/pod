
#include "rccopter.h"

RCCopter::RCCopter()
{
}

void RCCopter::connect()
{
  radio = new RCRadio(TRANSMITTER_PIN_CE, TRANSMITTER_PIN_CSN,TRANSMITTER_ADDRESS, RECEIVER_ADDRESS, *rccopter_receive, this);
}

void RCCopter::arm()
{
  radio->send(CMD_ARM, 0);
}

void RCCopter::disarm()
{
  radio->send(CMD_DISARM, 0);
}

void RCCopter::returnHome()
{
  radio->send(CMD_HOME, 0);
}

void RCCopter::ping()
{
  radio->send(CMD_PING, 0);
}

void RCCopter::setThrottle(uint16_t throttle)
{
  Serial.println("SetThrottle");
  radio->send(CMD_THROTTLE, throttle);
}

void RCCopter::setYaw(uint16_t yaw)
{
  radio->send(CMD_YAW, yaw);
}

void RCCopter::setPitch(uint16_t pitch)
{
  radio->send(CMD_PITCH, pitch);
}

void RCCopter::setRoll(uint16_t roll)
{
    radio->send(CMD_ROLL, roll);
}

void RCCopter::setAux1(uint16_t aux1)
{
    radio->send(CMD_AUX1, aux1);
}

void RCCopter::setAux2(uint16_t aux2)
{
    radio->send(CMD_AUX2, aux2);
}

//=========================== Copter =========================


Copter::Copter()
{
}


void Copter::connect()
{
    radio = new RCRadio(RECEIVER_PIN_CE, RECEIVER_PIN_CSN, RECEIVER_ADDRESS,TRANSMITTER_ADDRESS, *copter_receive, this);
    s_throttle.attach(COPTER_PIN_THROTTLE);
    s_yaw.attach(COPTER_PIN_YAW);
    s_pitch.attach(COPTER_PIN_PITCH);
    s_roll.attach(COPTER_PIN_ROLL);
    s_aux1.attach(COPTER_PIN_AUX1);
    s_aux2.attach(COPTER_PIN_AUX2);
}

void Copter::arm()
{
  setThrottle(SERVO_MIN);
  setAux1(SERVO_MID);
}

void Copter::disarm()
{
  setAux1(SERVO_MIN);
  setThrottle(SERVO_MIN);
}

void Copter::returnHome()
{
  // TODO: How to RETURN home properly?
  disarm();
}

void Copter::pong()
{
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
  radio->receive();  
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
    break;
  default:
    Serial.print("Copter received unknown command: ");
    Serial.println(cmd);
    break;
  }
}


void copter_receive(void* cptr, uint8_t cmd, uint16_t data)
{
  Copter *copter = (Copter*)cptr;

  Serial.print ("Copter Received cmd: ");
  Serial.print(cmd);
  Serial.print(" data: ");
  Serial.println(data);

  switch(cmd)
  {
  case CMD_PING:
    copter->pong();
    break;
  case CMD_ARM:
    copter->arm();
    break;
  case CMD_DISARM:
    copter->disarm();
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
    Serial.print("Copter received unknown command: ");
    Serial.println(cmd);
    break;
  }
}

