#include <Wire.h>
#include "Chassis.h"
#include "IC2_DataTypes.h"

Chassis::Chassis() {
}

void Chassis::setup() {
  motor_init(&motor1);
  motor_init(&motor2);

  // Attach motor drive pins
  pinMode(M1DIR,OUTPUT);
  pinMode(M1SPEED,OUTPUT);
  pinMode(M2DIR,OUTPUT);
  pinMode(M2SPEED,OUTPUT);

  Wire.begin(CHASSIS_DEVICE_ADDR);                    // join i2c bus with address #8
  Wire.onReceive(receiveWireEvent); // register event
}

/*
 * Communication with control board
 */
void Chassis::recieve_wire_event() {
  while (Wire.available() > 0) {
    byte cmd = Wire.read();
    switch (cmd) {
    case CMD_SET_SPEED: {
    	byte speed = Wire.read();
    	set_speed(speed);
    	break;
    }
    case CMD_MOVE_ROT: {
      ic2_long_t rot;
      wire_read_bytes(rot.data, sizeof(ic2_long_t));
      move_rot(rot.l);
      break;
    }
    case CMD_SET_PID: {
      ic2_pid_t pid;
      wire_read_bytes(pid.data, sizeof(ic2_pid_t));
      set_pid(pid.ki, pid.kp, pid.kd);
      break;
    }
    }
  }
}

/*
 * 
 */
void Chassis::move_rot(long rot) {
//  Serial.print("Move Rotations: ");
//  Serial.println(rot);

  motor_move_init(&motor1, rot);
  motor_move_init(&motor2, rot);
}

/*
 * Control Real speed of both motors based on PID
 */
void Chassis::adjust_navigation() {
  bool m1 = motor_adjust_navigation(&motor1);
//  bool m2 = motor_adjust_navigation(&motor2);
  if (m1) {    
    motor_t &m = motor1;
    analogWrite(M1SPEED, motor1.pwm);
    digitalWrite(M1DIR, DIR_CW);

    // Use same PID and coefficient for better wheels sync;
    motor2.pwm = motor1.pwm * differential;

    analogWrite(M2SPEED, motor2.pwm);
    digitalWrite(M2DIR, LOW);

    telemetry(m.Input, m.pwm, m.Setpoint, 0.0f, m.pid->GetKi(), m.pid->GetKp(), m.pid->GetKd());
  }
//  if (m2) {
//  }
}

/*
 * Set Speed 0-255
 */
void Chassis::set_speed(byte speed) {
	Serial.print("Set Speed: ");
	Serial.println(speed);
}

/*
 * 
 */
void Chassis::update_motors_stats(long ms) {
  motor_stats(&motor1, ms);
  motor_stats(&motor2, ms);
}

/*
 * 
 */
void Chassis::update_photo_state(byte v11, byte v12, byte v21, byte v22) {
  photo_update(&motor1.photo1, v11);
  photo_update(&motor1.photo2, v12);
  photo_update(&motor2.photo1, v21);
  photo_update(&motor2.photo2, v22);
//  Serial.print(v21);
//  Serial.print(" ");
//  Serial.print(v22);
//  Serial.println();
}

/*
 * 
 */
void Chassis::set_pid(double kp, double ki, double kd) {
  motor1.pid->SetTunings(kp, ki, kd);
  motor2.pid->SetTunings(kp, ki, kd);
}

//----------------- Remote Chassis ------------------

RemoteChassis::RemoteChassis() {
}

void RemoteChassis::setup() {
  Wire.begin(); // join i2c bus (address optional for master)
}

void RemoteChassis::move_rot(long rot) {
  ic2_long_t *l = (void*)&rot;
  Serial.print("Sending long: ");
  Serial.println(l->l);
  send_wire_event(CMD_MOVE_ROT, l->data, sizeof(ic2_long_t));
}

void RemoteChassis::set_pid(double kp, double ki, double kd) {
  ic2_pid_t pid;
  pid.ki = ki;
  pid.kp = kp;
  pid.kd = kd;
  send_wire_event(CMD_SET_PID, pid.data, sizeof(ic2_pid_t));
}

void RemoteChassis::send_wire_event(byte cmd, byte *data, int datasize) {
  Wire.beginTransmission(CHASSIS_DEVICE_ADDR); // transmit to device #8
  Wire.write(cmd);
  Wire.write(data, datasize);        // sends five bytes
  Wire.endTransmission();    // stop transmitting
  Serial.print("Transmitted ");
  Serial.print(datasize + 1);
  Serial.println(" bytes");
//  Serial.print(cmd);
//  Serial.print(" ");
//  for (int i = 0; i < datasize; ++i) {
//    Serial.print(data[i]);
//    Serial.print(" ");
//  }
//  Serial.println();
}

/*
 * MOTOR INIT
 */
void motor_init(motor_t* motor) {
  motor_t &m = *motor;
  photo_init(&m.photo1);
  photo_init(&m.photo2);
  m.dir = DIR_CW;
  m.lastms = millis();
  m.rps = 0;
  m.pwm = 0;
  m.pid = new PID(&(m.Input), &(m.pwm), &(m.Setpoint), DEF_KP, DEF_KI, DEF_KD, P_ON_M, DIRECT);
  //tell the PID to range between 0 and the full speed
  m.pid->SetOutputLimits(0, 255);
  m.pid->SetSampleTime(ADJUST_NAVIGATION_TIMER);
  m.pid->SetMode(MANUAL);
}

/*
 * MOTOR MOVE INIT
 */
void motor_move_init(motor_t* motor, long rot) {
  motor_t &m = *motor;
  m.Input = m.photo1.count;
  m.Setpoint = m.Input + rot;
//  Serial.print("Init Move Input: ");
//  Serial.print(m.Input);
//  Serial.print(", Output: ");
//  Serial.print(m.pwm);
//  Serial.print(", Setpoint: ");
//  Serial.println(m.Setpoint);
  //turn the PID on
  m.pid->SetMode(AUTOMATIC);
}

/*
 * MOTOR ADJUST NAVIGATION
 */
 bool motor_adjust_navigation(motor_t* motor) {
  motor_t &m = *motor;
    // Still active?
  if (m.pid->GetMode() == AUTOMATIC) {
    m.Input = m.photo1.count;
    // Stop control if Setpoint is reached
//    if (m.Input >= m.Setpoint) {
//      m.pid->SetMode(MANUAL);
//      m.pwm = 0;
//    }
//    else {
      m.pid->Compute();
//      // Repeat Stop control if Setpoint is reached
//      if (m.Input >= m.Setpoint) {
//        m.pid->SetMode(MANUAL);
//        m.pwm = 0;
//      }
//    }
    return true;
  }
  return false;
 }

/*
 * PHOTO INIT
 */
void photo_init(photo_t* photo) {
  photo_t &p = *photo;
  p.high = 0;
  p.low = 0;
  p.treshold = 0;
  p.state = PHOTO_OFF;
  p.count = 0;
  p.lastCount = 0;
}


/**
 * MOTOR STATS
 */
void motor_stats (motor_t* motor, long ms)
{
  motor_t &m = *motor;
  long elapsed = ms - m.lastms;
  long p1count = m.photo1.count;
  long p2count = m.photo2.count;
  float p1rps = ((float)(p1count - m.photo1.lastCount)) * 1000.0f / (float)elapsed;
  float p2rps = ((float)(p2count - m.photo2.lastCount)) * 1000.0f / (float)elapsed;
  m.rps = (p1rps + p2rps) / 2.0f;
  m.photo1.lastCount = p1count;
  m.photo2.lastCount = p2count;
  m.lastms = ms;
//  Serial.print("Motor Stats: ");
//  Serial.print(ms);
//  Serial.print(" ");
//  Serial.print(elapsed);
//  Serial.print(" ");
//  Serial.print(p1count);
//  Serial.print(" ");
//  Serial.print(p1count - m.photo1.lastCount);
//  Serial.print(" ");
//  Serial.print(p1rps);
//  Serial.print(" ");
//  Serial.println();
}

/*
 * READ Photo Cell and update stats
 */
void photo_update(photo_t* photo, int v) {
  photo_t &p = *photo;
  // Calibrate sensor's high and low
  if (v > p.high) {
    p.high = v;
    p.treshold = (p.high + p.low) / 2;
  }
  else if (v < p.low || p.low == 0) {
    p.low = v;
    p.treshold = (p.high + p.low) / 2;
  }
  // Light up sensor
  if (p.state == PHOTO_OFF && v > p.treshold) {
    p.state = PHOTO_ON;
    ++p.count;
  }
  // Sensor is off
  else if (p.state == PHOTO_ON && v < p.treshold) {
    p.state = PHOTO_OFF;
  }
}


/*
 * TELEMETRY
 */
// or use this loop if sending floats
void telemetry(float input, float output, float setpoint, float output2, float ki, float kp, float kd) {
  char input_text[30];
  char output_text[30];
  char setpoint_text[30];
  char output2_text[30];
  char ki_text[30];
  char kp_text[30];
  char kd_text[30];

  dtostrf(input, 10, 10, input_text);
  dtostrf(output, 10, 10, output_text);
  dtostrf(setpoint, 10, 10, setpoint_text);
  dtostrf(output2, 10, 10, output2_text);
  dtostrf(ki, 10, 10, ki_text);
  dtostrf(kp, 10, 10, kp_text);
  dtostrf(kd, 10, 10, kd_text);

  char text[217];
  snprintf(text, 217, "%s,%s,%s,%s,%s,%s,%s", input_text, output_text, setpoint_text, output2_text, ki_text, kp_text, kd_text);
  Serial.println(text);
}

/*
 * WIRE READ BYTES
 */
void wire_read_bytes(byte *data, byte size) {
  for (byte i = 0; i < size; ++i) {
    data[i] = Wire.read();
  }
}

