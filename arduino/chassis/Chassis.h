
#ifndef __Chassis_h__
#define __Chassis_h__

#include <Arduino.h>
#include <PID_v1.h>

// 2 bytes. Argument 1 byte - speed (0-255). Example: 1000 = Stop
#define CMD_SET_SPEED 10
// 5 bytes. Argument 1 long - number of rotations.
#define CMD_MOVE_ROT  11

// 1 + 3*4 bytes. Arguments: Kp - float, Ki - float, Kd - float.
#define CMD_SET_PID   12
// Photo Sensor State
#define PHOTO_ON 1
#define PHOTO_OFF 0

// DIRECTION Clockwise and Counter Clock Wise
#define DIR_CW  LOW
#define DIR_CCW  HIGH

// PID Default settings, most likely this is the best place to start tune in if something goes wrong
#define DEF_KP 0.3
#define DEF_KI 0.3
#define DEF_KD 0.3

// Motor pins
#define M1DIR 7
#define M1SPEED 6
#define M2DIR  4
#define M2SPEED 5


#define ADJUST_NAVIGATION_TIMER 100

#define CHASSIS_DEVICE_ADDR 8

/*
 * PHOTO Sensor Structure
 */
struct photo_t {
  int high;
  int low;
  int treshold;
  volatile int state;
  volatile long count;
  long lastCount;
};

/*
 * MOTOR Structure - dc engine and 2 photo sensors
 */
struct motor_t {
  photo_t photo1;
  photo_t photo2;
  int dir;
  long lastms;
  float rps;
  double pwm;
  double Input;
  double Setpoint;
  PID *pid;
};

class Chassis
{
  public:
    Chassis();
    void setup();
  	void recieve_wire_event();
  	// void turn(byte deg);
  	void set_speed(byte speed);
  	// void direction(byte dir);
  	// void move(int mm)
    void move_rot(long rot);
  	// void start_calibration();
  	// void stop_calibration(int leftmm, int rightmm);
    void update_motors_stats(long ms);
    void update_photo_state(byte v11, byte v12, byte v21, byte v22);
    void adjust_navigation();
    void set_pid(double kp, double ki, double kd);
  private:
  	byte dir;
    motor_t motor1;
    motor_t motor2;
    float differential = 1.0f;
};

class RemoteChassis
{
  public:
    RemoteChassis();
    void setup();
//    void set_speed(byte speed);
    void move_rot(long rot);
    void set_pid(double kp, double ki, double kd);
  private:
  void send_wire_event(byte cmd, byte *data, int len);
};

void motor_init(motor_t* motor);
void photo_init(photo_t* photo);
void motor_stats (motor_t* motor, long ms);
void photo_update(photo_t* photo, int v);
void receiveWireEvent(int num);
void motor_move_init(motor_t* motor, long rot);
bool motor_adjust_navigation(motor_t* motor);
void telemetry(float input, float output, float setpoint, float output2, float ki, float kp, float kd);
void wire_read_bytes(byte *data, byte size);

#endif
