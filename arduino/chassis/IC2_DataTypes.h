
#ifndef __IC2_DataTypes_h__
#define __IC2_DataTypes_h__

union ic2_long_t {
  long l;
  byte data[sizeof(long)];
};

struct pid_t {
  float Kp;
  float Ki;
  float Kd;
};

union ic2_pid_t {
  struct {
    float kp;
    float ki;
    float kd;
  };
  byte data[sizeof(float) * 3];
};

#endif

