#ifndef DISCRETE_PID_H
#define DISCRETE_PID_H

#include <Arduino.h>

enum PIDMode {
  BASIC,
  IIR,
  DERIVATIVE_FILTERED
};

class DiscretePID {
public:
  DiscretePID(float Kp, float Ki, float Kd, float dt, PIDMode mode = BASIC, int N = 5);

  void reset();
  float compute(float setpoint, float measured_value);

private:
  float Kp, Ki, Kd, dt;
  PIDMode mode;
  int N;

  // Common variables
  float error[3];
  float output;
  
  // BASIC PID
  float integral;
  float previous_error;

  // IIR
  float A0, A1, A2;

  // Filtered derivative
  float alpha_1, alpha_2;
  float d0, d1, fd0, fd1;
};

#endif
