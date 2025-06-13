#include "DiscretePID.h"

DiscretePID::DiscretePID(float Kp_, float Ki_, float Kd_, float dt_, PIDMode mode_, int N_)
  : Kp(Kp_), Ki(Ki_), Kd(Kd_), dt(dt_), mode(mode_), N(N_), output(0), integral(0), previous_error(0),
    d0(0), d1(0), fd0(0), fd1(0)
{
  reset();

  // Precomputar coeficientes para IIR
  A0 = Kp + Ki * dt + Kd / dt;
  A1 = -Kp - 2 * Kd / dt;
  A2 = Kd / dt;

  // Coeficientes para el filtro pasa baja derivativo
  float tau = Kd / (Kp * N);
  float alpha = dt / (2 * tau);
  alpha_1 = alpha / (alpha + 1);
  alpha_2 = (alpha - 1) / (alpha + 1);
}

void DiscretePID::reset() {
  for (int i = 0; i < 3; ++i) error[i] = 0;
  output = 0;
  integral = 0;
  previous_error = 0;
  d0 = d1 = fd0 = fd1 = 0;
}

float DiscretePID::compute(float setpoint, float measured_value) {
  // Desplazar errores
  error[2] = error[1];
  error[1] = error[0];
  error[0] = setpoint - measured_value;

  switch (mode) {
    case BASIC:
      integral += error[0] * dt;
      float derivative = (error[0] - previous_error) / dt;
      output = Kp * error[0] + Ki * integral + Kd * derivative;
      previous_error = error[0];
      break;

    case IIR:
      output = A0 * error[0] + A1 * error[1] + A2 * error[2];
      break;

    case DERIVATIVE_FILTERED:
      // PI
      output += Kp * (error[0] - error[1]) + Ki * dt * error[0];

      // Derivada filtrada
      d1 = d0;
      d0 = Kd / dt * (error[0] - 2 * error[1] + error[2]);
      fd1 = fd0;
      fd0 = alpha_1 * (d0 + d1) - alpha_2 * fd1;

      output += fd0;
      break;
  }

  return output;
}
