#include "PID.h"

PID::PID() {
  p_error = i_error = d_error = 0.0;
  Kd = Ki = Kp = 0.0;
  totalError = 0.0;
  counter = 0;
}

PID::~PID() {}

void PID::Init(const double Kp, const double Ki, const double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  totalError = 0.0;
  counter = 0;
}

std::array<double, 3> PID::GetCoeffs() const {
  return std::array<double, 3>({Kp, Ki, Kd});
}

void PID::UpdateError(const double cte) {
  if(0 == counter) {
    p_error = cte;
  }

  d_error = cte - p_error;
  i_error += cte;
  p_error = cte;
  ++counter;
  totalError += cte*cte;
}

double PID::TotalError() const {
  return totalError/counter;
}

size_t PID::GetCounter() const {
  return counter;
}

double PID::Compute() const {
  return -Kp * p_error - Ki * i_error - Kd * d_error;
}
