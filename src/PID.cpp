#include "PID.h"

#include <math.h>

#include <iostream>

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  K[0] = Kp_;
  K[1] = Ki_;
  K[2] = Kd_;

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;

  D[0] = 1;
  D[1] = 1;
  D[2] = 0.001;

  tunedParamIndex = 0;
  maxCounter = 0;
  counter = 0;
  latLimit = 2.5;
}

void PID::UpdateError(double cte) {
  i_error += cte;
  d_error = cte - p_error;
  p_error = cte;
}

double PID::TotalError() {
  double result = -K[0] * p_error - K[1] * d_error - K[2] * i_error;
  counter++;
  return result;
}

void PID::Twiddle() {
  // check if vehicle was able to travel longer than before on the track.
  bool isThisRoundBetter = false;
  if (maxCounter < counter) {
    isThisRoundBetter = true;
  }

  // twiddle the PID parameter
  if (isThisRoundBetter) {
    maxCounter = counter;
    D[tunedParamIndex] *= 1.1;
  } else {
    K[tunedParamIndex] -= D[tunedParamIndex];
    D[tunedParamIndex] *= 0.9;
  }
  tunedParamIndex += 1;
  tunedParamIndex = tunedParamIndex % 3;
  K[tunedParamIndex] += D[tunedParamIndex];

  // reset
  counter = 0;
}
