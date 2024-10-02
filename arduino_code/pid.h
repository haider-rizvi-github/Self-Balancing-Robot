#pragma once

#include "Arduino.h"

class PIDController {
  private:
    float integrator;
    float prevError;
    float differentiator;
    float prevMeasurement;
    float out;

    // PID coefficients
    float Kp, Ki, Kd;

    // Sample time and filter coefficient
    float T, tau;

    // Output limits
    float limMin, limMax;

  public:
    // Constructor to initialize PID parameters
    PIDController(float _Kp, float _Ki, float _Kd, float _T, float _tau, float _limMin, float _limMax)
        : Kp(_Kp), Ki(_Ki), Kd(_Kd), T(_T), tau(_tau), limMin(_limMin), limMax(_limMax),
          integrator(0.0f), prevError(0.0f), differentiator(0.0f), prevMeasurement(0.0f), out(0.0f) {}

    // Method to initialize/reset the controller
    void Init();

    // Method to update the controller with new setpoint and measurement
    float Update(float setpoint, float measurement);
};
