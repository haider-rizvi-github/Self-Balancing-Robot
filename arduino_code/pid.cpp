#include "pid.h"

void PIDController::Init() {
    integrator = 0.0f;
    prevError = 0.0f;
    differentiator = 0.0f;
    prevMeasurement = 0.0f;
    out = 0.0f;
}

// Method to update the controller with new setpoint and measurement
float PIDController::Update(float setpoint, float measurement) {
    // Error signal
    float error = setpoint - measurement;

    // Proportional term
    float proportional = Kp * error;

    // Integral term with trapezoidal rule
    integrator += 0.5f * Ki * (error + prevError) * T;

    // Anti-windup via dynamic integrator clamping
    float limMaxInt = (limMax > proportional) ? limMax - proportional : 0.0f;
    float limMinInt = (limMin < proportional) ? limMin - proportional : 0.0f;

    // Clamp Integrator
      if (integrator > limMaxInt) {
          integrator = limMaxInt;
      } else if (integrator < limMinInt) {
          integrator = limMinInt;
      }
    // Derivative term with low-pass filter (band-limited differentiator)
    differentiator = (2.0f * Kd * (measurement - prevMeasurement) 
                      + (2.0f * tau - T) * differentiator) 
                      / (2.0f * tau + T);

    // Compute the output
    out = proportional + integrator + differentiator;

    // Limit the output
    if (out > limMax) {
        out = limMax;
    } else if (out < limMin) {
        out = limMin;
    }        

    // Store current error and measurement for the next update
    prevError = error;
    prevMeasurement = measurement;

    return out;
}