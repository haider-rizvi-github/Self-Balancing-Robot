#pragma once

#include <stdint.h>
#include "EnableInterrupt.h"

#include "pins.h"
#include "pid.h"
#include "tb6612fng.h"
#include "mpu6050_base.h"
#include "HardwareSerial.h"

// Volatile variables that track the pulse counts from the encoders.
volatile int long encoder_count_left_a = 0; 
volatile int long encoder_count_right_a = 0;
/* Note: 
    'volatile' tells the compiler that the value of encoder_count_right_a can change at any time, outside of the normal program flow. 
    This is commonly used for variables that are modified within an interrupt service routine (ISR) or by another thread. */
void encoderCounterLeftA();
void encoderCounterRightA();
double pwm_left = 0;
double pwm_right = 0;

class motion_controller {
  private:
    //Setting PID parameters
    double kp_balance = 55, kd_balance = 0.75; //  PID parameters for balance control.
    double kp_speed = 10, ki_speed = 0.26; // PID parameters for controlling speed.
    double kp_turn = 2.5, kd_turn = 0.5; // PID parameters for controlling the robot's turning.
    double data = 0;

    int16_t encoder_left_pulse_num_speed = 0;
    int16_t encoder_right_pulse_num_speed = 0;


  public:
    TB6612FNG motor;
    PIDController pid;
    mpu6050_base mpu;

    motion_controller() : motor(TB6612FNG(STBY_PIN, AIN1, BIN1, PWMA_LEFT, PWMB_RIGHT)),
      pid(PIDController(kp_balance, 0, kd_balance, -3000.0f, 3000.0f)),
      mpu(mpu6050_base()){
      };
      
    void init();
    // run balance every 5ms
    //MsTimer2::set(5, balance);
    void balance();
    // void balance(float speed, float turn);

    void moveForward(float speed);
    void turnLeft(float rotation);
    void turnRight(float rotation);
    void moveBack(float speed);
};

void motion_controller::init(){
  motor.init();
  mpu.init();
  enableInterrupt(ENCODER_LEFT_A_PIN|PINCHANGEINTERRUPT, encoderCounterLeftA, CHANGE);
  enableInterrupt(ENCODER_RIGHT_A_PIN, encoderCounterRightA, CHANGE);
  //MsTimer2::set(5, balance);
}

void motion_controller::balance() {
  sei(); // set enable interrupts

  mpu.calculate();
  pid.Compute(0.0f, mpu.angle_pitch);
  pwm_left = pid.Output;
  pwm_right = pid.Output;

  pwm_left = constrain(pwm_left, -255, 255);
  pwm_right = constrain(pwm_right, -255, 255);
  (pwm_left < 0) ? motor.motorA(-pwm_left, 1) : motor.motorA(pwm_left, 0);
  (pwm_right < 0) ? motor.motorB(-pwm_right, 1) : motor.motorB(pwm_right, 0);
}

void encoderCounterLeftA() {
  encoder_count_left_a += (pwm_left > 0) - (pwm_left < 0); 
  // This approach eliminates branching, making it faster for embedded systems
}

void encoderCounterRightA() {
  encoder_count_right_a++;
}

