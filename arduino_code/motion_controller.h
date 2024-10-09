#pragma once

#include "pins.h"
#include "pid.h"
#include "tb6612fng.h"
#include "mpu6050_base.h"
#include "HardwareSerial.h"

class motion_controller {
  private:
    //Setting PID parameters
    double kp_balance = 55, kd_balance = 0.75;
    double kp_speed = 10, ki_speed = 0.26;
    double kp_turn = 2.5, kd_turn = 0.5;
    double data = 0;
  public:
    TB6612FNG motor;
    PIDController pid;
    mpu6050_base mpu;

    motion_controller() : motor(TB6612FNG(STBY_PIN, AIN1, BIN1, PWMA_LEFT, PWMB_RIGHT)),
      pid(PIDController(kp_balance, 0, kd_balance, -3000.0f, 3000.0f)),
      mpu(mpu6050_base()){};
      
    void init();
    void balance();
    void moveForward(float speed);
    void turnLeft(float rotation);
    void turnRight(float rotation);
    void moveBack(float speed);
};

void motion_controller::init(){
    mpu.init();
}

void motion_controller::balance() {
  mpu.calculate();
  pid.Compute(0.0f, mpu.angle_pitch);
  (pid.Output < 0) ? motor.moveForw(pid.Output) : motor.moveBack(pid.Output);

}