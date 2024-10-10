#include "voltage.h"
#include "motion_controller.h"

// //Setting PID parameters
// double kp_balance = 55, kd_balance = 0.75;
// double kp_speed = 10, ki_speed = 0.26;
// double kp_turn = 2.5, kd_turn = 0.5;
// double data = 0;

// TB6612FNG motor = TB6612FNG(STBY_PIN, AIN1, BIN1, PWMA_LEFT, PWMB_RIGHT);
// // PIDController pid = PIDController(kp_balance, 0, kd_balance, -3000.0f, 3000.0f);
// PIDController pid = PIDController(kp_speed, ki_speed, 0, -3000.0f, 3000.0f);
// mpu6050_base mpu;

motion_controller motion;

void setup() {
  // put your setup code here, to run once:
  pinMode(LEFT_RECEIVE_PIN, INPUT_PULLUP);
  pinMode(RIGHT_RECEIVE_PIN, INPUT_PULLUP);
  voltage_init(VOL_MEASURE_PIN);
  Serial.begin(9600);
  Serial.print("Start:");
  Wire.begin();
  // mpu.init();
  motion.init();
  // Serial.print(motion.mpu.acc_x);

}

void loop() {
  // put your main code here, to run repeatedly:
  // data = (voltage_read()* 1.1 / 1024) * ((10 + 1.5) / 1.5);

  // Serial.println(data);
  // motor.moveCCW(50);
  // delay(1000);
  // motor.moveCW(50);
  // delay(1000);
  // motor.moveForw(50);
  // delay(1000);
  // motor.moveBack(50);
  // delay(1000);
  // motor.stop();
  // delay(1000);

  // mpu.calculate();
  // Serial.print(mpu.angle_pitch);
  // Serial.print(',');
  // // Serial.print(mpu.angle_roll);
  // // Serial.print(',');
  // Serial.println(mpu.angle_roll);

  // pid.Compute(0, mpu.angle_pitch);
  // Serial.print(mpu.angle_pitch);
  // Serial.print(',');
  // Serial.println(pid.Output);

  // if (pid.Output < 0) {motor.moveForw(pid.Output); } 
  // else {motor.moveBack(pid.Output);}

  motion.balance();
  // // motion.pid.Compute(0, motion.mpu.angle_roll);
  // Serial.print(motion.mpu.angle_roll);
  // Serial.print(',');
  Serial.print((float)motion.pwm_left);
  Serial.print(',');
  Serial.println((float)motion.pwm_right);


//// State Pattern (eventually)
    // getKeyValue();
    // getBluetoothData();
    // keyEventHandle();
    // getDistance();
    // voltageMeasure();
    // // Delegate behavior to the current states
    // robot.handleMotion();
    // robot.handleFunction();
    // // Other loop code
////
}
