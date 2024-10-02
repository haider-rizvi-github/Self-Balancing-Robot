#include "pins.h"
#include "voltage.h"
#include "pid.h"
#include "tb6612fng.h"
#include "controller.h"

double data = 0;
TB6612FNG motor = TB6612FNG(STBY_PIN, AIN1, BIN1, PWMA_LEFT, PWMB_RIGHT);

void setup() {
  // put your setup code here, to run once:
  pinMode(LEFT_RECEIVE_PIN, INPUT_PULLUP);
  pinMode(RIGHT_RECEIVE_PIN, INPUT_PULLUP);

  voltage_init(VOL_MEASURE_PIN);

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

  // data = (voltage_read()* 1.1 / 1024) * ((10 + 1.5) / 1.5);
  // Serial.println(data);
  motor.moveCCW(50);
  delay(1000);
  motor.moveCW(50);
  delay(1000);
  motor.moveForw(50);
  delay(1000);
  motor.moveBack(50);
  delay(1000);
  motor.stop();
  delay(1000);
}
