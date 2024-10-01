#include "voltage.h"

#define VOL_MEASURE_PIN A2
#define ECHO_PIN A3
#define TRIG_PIN 11
#define RECV_PIN 9
#define NUMPIXELS 4
#define RGB_PIN 3
#define AIN1 7
#define PWMA_LEFT 5
#define BIN1 12
#define PWMB_RIGHT 6
#define STBY_PIN 8
#define ENCODER_LEFT_A_PIN 2
#define ENCODER_RIGHT_A_PIN 4
#define IR_SEND_PIN 9
#define LEFT_RECEIVE_PIN A0   // IR_left
#define RIGHT_RECEIVE_PIN A1  // IR_right
#define KEY_MODE 10

double data = 0;


void setup() {
  // put your setup code here, to run once:
  pinMode(LEFT_RECEIVE_PIN, INPUT_PULLUP);
  pinMode(RIGHT_RECEIVE_PIN, INPUT_PULLUP);
  pinMode(VOL_MEASURE_PIN, INPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

  data = (analogRead(VOL_MEASURE_PIN)* 1.1 / 1024) * ((10 + 1.5) / 1.5);
  Serial.println(data);
}
