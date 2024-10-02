#include "Arduino.h"
// STBY: Standby	Input	Allows the H-bridges to work when high (has a pulldown resistor so it must actively pulled high)
// AIN1/BIN1:	Input 1 for channels A/B	Input	One of the two inputs that determines the direction.
// AIN2/BIN2:	Input 2 for channels A/B	Input	One of the two inputs that determines the direction.
// PWMA/PWM:	PWM input for channels A/B	Input	PWM input that controls the speed
// A01/B01:	Output 1 for channels A/B	Output	One of the two outputs to connect the motor
// A02/B02:	Output 2 for channels A/B	Output	One of the two outputs to connect the motor

enum TB6612FNGMode {
    SINGLE_DIRECTIONAL = 1,
    INVERTED_PINS_BI_DIRECTIONAL = 2,
    FOUR_PINS_BI_DIRECTIONAL = 4
};

class TB6612FNG {
private:
  TB6612FNGMode mode;
  uint16_t stbyPin, pwmaPin, pwmbPin;
  uint16_t ain1Pin, ain2Pin, bin1Pin, bin2Pin;

public:
  // Constructors
  TB6612FNG();
  TB6612FNG(uint16_t stby, uint16_t ain1, uint16_t ain2, uint16_t pwma);
  TB6612FNG(uint16_t stby, uint16_t ain1, uint16_t bin1, uint16_t pwma, uint16_t pwmb);
  TB6612FNG(uint16_t stby, uint16_t ain1, uint16_t ain2, uint16_t bin1, uint16_t bin2, uint16_t pwma, uint16_t pwmb);

  TB6612FNGMode getMode();
  // Motor control functions
  void standBy();
  void shortBrake();
  void moveCCW(unsigned char);  // Counter Clockwise
  void moveCW(unsigned char);  // Clockwise
  void moveForw(unsigned char);  // Clockwise
  void moveBack(unsigned char);  // Clockwise
  void stop();
};

// Constructor for single-direction control
TB6612FNG::TB6612FNG(uint16_t stby, uint16_t ain1, uint16_t ain2, uint16_t pwma) {
  stbyPin = stby;
  ain1Pin = ain1;
  ain2Pin = ain2;
  pwmaPin = pwma;
  mode = SINGLE_DIRECTIONAL;  
  
  pinMode(stbyPin, OUTPUT);
  pinMode(ain1Pin, OUTPUT);
  pinMode(ain2Pin, OUTPUT);
  pinMode(pwmaPin, OUTPUT);
  pinMode(pwmbPin, OUTPUT);
  
  standBy();
}

// Constructor for full H-bridge control (2pin-bidirectional)
TB6612FNG::TB6612FNG(uint16_t stby, uint16_t ain1, uint16_t bin1, uint16_t pwma, uint16_t pwmb) {
  stbyPin = stby;
  ain1Pin = ain1;
  bin1Pin = bin1;
  pwmaPin = pwma;
  pwmbPin = pwmb;
  mode = INVERTED_PINS_BI_DIRECTIONAL;  

  pinMode(stbyPin, OUTPUT);
  pinMode(ain1Pin, OUTPUT);
  pinMode(bin1Pin, OUTPUT);
  pinMode(pwmaPin, OUTPUT);
  pinMode(pwmbPin, OUTPUT);

  standBy();
}

// Constructor for full H-bridge control (4pin-bidirectional)
TB6612FNG::TB6612FNG(uint16_t stby, uint16_t ain1, uint16_t ain2, uint16_t bin1, uint16_t bin2, uint16_t pwma, uint16_t pwmb) {
  stbyPin = stby;
  ain1Pin = ain1;
  ain2Pin = ain2;
  bin1Pin = bin1;
  bin2Pin = bin2;
  pwmaPin = pwma;
  pwmbPin = pwmb;
  mode = FOUR_PINS_BI_DIRECTIONAL;  

  pinMode(stbyPin, OUTPUT);
  pinMode(ain1Pin, OUTPUT);
  pinMode(ain2Pin, OUTPUT);
  pinMode(bin1Pin, OUTPUT);
  pinMode(bin2Pin, OUTPUT);
  pinMode(pwmaPin, OUTPUT);
  pinMode(pwmbPin, OUTPUT);

  standBy();
}

TB6612FNGMode TB6612FNG::getMode() {
  return mode;
}

// Puts the motor driver into standby mode
void TB6612FNG::standBy() {
  digitalWrite(stbyPin, LOW); // Enable standby
}

// Short brake: quickly stops the motors
void TB6612FNG::shortBrake() {
  digitalWrite(stbyPin, HIGH);  
  digitalWrite(ain1Pin, HIGH);
  digitalWrite(ain2Pin, HIGH);
  digitalWrite(bin1Pin, HIGH);
  digitalWrite(bin2Pin, HIGH);
}

// Moves both motors in the counter-clockwise direction
void TB6612FNG::moveCCW(unsigned char speed) {
  if (mode == SINGLE_DIRECTIONAL) {
    digitalWrite(ain1Pin, LOW);
    digitalWrite(ain2Pin, HIGH);
    digitalWrite(stbyPin, HIGH); 
    analogWrite(PWMA_LEFT, speed);

  } else if (mode == INVERTED_PINS_BI_DIRECTIONAL) {
    digitalWrite(ain1Pin, LOW);
    digitalWrite(bin1Pin, HIGH);
    analogWrite(PWMA_LEFT, speed);
    analogWrite(PWMB_RIGHT, speed);
    digitalWrite(stbyPin, HIGH);
    
  } else if (mode == FOUR_PINS_BI_DIRECTIONAL) {
    digitalWrite(ain1Pin, LOW);
    digitalWrite(ain2Pin, HIGH);
    digitalWrite(bin1Pin, HIGH);
    digitalWrite(bin2Pin, LOW);
    analogWrite(PWMA_LEFT, speed);
    analogWrite(PWMB_RIGHT, speed);
    digitalWrite(stbyPin, HIGH);
  }  
}

// Moves both motors in the clockwise direction
void TB6612FNG::moveCW(unsigned char speed) {
  if (mode == SINGLE_DIRECTIONAL) {
    digitalWrite(ain1Pin, HIGH);
    digitalWrite(ain2Pin, LOW);
    digitalWrite(stbyPin, HIGH); 
    analogWrite(PWMA_LEFT, speed);

  } else if (mode == INVERTED_PINS_BI_DIRECTIONAL) {
    digitalWrite(ain1Pin, HIGH);
    digitalWrite(bin1Pin, LOW);
    analogWrite(PWMA_LEFT, speed);
    analogWrite(PWMB_RIGHT, speed);
    digitalWrite(stbyPin, HIGH);
    
  } else if (mode == FOUR_PINS_BI_DIRECTIONAL) {
    digitalWrite(ain1Pin, HIGH);
    digitalWrite(ain2Pin, LOW);
    digitalWrite(bin1Pin, LOW);
    digitalWrite(bin2Pin, HIGH);
    analogWrite(PWMA_LEFT, speed);
    analogWrite(PWMB_RIGHT, speed);
    digitalWrite(stbyPin, HIGH);
  }  
}

void TB6612FNG::moveForw(unsigned char speed) {
  if (mode == SINGLE_DIRECTIONAL) {
    digitalWrite(ain1Pin, LOW);
    digitalWrite(ain2Pin, HIGH);
    digitalWrite(stbyPin, HIGH); 
    analogWrite(PWMA_LEFT, speed);

  } else if (mode == INVERTED_PINS_BI_DIRECTIONAL) {
    digitalWrite(ain1Pin, LOW);
    digitalWrite(bin1Pin, LOW);
    analogWrite(PWMA_LEFT, speed);
    analogWrite(PWMB_RIGHT, speed);
    digitalWrite(stbyPin, HIGH);
    
  } else if (mode == FOUR_PINS_BI_DIRECTIONAL) {
    digitalWrite(ain1Pin, LOW);
    digitalWrite(ain2Pin, HIGH);
    digitalWrite(bin1Pin, LOW);
    digitalWrite(bin2Pin, HIGH);
    analogWrite(PWMA_LEFT, speed);
    analogWrite(PWMB_RIGHT, speed);
    digitalWrite(stbyPin, HIGH);
  }  
}

void TB6612FNG::moveBack(unsigned char speed) {
  if (mode == SINGLE_DIRECTIONAL) {
    digitalWrite(ain1Pin, HIGH);
    digitalWrite(ain2Pin, LOW);
    digitalWrite(stbyPin, HIGH); 
    analogWrite(PWMA_LEFT, speed);

  } else if (mode == INVERTED_PINS_BI_DIRECTIONAL) {
    digitalWrite(ain1Pin, HIGH);
    digitalWrite(bin1Pin, HIGH);
    analogWrite(PWMA_LEFT, speed);
    analogWrite(PWMB_RIGHT, speed);
    digitalWrite(stbyPin, HIGH);
    
  } else if (mode == FOUR_PINS_BI_DIRECTIONAL) {
    digitalWrite(ain1Pin, HIGH);
    digitalWrite(ain2Pin, LOW);
    digitalWrite(bin1Pin, HIGH);
    digitalWrite(bin2Pin, LOW);
    analogWrite(PWMA_LEFT, speed);
    analogWrite(PWMB_RIGHT, speed);
    digitalWrite(stbyPin, HIGH);
  }  
}

// Stops the motors (open circuit)
void TB6612FNG::stop() {
  digitalWrite(stbyPin, LOW);  // Activate standby mode, stopping the motors
}