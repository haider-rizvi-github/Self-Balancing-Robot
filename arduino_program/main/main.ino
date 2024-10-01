

#define IR_left A0
#define IR_right A1
#define Battery_Voltage_Level A2
#define US_ECHO A3
#define SDA_PIN A4
#define SCL_PIN A5


int data = 0;


void setup() {
  // put your setup code here, to run once:
  pinMode(IR_left, INPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  data = analogRead(IR_left);
  Serial.println(data);
}
