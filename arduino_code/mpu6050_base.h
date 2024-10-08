
#pragma once

#include "HardwareSerial.h"
#include "Wire.h"
class mpu6050_base {
private:
  uint8_t mpu6050_addr;

  // Setup timers and temp variables
  long loop_timer;
  float temp;
  
  void setup_mpu_6050_registers();
  void read_mpu_6050_data();

public:
  mpu6050_base() : mpu6050_addr(0x68) {};
  mpu6050_base(uint8_t i2c_addr): mpu6050_addr(i2c_addr) {};
  void init();
  void calculate();
  void calculate_IMU_error(HardwareSerial &serialPort);
  // Variables for gyroscope
  float gyro_x = 0, gyro_y = 0, gyro_z = 0;           // To store gyro values
  long gyro_x_cal = 0, gyro_y_cal = 0, gyro_z_cal = 0;  // To store offset
  bool set_gyro_angles;

  // Variables for Accelerometer
  float acc_x = 0, acc_y = 0, acc_z = 0, acc_total_vector = 0;
  float angle_roll_acc = 0, angle_pitch_acc = 0;

  // Variables for angles
  float angle_pitch = 0, angle_roll = 0;
  float angle_pitch_buffer = 0, angle_roll_buffer = 0;
  float angle_pitch_output = 0, angle_roll_output = 0;
};


void mpu6050_base::init() {
 // Setup the registers of the MPU_6050
  setup_mpu_6050_registers();  // This will go manually and setup the sensor register

  // Read the raw acc and gyro data from the MPU_6050 1000 times
  for (int cal_int = 0; cal_int < 1000; cal_int++) {
    read_mpu_6050_data();
    // calibrating the gyro
    gyro_x_cal += gyro_x;   //add the gyro x offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;   //add the gyro y offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;   //add the gyro z offset to the gyro_z_cal variable
    delay(3);  //setting delay 3us to have 250Hz for-loop
  };

  // Getting average offset by dividing results with 1000
  gyro_x_cal = gyro_x_cal / 1000;
  gyro_y_cal = gyro_y_cal / 1000;
  gyro_z_cal = gyro_z_cal / 1000;

  // // init Timer
  // loop_timer = micros();  // To calculate microseconds
}

void mpu6050_base::calculate(){
  // put your main code here, to run repeatedly:

  //Get data from MPU-6050
  read_mpu_6050_data();

  //Subtract the offset values from the raw gyro values
  gyro_x -= gyro_x_cal;  // gyro_x_cal is the offset got by averaging 1000 values
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;

  // gyro angle calculations. NOTE: 0.0000611 = 1 / (250 Hz x 65.5)

  // Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_pitch += gyro_x * 0.0000611;

  // Calculate the traveled roll and add this to the angle_roll variable
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_roll += gyro_y * 0.000611;

  //If the IMU has yawed transfer the roll angle to the pitch angle
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);
  //If the IMU has yawed transfer the pitch angle to the roll angle
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);

  // Accelerometer angle calculations
  // Calculate the total accelerometer vector
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));

  // 57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  // Calculate the pitch angle
  angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;  // pi/180 = 57.296

  // Calculate the roll angle
  angle_roll_acc = asin((float)acc_x / acc_total_vector) * -57.296;

  // Accelerometer calibration value for pitch
  angle_pitch_acc -= 0.0;
  // Accelerometer calibration value for roll
  angle_roll_acc -= 0.0;

  if (set_gyro_angles) {

    // if the IMU has been running
    // Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_pitch = angle_pitch*0.9996 + angle_pitch_acc*0.0004;
    // Correct the drift of the gyro roll angle with the accelerometer roll angle
    angle_roll = angle_roll*0.9996 + angle_roll_acc*0.0004; 
  }else{
    // IMU has just started
    // Set the gyro pitch angle equal to the accelerometer pitch angle
    angle_pitch = angle_pitch_acc;
    // Set the gyro roll angle equal to the accelerometer roll angle
    angle_roll = angle_roll_acc;
    // Set the IMU started flag 
    set_gyro_angles = true;
  };

  // To dampen the pitch and roll angles a complementary filter is used
  // Take 90% of the output pitch value and add 10% of the raw pitch 
  angle_pitch_output = angle_pitch_output*0.9 + angle_pitch*0.1;
  // Take 90% of the output roll value and add 10% of the raw roll
  angle_roll_output = angle_roll_output*0.9 + angle_roll*0.1;
  //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop  

  while(micros() - loop_timer < 4000);
  // Reset the loop timer
  loop_timer = micros();
}

void mpu6050_base::setup_mpu_6050_registers() {
  // Start communicating with MPU6050
  Wire.beginTransmission(mpu6050_addr);
  Wire.write(0x6B);  // Talk to the Power Management register 0x6B
  Wire.write(0x01);  // setup to wake up the register
  Wire.endTransmission(); //end the transmission

  // Configuring the accelerometer (+/- 8g)
  Wire.beginTransmission(mpu6050_addr);
  Wire.write(0x1C);  // talk to ACCEL_CONFIG register: 1C
  Wire.write(0x00);  //  (+/- 2g full scale range)
  Wire.endTransmission();

  // Configuring gyroscope (500dps full scale)
  Wire.beginTransmission(mpu6050_addr);
  Wire.write(0x1B);  // talk to GYRO_CONFIG register: 1B
  Wire.write(0x00);  //  (250deg/s full scale)
  Wire.endTransmission();
  delay(20);
};

void mpu6050_base::read_mpu_6050_data() {
  // Read the raw gyro and accelerometer data
  // Start communicating with the MPU-6050
  Wire.beginTransmission(mpu6050_addr);
  Wire.write(0x3B);
  Wire.endTransmission();
  // Request 14 bytes from the MPU 6050  to get data from ACCEL_XOUT_H -> GYRO_ZOUT_L (check register sheet of MPU)
  Wire.requestFrom(mpu6050_addr, (uint8_t)14);

  // Wait until all the bytes are recieved
  if (Wire.available() >= 14) {
    // using condition to store the data
    acc_x = Wire.read() << 8 | Wire.read();  // first condition shifts the high bit to 8 digits and second condition places the LOW bit to complete data
    acc_y = Wire.read() << 8 | Wire.read();
    acc_z = Wire.read() << 8 | Wire.read();
    temp = Wire.read() << 8 | Wire.read();
    gyro_x = Wire.read() << 8 | Wire.read();
    gyro_y = Wire.read() << 8 | Wire.read();
    gyro_z = Wire.read() << 8 | Wire.read();
  };
};

void mpu6050_base::calculate_IMU_error(HardwareSerial &serialPort) {
  float AccErrorX = 0 ; 
  float AccErrorY = 0 ; 
  float AccErrorZ = 0 ; 
  float GyroErrorX = 0 ; 
  float GyroErrorY = 0 ; 
  float GyroErrorZ = 0 ; 
  uint16_t c = 0;
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(mpu6050_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(mpu6050_addr, (uint16_t)6, (uint16_t)true);
    acc_x = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    acc_y = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    acc_z = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((acc_y) / sqrt(pow((acc_x), 2) + pow((acc_z), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (acc_x) / sqrt(pow((acc_y), 2) + pow((acc_z), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(mpu6050_addr);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(mpu6050_addr, (uint16_t)6, (uint16_t)true);
    gyro_x = Wire.read() << 8 | Wire.read();
    gyro_y = Wire.read() << 8 | Wire.read();
    gyro_z = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (gyro_x / 131.0);
    GyroErrorY = GyroErrorY + (gyro_y / 131.0);
    GyroErrorZ = GyroErrorZ + (gyro_z / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  // Print the error values on the Serial Monitor
  serialPort.print("AccErrorX: ");
  serialPort.println(AccErrorX);
  serialPort.print("AccErrorY: ");
  serialPort.println(AccErrorY);
  serialPort.print("GyroErrorX: ");
  serialPort.println(GyroErrorX);
  serialPort.print("GyroErrorY: ");
  serialPort.println(GyroErrorY);
  serialPort.print("GyroErrorZ: ");
  serialPort.println(GyroErrorZ);
}