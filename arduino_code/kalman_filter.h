

class KalmanFilter {
 private:
  float dt, Q_angle, Q_gyro, R_angle, C_0;
  float q_bias, angle_err;
  float Pdot[4] = {0,0,0,0};
  float P[2][2] = {{1, 0}, {0, 1}};
  float K_0 = 0;
  float K_1 = 0;
  float angle_dot;

 public:
  double angle;
  KalmanFilter(float dt, float Q_angle, float Q_gyro, float R_angle, float C_0);
  double getAngle(double measured_angle, double measured_gyro);
};

KalmanFilter::KalmanFilter(float dt, float Q_angle, float Q_gyro, float R_angle, float C_0) : 
dt(dt), Q_angle(Q_angle), Q_gyro(Q_gyro), R_angle(R_angle), C_0(C_0) {};

double KalmanFilter::getAngle( double measured_angle, double measured_gyro) {
  
  angle += ( measured_gyro - q_bias ) * dt; // Estimated states
  angle_err  = measured_angle - angle;      // Error Calculation

  // Time Update
  Pdot[0] = Q_angle - P[0][1] - P[1][0];
  Pdot[1] = - P[1][1];
  Pdot[2] = - P[1][1];
  Pdot[3] = Q_gyro;

  P[0][0] += Pdot[0] * dt;
  P[0][1] += Pdot[1] * dt;
  P[1][0] += Pdot[2] * dt;
  P[1][1] += Pdot[3] * dt;

  // 
  double C0_P00 = C_0 * P[0][0];
  double C0_P01 = C_0 * P[0][1];
  double E = R_angle + C_0 * C0_P00; 

  // Kalman gain
  K_0 = C_0 * P[0][0] / E;
  K_1 = C_0 * P[1][0] / E;

  // Measurement Update
  P[0][0] -= K_0 * C0_P00;
  P[0][1] -= K_0 * C0_P01;
  P[1][0] -= K_1 * C0_P00;
  P[1][1] -= K_1 * C0_P01;

  // Correct the estimate with the Kalman gain
  angle += K_0 * angle_err;
  q_bias += K_1 * angle_err;
  angle_dot = measured_angle - q_bias;
  return angle;
}

