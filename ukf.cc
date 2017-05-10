#include "ukf.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace utility;


Ukf::Ukf() {

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a = 0.6;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd = 0.9;

  is_initialized = false;
  previous_timestamp = 0;

  n_x = 5;
  n_aug = 7;
  lambda_ = 3 - n_aug;

  total_sig_points = 2 * n_aug + 1;

  double weight_remaining = 0.5/(lambda_+n_aug);
  weights = VectorXd::Zero(total_sig_points);
  weights =  weights.array() + weight_remaining;
  weights(0) = lambda_/(lambda_+n_aug);

  // initialize x, P
  x = VectorXd(n_x);
  P = MatrixXd::Identity(n_x, n_x);

  // augmented x, P
  x_aug = VectorXd::Zero(n_aug);
  P_aug = MatrixXd::Zero(n_aug, n_aug);

  x_aug.head(n_x) = x;

  P_aug.topLeftCorner(n_x,n_x) = P;
  P_aug(5,5) = std_a*std_a;
  P_aug(6,6) = std_yawdd*std_yawdd;

  //create sigma point matrices
  Xsig_aug = MatrixXd(n_aug, total_sig_points);
  Xsig_pred = MatrixXd(n_x, total_sig_points);

  n_z_radar = 3;
  n_z_lidar = 2;

  // Laser measurement noise standard deviation position1 in m
  std_laspx = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy = 0.15;

  R_lidar_ = MatrixXd(n_z_lidar,n_z_lidar);
  R_lidar_ << std_laspx*std_laspx, 0,
              0, std_laspy*std_laspy;

  H_lidar = MatrixXd(n_z_lidar,n_x);
  H_lidar << 1,0,0,0,0,
             0,1,0,0,0;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  R_radar_ = MatrixXd(n_z_radar,n_z_radar);
  R_radar_ << std_radr_*std_radr_, 0, 0,
              0, std_radphi_*std_radphi_, 0,
              0, 0,std_radrd_*std_radrd_;

  nis = 0;
}


void Ukf::ProcessMeasurement(SensorReading& reading) {
  if(!is_initialized) {
    InitializeState(reading);
  }

  double dt = (reading.timestamp - previous_timestamp) / 1.0e6;
  previous_timestamp = reading.timestamp;

  //if the measurement comes back to back almost instantly,
  // then we don't really need to predict again.
  if ( dt > 1e-3 ) {
    Ukf::Prediction(dt);
  }

  switch (reading.sensor_type) {
    case SensorType::LASER:
      UpdateLidar(reading);
      break;
    case SensorType::RADAR:
      UpdateRadar(reading);
      break;
  }
}


void Ukf::InitializeState(SensorReading &reading) {
  if(reading.sensor_type == SensorType::RADAR) {
    double ro    = reading.measurement[0];
    double theta = reading.measurement[1];
    double ro_dot= reading.measurement[2];

    x << ro * cos(theta), ro * sin(theta), ro_dot, 0, 0;

  } else if (reading.sensor_type == SensorType::LASER) {
    x << reading.measurement[0], reading.measurement[1], 0, 0, 0;
  }

  //we can't let x and y be zero.
  if ( fabs(x(0)+x(1)) < 1e-4){
    x(0) = 1e-4;
    x(1) = 1e-4;
  }

  previous_timestamp = reading.timestamp;
  is_initialized = true;
}


void Ukf::Prediction(double dt) {
  GenerateSigmaPoints();
  PredictSigmaPoints(dt);
  PredictMeanCovariance();
}


void Ukf::GenerateSigmaPoints() {
  P_aug.topLeftCorner(n_x,n_x) = P;
  MatrixXd A = P_aug.llt().matrixL();

  x_aug.head(5) = x;
  Xsig_aug.col(0)  = x_aug;

  for (int i = 0; i< n_aug; i++) {
    Xsig_aug.col(i+1)        = x_aug + sqrt(lambda_+n_aug) * A.col(i);
    Xsig_aug.col(i+1+n_aug) = x_aug - sqrt(lambda_+n_aug) * A.col(i);
  }
}


void Ukf::PredictSigmaPoints(double dt) {
  for (int i = 0; i< total_sig_points; i++) {
    //extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*dt) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*dt) );
    }
    else {
        px_p = p_x + v*dt*cos(yaw);
        py_p = p_y + v*dt*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*dt;
    double yawd_p = yawd;

    //add noise
    double dt2 = dt*dt;
    px_p = px_p + 0.5*nu_a*dt2 * cos(yaw);
    py_p = py_p + 0.5*nu_a*dt2 * sin(yaw);
    v_p = v_p + nu_a*dt;

    yaw_p = yaw_p + 0.5*nu_yawdd*dt2;
    yawd_p = yawd_p + nu_yawdd*dt;

    //write predicted sigma point into right column
    Xsig_pred(0,i) = px_p;
    Xsig_pred(1,i) = py_p;
    Xsig_pred(2,i) = v_p;
    Xsig_pred(3,i) = yaw_p;
    Xsig_pred(4,i) = yawd_p;
  }
}


void Ukf::PredictMeanCovariance() {
  x =  Xsig_pred * weights;

  P.fill(0.0);
  for (int i = 0; i < total_sig_points; i++) {
    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x;
    //angle normalization
    x_diff(3) = atan2(sin(x_diff(3)), cos(x_diff(3)));

    P += weights(i) * x_diff * x_diff.transpose();
  }

  x_aug.head(5) = x;
  P_aug.topLeftCorner(5,5) = P;
}


void Ukf::UpdateLidar(SensorReading& reading) {
  //transform sigma points into measurement space
  MatrixXd Zsig = MatrixXd(n_z_lidar, total_sig_points);
  Zsig = H_lidar*Xsig_pred;

  CompleteUpdate(n_z_lidar, R_lidar_, Zsig, reading);
}


void Ukf::UpdateRadar(SensorReading& reading) {

  //transform sigma points into measurement space
  MatrixXd Zsig = MatrixXd(n_z_radar, total_sig_points);
  for (int i = 0; i < total_sig_points; i++) {

    double p_x = Xsig_pred(0,i);
    double p_y = Xsig_pred(1,i);
    double v  = Xsig_pred(2,i);
    double yaw = Xsig_pred(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);              //r
    Zsig(1,i) = atan2(p_y,p_x);                       //phi
    if(Zsig(0,i) < 1e-4) {            //r_dot check for div by 0
      Zsig(2,i) = (p_x*v1 + p_y*v2) / 1e-4;
    } else {
      Zsig(2,i) = (p_x*v1 + p_y*v2) / Zsig(0,i);
    }
    
  }

  CompleteUpdate(n_z_radar, R_radar_, Zsig, reading);
}


void Ukf::CompleteUpdate(int n_z, MatrixXd &R, MatrixXd &Zsig, SensorReading &reading) {
  //mean predicted measurement
  VectorXd z_pred = Zsig * weights;

  //measurement covariance matrix S
  MatrixXd S = MatrixXd::Zero(n_z,n_z);
  for (int i = 0; i < total_sig_points; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    z_diff(1) = atan2(sin(z_diff(1)), cos(z_diff(1)));

    S += weights(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  S += R;



  //calculate cross correlation matrix Tc
  MatrixXd Tc = MatrixXd::Zero(n_x, n_z);
  for (int i = 0; i < total_sig_points; i++) {
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    z_diff(1) = atan2(sin(z_diff(1)), cos(z_diff(1)));
    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x;
    //angle normalization
    x_diff(3) = atan2(sin(x_diff(3)), cos(x_diff(3)));

    Tc += weights(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd Si = S.inverse();
  MatrixXd K = Tc * Si;

  //error
  VectorXd z_error = reading.measurement - z_pred;

  //angle normalization
  z_error(1) = atan2(sin(z_error(1)), cos(z_error(1)));

  //correct x and P
  x = x + K * z_error;
  P = P - K*S*K.transpose();

  nis = z_error.transpose() * Si * z_error;
}