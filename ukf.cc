#include <iostream>
#include "ukf.h"


using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::ArrayXd;
using utility::SensorReading;
using utility::SensorType;

Ukf::Ukf() {
  is_initialized_ = false;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd::Identity(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  ///* State dimension
  n_x_ = 5;

  ///* Augmented state dimension
  n_aug_ = 7;

  ///* Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  ///* the current NIS for radar
  NIS_radar_ = 0;

  ///* the current NIS for laser
  NIS_laser_ = 0;

  double weight_0 = lambda_/(lambda_+n_aug_);
  double weight_remaining = 0.5/(lambda_+n_aug_);
  weights_ = VectorXd::Zero(15);
  weights_ =  weights_.array() + weight_remaining;
  weights_(0) = weight_0;

  //add measurement noise covariance matrix
  R_radar_ = MatrixXd(3,3);
  R_radar_ << std_radr_*std_radr_, 0, 0,
              0, std_radphi_*std_radphi_, 0,
              0, 0,std_radrd_*std_radrd_;

  R_laser_ = MatrixXd(2,2);
  R_laser_ << std_laspx_*std_laspx_, 0,
              0, std_laspy_*std_laspy_;

  H_laser_ = MatrixXd(2,5);
  H_laser_ << 1, 0, 0, 0, 0,
              0, 1, 0, 0, 0;
}

Ukf::~Ukf() {}

/**
 * @param {SensorReading} reading The latest measurement data of
 * either radar or laser.
 */
void Ukf::ProcessMeasurement(SensorReading reading) {
  if (!is_initialized_) {
    switch (reading.sensor_type) {
      case SensorType::LASER:
        x_ << reading.measurement(0), reading.measurement(1), 0.0, 0.0, 0.0;
        break;
      case SensorType::RADAR:
        x_ << reading.measurement(0) * cos(reading.measurement(1)),
              reading.measurement(0) * sin(reading.measurement(1)),
              0.0, 0.0, 0.0;
        break;
    }
    //P_ is already set in the ctor.
    previous_timestamp_ = reading.timestamp;
    is_initialized_ = true;

    return;
  }


}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void Ukf::Prediction(double delta_t) {}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {SensorReading} reading
 */
void Ukf::UpdateLidar(SensorReading reading) {}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {SensorReading} reading
 */
void Ukf::UpdateRadar(const MatrixXd& Xsig_pred,
                      const MatrixXd& Zsig,
                      const VectorXd& z_pred,
                      const MatrixXd& S,
                      SensorReading reading) {

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd::Zero(5, 3);

  //calculate cross correlation matrix
  for (int i = 0; i < 15; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    z_diff(1) = atan2(sin(z_diff(1)), cos(z_diff(1)));

    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x_;
    //angle normalization
    x_diff(3) = atan2(sin(x_diff(3)), cos(x_diff(3)));

    Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = reading.measurement - z_pred;

  //angle normalization
  z_diff(1) = atan2(sin(z_diff(1)), cos(z_diff(1)));

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();
}

void Ukf::GenerateSigmaPoints(MatrixXd* Xsig_out) {
  /*
  * na = 7
  * lambda = 3-na
  * x_
  * P_
  * std_a_
  * std_yawdd_
  * MatrixXd Xsig = MatrixXd(na, 2*na+1)
  */

  //create augmented mean vector
  VectorXd x_aug = VectorXd(7);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  /*******************************************************************************
  * Student part begin
  ******************************************************************************/

  //create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  //create augmented covariance matrix
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  //create square root matrix
  MatrixXd A = P_aug.llt().matrixL();
  //create augmented sigma points
  Xsig_aug.col(0) = x_aug;

  double sqrt_lambda_n_aug = sqrt(lambda_+n_aug_);
  for (int i=0; i < n_aug_; ++i) {
      Xsig_aug.col(i+1) = x_aug + sqrt_lambda_n_aug*A.col(i);
      Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt_lambda_n_aug*A.col(i);
  }

  *Xsig_out = Xsig_aug;
}

void Ukf::PredictSigmaPoints(const Eigen::MatrixXd& Xsig_aug, const double dt, Eigen::MatrixXd* Xsig_out) {
  MatrixXd Xsig_pred = MatrixXd(5, 15);
  double dt2 = dt*dt;

  for (int i = 0; i< 15; ++i) {

    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    //predict sigma points
    //avoid division by zero
    //write predicted sigma points into right column
    //predicted state values
    double px_p, py_p;

    double v_p = v;
    double yaw_p = yaw + yawd*dt;
    double yawd_p = yawd;

    //avoid division by zero
    if (fabs(yawd) > 1e-3) {
      px_p = p_x + v/yawd * ( sin (yaw_p) - sin(yaw));
      py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw_p) );
    } else {
      px_p = p_x + v*dt*cos(yaw);
      py_p = p_y + v*dt*sin(yaw);
    }

    //add noise
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

    *Xsig_out = Xsig_pred;
  }
}

void Ukf::PredictMeanAndCovariance(const MatrixXd& Xsig_pred, VectorXd* x_pred, MatrixXd* P_pred) {
  //create vector for predicted state
  VectorXd x = VectorXd(5);
  //create covariance matrix for prediction
  MatrixXd P = MatrixXd::Zero(5, 5);

  //Predict mean here.
  x = Xsig_pred * weights_;

  MatrixXd x_diff = MatrixXd(5, 15);
  //we could do a matrix of diffs like this, but I still haven't figured out
  // how to normalize the angles in a vectorized manner yet. :(
  //x_diff = Xsig_pred.colwise() - x;

  //predicted state covariance matrix
  for (int i = 0; i < 15; i++) {  //iterate over sigma points
    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x;
    //angle normalization
    x_diff(3) = atan2(sin(x_diff(3)), cos(x_diff(3)));
    P += weights_(i) * x_diff * x_diff.transpose();
  }

  *x_pred = x;
  *P_pred = P;
}

void Ukf::PredictLidarMeasurement(const MatrixXd& Xsig_pred, VectorXd* z_out, MatrixXd* S_out) {
  MatrixXd Zsig_pred = MatrixXd(2, 15);

  Zsig_pred = H_laser_ * Xsig_pred;

  //mean predicted measurement
  VectorXd z_pred = VectorXd::Zero(2);
  z_pred = Zsig_pred * weights_;

  MatrixXd S = MatrixXd::Zero(2,2);
  for (int i = 0; i < 15; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig_pred.col(i) - z_pred;
    S += weights_(i) * z_diff * z_diff.transpose();
  }

  S += R_laser_;

  *z_out = z_pred;
  *S_out = S;
}

void Ukf::PredictRadarMeasurement(const MatrixXd& Xsig_pred, VectorXd* z_out, MatrixXd* S_out) {
  MatrixXd Zsig_pred = MatrixXd(3, 15);

  //transform sigma points into measurement space
  for (int i = 0; i < 15; i++) {
    // extract values for better readibility
    double p_x = Xsig_pred(0,i);
    double p_y = Xsig_pred(1,i);
    double v  = Xsig_pred(2,i);
    double yaw = Xsig_pred(3,i);
    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig_pred(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Zsig_pred(1,i) = atan2(p_y,p_x);                                 //phi
    Zsig_pred(2,i) = (Zsig_pred(0,i) < 1e-4) ? 0.0 : (p_x*v1 + p_y*v2 )/sqrt(p_x*p_x + p_y*p_y);   //r_dot
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd::Zero(3);
  z_pred = Zsig_pred * weights_;

  //measurement covariance matrix S
  MatrixXd S = MatrixXd::Zero(3,3);

  for (int i = 0; i < 15; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig_pred.col(i) - z_pred;
    //angle normalization
    z_diff(1) = atan2(sin(z_diff(1)), cos(z_diff(1)));

    S += weights_(i) * z_diff * z_diff.transpose();
  }
  S += R_radar_;

  *z_out = z_pred;
  *S_out = S;
}
