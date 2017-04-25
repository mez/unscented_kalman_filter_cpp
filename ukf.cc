#include "ukf.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using utility::SensorReading;

Ukf::Ukf() {
  is_initialized_ = false;
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

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
}

Ukf::~Ukf() {}

/**
 * @param {SensorReading} reading The latest measurement data of
 * either radar or laser.
 */
void Ukf::ProcessMeasurement(SensorReading reading) {}

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
void Ukf::UpdateRadar(SensorReading reading) {}

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

  for (int i = 0; i< 15; i++) {

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
void Ukf::PredictMeanAndCovariance(VectorXd* x_pred, MatrixXd* P_pred){}
void Ukf::PredictLidarMeasurement(VectorXd* z_out, MatrixXd* S_out){}
void Ukf::PredictRadarMeasurement(VectorXd* z_out, MatrixXd* S_out){}
