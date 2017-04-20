#include "ukf.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

UKF::UKF() {
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
  double lambda_;

  ///* the current NIS for radar
  double NIS_radar_ = 0;

  ///* the current NIS for laser
  double NIS_laser_ = 0;
}

UKF::~UKF() {}

/**
 * @param {SensorReading} reading The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(SensorReading reading) {}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {SensorReading} reading
 */
void UKF::UpdateLidar(SensorReading reading) {}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {SensorReading} reading
 */
void UKF::UpdateRadar(SensorReading reading) {}
