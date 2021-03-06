#ifndef UKF_H
#define UKF_H

#include "libs/Eigen/Dense"
#include "utility.h"


class Ukf {

public:

  bool is_initialized;
  long previous_timestamp;

  // state vector [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::VectorXd x;
  Eigen::VectorXd x_aug;

  // state covariance matrix
  Eigen::MatrixXd P;
  Eigen::MatrixXd P_aug;

  // sigma points matrices and weights
  Eigen::MatrixXd Xsig_aug;
  Eigen::MatrixXd Xsig_pred;
  Eigen::VectorXd weights;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a;

  // Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd;

  // Laser measurement noise standard deviation position1 in m
  double std_laspx;

  // Laser measurement noise standard deviation position2 in m
  double std_laspy;

  // Radar measurement noise standard deviation radius in m
  double std_radr;

  // Radar measurement noise standard deviation angle in rad
  double std_radphi;

  // Radar measurement noise standard deviation radius change in m/s
  double std_radrd ;

  // Measurement noise covariance matrices
  Eigen::MatrixXd R_radar;
  Eigen::MatrixXd R_lidar;
  Eigen::MatrixXd H_lidar;

  ///* State dimension
  int n_x;

  ///* Augmented state dimension
  int n_aug;

  int n_z_radar;
  int n_z_lidar;

  ///* Sigma point spreading parameter
  double lambda;

  ///* the current NIS for last processed measurement
  double nis;

  ///* the total number of sigma points
  int total_sig_points;

  
  // Constructor
  Ukf();
  

  void GenerateSigmaPoints();
  void PredictSigmaPoints(double dt);
  void PredictMeanCovariance();

  /**
   * ProcessMeasurement
   * @param reading The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(const utility::SensorReading& reading);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param dt Time between k and k+1 in s
   */
  void Prediction(double dt);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param reading The measurement at k+1
   */
  void UpdateLidar(const utility::SensorReading& reading);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param reading The measurement at k+1
   */
  void UpdateRadar(const utility::SensorReading& reading);
  void CompleteUpdate(const Eigen::MatrixXd& Zsig, const utility::SensorReading& reading);

private:
  void InitializeState(const utility::SensorReading& reading);
};

#endif /* UKF_H */
