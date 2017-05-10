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
  double std_radr_;

  // Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  // Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  // Measurement noise covariance matrices
  Eigen::MatrixXd R_radar_;
  Eigen::MatrixXd R_lidar_;
  Eigen::MatrixXd H_lidar;

  ///* State dimension
  int n_x;

  ///* Augmented state dimension
  int n_aug;

  int n_z_radar;
  int n_z_lidar;

  ///* Sigma point spreading parameter
  double lambda_;

  ///* the current NIS for last processed measurement
  double nis;

  ///* the total number of sigma points
  int total_sig_points;

  
  // Constructor
  Ukf();
  
  void InitializeState(utility::SensorReading &reading);
  void GenerateSigmaPoints();
  void PredictSigmaPoints(double dt);
  void PredictMeanCovariance();

  /**
   * ProcessMeasurement
   * @param reading The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(utility::SensorReading& reading);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param dt Time between k and k+1 in s
   */
  void Prediction(double dt);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(utility::SensorReading& reading);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param reading The measurement at k+1
   */
  void UpdateRadar(utility::SensorReading& reading);
  void CompleteUpdate(int n_z,
                      Eigen::MatrixXd &R,
                      Eigen::MatrixXd &Zsig,
                      utility::SensorReading &reading);
};

#endif /* UKF_H */
