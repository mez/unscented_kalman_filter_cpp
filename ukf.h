#ifndef UKF_H
#define UKF_H

#include "libs/Eigen/Dense"
#include "utility.h"

class Ukf {
 public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::VectorXd x_;

  ///* state covariance matrix
  Eigen::MatrixXd P_;

  ///* let's setup the noise matrices for sensors to save compute later
  Eigen::MatrixXd R_radar_;
  Eigen::MatrixXd R_laser_;

  ///* for Zpred transformation
  Eigen::MatrixXd H_laser_;


  ///* time when the state is true, in us
  long long previous_timestamp_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* Weights of sigma points
  Eigen::VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Sigma point spreading parameter
  double lambda_;

  ///* the current NIS for radar
  double NIS_radar_;

  ///* the current NIS for laser
  double NIS_laser_;

  ///* Sigma point prediction
  Eigen::MatrixXd Xsig_pred_;
  Eigen::MatrixXd Zsig_pred_;


  /**
   * Constructor
   */
  Ukf();

  /**
   * Destructor
   */
  virtual ~Ukf();

  /**
   * ProcessMeasurement
   * @param reading The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(utility::SensorReading reading);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   *
   * Update step.
   * @param reading  The measurement at k+1
   */
  void Update(utility::SensorReading reading);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param reading The measurement at k+1
   */
  void UpdateLidar(const Eigen::MatrixXd& Xsig_pred,
                   const Eigen::MatrixXd& Zsig,
                   const Eigen::VectorXd& z_pred,
                   const Eigen::MatrixXd& S,
                   utility::SensorReading reading);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param reading The measurement at k+1
   */
  void UpdateRadar(const Eigen::MatrixXd& Xsig_pred,
                   const Eigen::MatrixXd& Zsig,
                   const Eigen::VectorXd& z_pred,
                   const Eigen::MatrixXd& S,
                   utility::SensorReading reading);

  /**
   * Generates augmented sigma points
   * @param Xsig_out expects to be a (7,15) matrix to hold the result.
   */
  void GenerateSigmaPoints(Eigen::MatrixXd* Xsig_out);

  /**
   * Given the sigma points and delta time (dt) it will make the sigma point prediction.
   *
   * @param Xsig_aug the augmented sigma points to use.
   * @param dt the delta time
   * @param Xsig_pred the output to place results. Expected to be (5,15)
   */
  void PredictSigmaPoints(const Eigen::MatrixXd& Xsig_aug, const double dt, Eigen::MatrixXd* Xsig_out);
  void PredictMeanAndCovariance(const Eigen::MatrixXd& Xsig_pred);
  void PredictLidarMeasurement(const Eigen::MatrixXd& Xsig_pred,
                               Eigen::VectorXd* z_out,
                               Eigen::MatrixXd* S_out,
                               Eigen::MatrixXd* Zsig_out);

  void PredictRadarMeasurement(const Eigen::MatrixXd& Xsig_pred,
                               Eigen::VectorXd* z_out,
                               Eigen::MatrixXd* S_out,
                               Eigen::MatrixXd* Zsig_out);

};

#endif /* UKF_H */
