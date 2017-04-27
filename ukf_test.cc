/*
 * The need to unit test and BDD is critical to all software. Especially if you are
 * doing matrix operations, where small bugs can easily go unnoticed.
 * Took the test cases from UKF quizzes and implement them in CATCH.
 *
 */

#include "libs/catch.hpp"
#include "ukf.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;

using namespace std;

using Matrix = MatrixXd; //typedef this so it is easier to write below.

/**
 * Creates our version of a mock object. With some predefined state to test against.
 * @return ukf object we can test
 */
Ukf GetDummyEkfObject() {
  Ukf ukf;
  //set state dimension
  ukf.n_x_ = 5;
  //set augmented dimension
  ukf.n_aug_ = 7;
  //Process noise standard deviation longitudinal acceleration in m/s^2
  ukf.std_a_ = 0.2;
  //Process noise standard deviation yaw acceleration in rad/s^2
  ukf.std_yawdd_ = 0.2;
  //define spreading parameter
  ukf.lambda_ = 3 - ukf.n_aug_;
  //set example state
  ukf.x_ = VectorXd(ukf.n_x_);
  ukf.x_ << 5.7441,
    1.3800,
    2.2049,
    0.5015,
    0.3528;
  //create example covariance matrix
  ukf.P_ = Matrix(ukf.n_x_, ukf.n_x_);
  ukf.P_ << 0.0043, -0.0013, 0.0030, -0.0022, -0.0020,
    -0.0013, 0.0077, 0.0011, 0.0071, 0.0060,
    0.0030, 0.0011, 0.0054, 0.0007, 0.0008,
    -0.0022, 0.0071, 0.0007, 0.0098, 0.0100,
    -0.0020, 0.0060, 0.0008, 0.0100, 0.0123;

  return ukf;
}

/*
  * Until we get to the BDD style test. The tests go down in order.
  * PREDICT STEP
  *  Generate Sigma Points
  *  Predict Sigma Points
  *  Predict Mean and Variance
  * ESTIMATE STEP
  *  Predict Measurement
  *  Update
  */
TEST_CASE("GenerateSigmaPoints() Test", "[sigmapoints]") {
  Ukf ukf = GetDummyEkfObject();

  Matrix Xsig_aug = Matrix(7, 15);
  ukf.GenerateSigmaPoints(&Xsig_aug);

  Matrix output = Matrix(7, 15);
  output
    << 5.7441, 5.85768, 5.7441, 5.7441, 5.7441, 5.7441, 5.7441, 5.7441, 5.63052, 5.7441, 5.7441, 5.7441, 5.7441, 5.7441, 5.7441,
    1.38, 1.34566, 1.52806, 1.38, 1.38, 1.38, 1.38, 1.38, 1.41434, 1.23194, 1.38, 1.38, 1.38, 1.38, 1.38,
    2.2049, 2.28414, 2.24557, 2.29582, 2.2049, 2.2049, 2.2049, 2.2049, 2.12566, 2.16423, 2.11398, 2.2049, 2.2049, 2.2049, 2.2049,
    0.5015, 0.44339, 0.631886, 0.516923, 0.595227, 0.5015, 0.5015, 0.5015, 0.55961, 0.371114, 0.486077, 0.407773, 0.5015, 0.5015, 0.5015,
    0.3528, 0.299973, 0.462123, 0.376339, 0.48417, 0.418721, 0.3528, 0.3528, 0.405627, 0.243477, 0.329261, 0.22143, 0.286879, 0.3528, 0.3528,
    0, 0, 0, 0, 0, 0, 0.34641, 0, 0, 0, 0, 0, 0, -0.34641, 0, 0, 0, 0, 0, 0, 0, 0, 0.34641, 0, 0, 0, 0, 0, 0, -0.34641;

  /*
   * If you want to learn about Eigen Matrix equality methods....
   * https://eigen.tuxfamily.org/dox/classEigen_1_1DenseBase.html#abf727b0aba068172f17c1725cf5d6819
   */
  REQUIRE(Xsig_aug.isApprox(output, 1e-6));
}

TEST_CASE("PredictSigmaPoints() Test") {
  Ukf ukf = GetDummyEkfObject();

  Matrix Xsig_aug = Matrix(7, 15);
  Xsig_aug << 5.7441,  5.85768,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.63052,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,
              1.38,  1.34566,  1.52806,     1.38,     1.38,     1.38,     1.38,     1.38,   1.41434,  1.23194,     1.38,     1.38,     1.38,     1.38,     1.38,
              2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049,   2.2049,   2.2049,   2.12566,  2.16423,  2.11398,   2.2049,   2.2049,   2.2049,   2.2049,
              0.5015,  0.44339, 0.631886, 0.516923, 0.595227,   0.5015,   0.5015,   0.5015,   0.55961, 0.371114, 0.486077, 0.407773,   0.5015,   0.5015,   0.5015,
              0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721,   0.3528,   0.3528,  0.405627, 0.243477, 0.329261,  0.22143, 0.286879,   0.3528,   0.3528,
              0,        0,        0,        0,        0,        0,  0.34641,        0,         0,        0,        0,        0,        0, -0.34641,        0,
              0,        0,        0,        0,        0,        0,        0,  0.34641,         0,        0,        0,        0,        0,        0, -0.34641;

  Matrix Xsig_pred = Matrix(5, 15);
  double delta_t = 0.1;

  ukf.PredictSigmaPoints(Xsig_aug, delta_t, &Xsig_pred);

  Matrix output = Matrix(5, 15);
  output << 5.93553, 6.06251, 5.92217, 5.9415, 5.92361, 5.93516, 5.93705, 5.93553, 5.80832, 5.94481, 5.92935, 5.94553, 5.93589, 5.93401, 5.93553,
            1.48939, 1.44673, 1.66484, 1.49719, 1.508, 1.49001, 1.49022, 1.48939, 1.5308, 1.31287, 1.48182, 1.46967, 1.48876, 1.48855, 1.48939,
            2.2049, 2.28414, 2.24557, 2.29582, 2.2049, 2.2049, 2.23954, 2.2049, 2.12566, 2.16423, 2.11398, 2.2049, 2.2049, 2.17026, 2.2049,
            0.53678, 0.473387, 0.678098, 0.554557, 0.643644, 0.543372, 0.53678, 0.538512, 0.600173, 0.395462, 0.519003, 0.429916, 0.530188, 0.53678, 0.535048,
            0.3528, 0.299973, 0.462123, 0.376339, 0.48417, 0.418721, 0.3528, 0.387441, 0.405627, 0.243477, 0.329261, 0.22143, 0.286879, 0.3528, 0.318159;
  REQUIRE(Xsig_pred.isApprox(output, 1e-6));
}

TEST_CASE("PredictMeanAndCovariance() Test") {
  Ukf ukf = GetDummyEkfObject();
  MatrixXd Xsig_pred = Matrix(5, 15);
  Xsig_pred << 5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
               1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
               2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
               0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
               0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

  VectorXd output_x = VectorXd(5);
  output_x << 5.93637,
              1.49035,
              2.20528,
              0.536853,
              0.353577;

  Matrix output_P = Matrix(5, 5);
  output_P << 0.00543425, -0.0024053, 0.00341576, -0.00348196, -0.00299378,
              -0.0024053, 0.010845, 0.0014923, 0.00980182, 0.00791091,
              0.00341576, 0.0014923, 0.00580129, 0.000778632, 0.000792973,
              -0.00348196, 0.00980182, 0.000778632, 0.0119238, 0.0112491,
              -0.00299378, 0.00791091, 0.000792973, 0.0112491, 0.0126972;

  VectorXd x_pred = VectorXd(5);
  Matrix P_pred = Matrix(5, 5);
  ukf.PredictMeanAndCovariance(Xsig_pred, &x_pred, &P_pred);

  REQUIRE(x_pred.isApprox(output_x, 1e-5));
  REQUIRE(P_pred.isApprox(output_P, 1e-5));
}

TEST_CASE("PredictRadarMeasurement() Test") {
  Ukf ukf = GetDummyEkfObject();
  //radar measurement noise standard deviation radius in m
  ukf.std_radr_  = 0.3;
  //radar measurement noise standard deviation angle in rad
  ukf.std_radphi_ = 0.0175;
  //radar measurement noise standard deviation radius change in m/s
  ukf.std_radrd_ = 0.1;

  ukf.R_radar_ = MatrixXd(3,3);
  ukf.R_radar_ << ukf.std_radr_*ukf.std_radr_, 0, 0,
    0, ukf.std_radphi_*ukf.std_radphi_, 0,
    0, 0,ukf.std_radrd_*ukf.std_radrd_;

  VectorXd output_z = VectorXd(3);
  output_z << 6.12155,
              0.245993,
              2.10313;

  Matrix output_S = Matrix(3, 3);
  output_S << 0.0946171, -0.000139448, 0.00407016,
              -0.000139448, 0.000617548, -0.000770652,
              0.00407016, -0.000770652, 0.0180917;


  //create example matrix with predicted sigma points
  MatrixXd Xsig_pred = MatrixXd(5, 15);
  Xsig_pred << 5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
               1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
               2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
               0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
               0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;


  VectorXd z_out = VectorXd(3);
  Matrix S_out = Matrix(3, 3);
  ukf.PredictRadarMeasurement(Xsig_pred, &z_out, &S_out);

  cout << "S_out: \n" << S_out << endl;
  REQUIRE(z_out.isApprox(output_z, 1e-6));
  REQUIRE(S_out.isApprox(output_S, 1e-3));

}

//
//TEST_CASE("Update() Test") {
//  REQUIRE(false);
//}
//
//SCENARIO("A sensor reading is sent to ProcessMeasurement()", "[use_radar_][use_laser_]") {
//  GIVEN( "A LIDAR measurement" ) {
//    WHEN( "the use_laser_ is set to false" ) {
//      THEN( "the state should not be updated." ) {
//        REQUIRE(false);
//      }
//    }
//
//    WHEN( "the use_laser_ is set to true" ) {
//      THEN( "the state should be updated." ) {
//        REQUIRE(false);
//      }
//    }
//  }
//
//  GIVEN( "A RADAR measurement" ) {
//    WHEN( "the use_radar_ is set to false" ) {
//      THEN( "the state should not be updated." ) {
//        REQUIRE(false);
//      }
//    }
//
//    WHEN( "the use_radar_ is set to true" ) {
//      THEN( "the state should be updated." ) {
//        REQUIRE(false);
//      }
//    }
//  }
//}
