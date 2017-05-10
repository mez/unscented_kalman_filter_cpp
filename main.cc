#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "utility.h"
#include "ukf.h"

using namespace std;
using Eigen::VectorXd;

using utility::SensorReading;
using utility::SensorType;
using utility::CheckArguments;
using utility::CheckFiles;
using utility::CalculateRmse;

int main(int argc, char* argv[]) {

  CheckArguments(argc, argv);

  string in_file_name_ = argv[1];
  ifstream in_file_(in_file_name_.c_str(), ifstream::in);

  string out_file_name_ = argv[2];
  ofstream out_file_(out_file_name_.c_str(), ofstream::out);

  CheckFiles(in_file_, in_file_name_, out_file_, out_file_name_);

  vector<SensorReading> sensor_readings;
  vector<VectorXd> ground_truths;

  string line;

  while (getline(in_file_, line)) {
    istringstream iss(line);

    string sensor_type;
    SensorReading sensor_reading;
    long long timestamp;

    // reads first element from the current line
    iss >> sensor_type;
    if (sensor_type.compare("L") == 0) {
      // LASER MEASUREMENT
      float x,y;
      iss >> x; iss >> y; iss >> timestamp;

      sensor_reading.sensor_type = SensorType::LASER;
      sensor_reading.measurement = VectorXd(2);
      sensor_reading.measurement << x, y;
      sensor_reading.timestamp = timestamp;
    } else if (sensor_type.compare("R") == 0) {
      // RADAR MEASUREMENT
      float ro, phi, ro_dot;
      iss >> ro; iss >> phi; iss >> ro_dot; iss >> timestamp;

      sensor_reading.sensor_type = SensorType::RADAR;
      sensor_reading.measurement = VectorXd(3);
      sensor_reading.measurement << ro, phi, ro_dot;
      sensor_reading.timestamp = timestamp;
    }

    sensor_readings.push_back(sensor_reading);
    // read ground truth data to compare later
    float x_gt, y_gt, vx_gt, vy_gt;
    iss >> x_gt; iss >> y_gt; iss >> vx_gt; iss >> vy_gt;

    VectorXd ground_truth(4);
    ground_truth << x_gt, y_gt, vx_gt, vy_gt;
    ground_truths.push_back(ground_truth);
  }

  Ukf ukf;
  vector<VectorXd> estimations;

  size_t number_of_measurements = sensor_readings.size();

  // column names for output file
  out_file_ << "time_stamp" << "\t";
  out_file_ << "px_state" << "\t";
  out_file_ << "py_state" << "\t";
  out_file_ << "v_state" << "\t";
  out_file_ << "yaw_angle_state" << "\t";
  out_file_ << "yaw_rate_state" << "\t";
  out_file_ << "sensor_type" << "\t";
  out_file_ << "NIS" << "\t";
  out_file_ << "px_measured" << "\t";
  out_file_ << "py_measured" << "\t";
  out_file_ << "px_ground_truth" << "\t";
  out_file_ << "py_ground_truth" << "\t";
  out_file_ << "vx_ground_truth" << "\t";
  out_file_ << "vy_ground_truth" << "\n";

  for (size_t k = 0; k < number_of_measurements; ++k) {
    SensorReading reading = sensor_readings[k];
    ukf.ProcessMeasurement(reading);

    // output the estimation
    VectorXd x_ = ukf.x;

    // timestamp
    out_file_ << reading.timestamp << "\t"; // pos1 - est

    // output the state vector
    out_file_ << ukf.x(0) << "\t"; // pos1 - est
    out_file_ << ukf.x(1) << "\t"; // pos2 - est
    out_file_ << ukf.x(2) << "\t"; // vel_abs -est
    out_file_ << ukf.x(3) << "\t"; // yaw_angle -est
    out_file_ << ukf.x(4) << "\t"; // yaw_rate -est

    switch (reading.sensor_type) {
      case SensorType::RADAR:
        // sensor type
        out_file_ << "radar" << "\t";

        // NIS value
        out_file_ << ukf.nis << "\t";
        out_file_ << reading.measurement(0) * cos(reading.measurement(1)) << "\t";
        out_file_ << reading.measurement(0) * sin(reading.measurement(1)) << "\t";
        break;

      case SensorType::LASER:
        // sensor type
        out_file_ << "lidar" << "\t";

        // NIS value
        out_file_ << ukf.nis << "\t";
        out_file_ << reading.measurement(0) << "\t";
        out_file_ << reading.measurement(1) << "\t";
        break;
    }

    // output the ground truth packages
    VectorXd ground_truth = ground_truths[k];
    out_file_ << ground_truth(0) << "\t";
    out_file_ << ground_truth(1) << "\t";
    out_file_ << ground_truth(2) << "\t";
    out_file_ << ground_truth(3) << "\n";

    // convert ukf x vector to cartesian to compare to ground truth
    VectorXd ukf_x_cartesian_ = VectorXd(4);

    float x_estimate_ = ukf.x(0);
    float y_estimate_ = ukf.x(1);
    float vx_estimate_ = ukf.x(2) * cos(ukf.x(3));
    float vy_estimate_ = ukf.x(2) * sin(ukf.x(3));

    ukf_x_cartesian_ << x_estimate_, y_estimate_, vx_estimate_, vy_estimate_;

    //cout << "ukf_x_cartesian_: " << ukf_x_cartesian_ << endl;

    estimations.push_back(ukf_x_cartesian_);
  }

  // compute the accuracy (RMSE)
  cout << "Accuracy - RMSE:" << endl << CalculateRmse(estimations, ground_truths) << endl;

  // close files
  if (out_file_.is_open()) {
    out_file_.close();
  }

  if (in_file_.is_open()) {
    in_file_.close();
  }

  cout << "Done!" << endl;
  return 0;
}
