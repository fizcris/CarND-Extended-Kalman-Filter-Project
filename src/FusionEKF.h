#ifndef FusionEKF_H_
#define FusionEKF_H_

#include <fstream>
#include <string>
#include <vector>
#include "Eigen/Dense"
#include "kalman_filter.h"
#include "measurement_package.h"
#include "tools.h"

class FusionEKF {
 public:
  /**
   * Constructor.
   */
  FusionEKF();

  /**
   * Destructor.
   */
  virtual ~FusionEKF();

  /**
   * Run the whole flow of the Kalman Filter from here.
   */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
   * Kalman Filter update and prediction math lives in here.
   */
  KalmanFilter ekf_;


 private:
 void CalculateJacobianAndMeas(const Eigen::VectorXd &x_state);
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;
  bool err_meas_;

  // previous timestamp
  long long previous_timestamp_;

  // tool object used to compute Jacobian and RMSE
  Tools tools;
  Eigen::MatrixXd R_laser_; // measurement covariance matrix
  Eigen::MatrixXd R_radar_; // measurement covariance matrix
  Eigen::MatrixXd Hj_; // measurement matrix
  Eigen::MatrixXd Qv_; // individual process covariance matrix
  Eigen::MatrixXd G_; // time matrix for process covaraince Q

  //acceleration noise components
	float noise_ax;
	float noise_ay;


};

#endif // FusionEKF_H_
