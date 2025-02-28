#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"



using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  Hj_ = MatrixXd(3, 4);
  Q_ = MatrixXd(4, 4);
  G_ = MatrixXd(4, 2);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  
  // set the acceleration noise components
  noise_ax = 9;
  noise_ay = 9;

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * Initialize the state ekf_.x_ with the first measurement.
     * Create & initiallize the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.P_ = MatrixXd(4,4);
    ekf_.F_ = MatrixXd(4,4);

    //ekf_.H_ = MatrixXd(2, 4); //It cahnges
    ekf_.h_ = VectorXd(3);
    ekf_.Q_ = MatrixXd(4,4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      float tanphi = tan(phi);
      
      float px = sqrt(pow(rho,2)/(1+pow(tanphi,2)));
      float py = px * tanphi;



      ekf_.x_ <<  px,
                  py,
                  0,
                  0;
      
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      //  Initialize state for Laser.
      ekf_.x_ << measurement_pack.raw_measurements_[0],
            measurement_pack.raw_measurements_[1],
            0,
            0;

    }


    // Initializacion covariance matrix Lidar - Radar (1st iteration)
    ekf_.P_ <<  0.05, 0, 0, 0,
                0, 0.05, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;




    // Update prevoius timestamp
    previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  // Compute dt
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  

  // Predict state matrix (mean)
  ekf_.F_ <<  1, 0, dt, 0,
              0, 1, 0, dt,
              0, 0, 1, 0,
              0, 0, 0, 1;

  // Predict state variance
  G_ << pow(dt, 2.0) / 2.0, 0,
        0, pow(dt, 2.0) / 2.0,
        dt, 0,
        0, dt;


  // Predict process variance
  float dt2 = dt*dt;
  float dt3 = dt2*dt;
  float dt4 = dt3*dt;
  ekf_.Q_<< (dt4*noise_ax/4.0), 0, (dt3*noise_ax/2), 0,
            0, (dt4*noise_ay/  4), 0, (dt3*noise_ay/2),
            (dt3*noise_ax/2), 0, (dt2*noise_ax), 0,
            0, (dt3*noise_ay/2), 0, (dt2*noise_ay);

  // Perform prediction of x and P
  ekf_.Predict();

  /**
   * Update
  */

  /**
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    //update measurement covariance matrix - radar
    ekf_.R_ = R_radar_;

    ekf_.H_ = MatrixXd(3, 4);

    CalculateJacobianAndMeas(ekf_.x_);
    if (!err_meas_){
      ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    }
    

  } else {
    // Laser updates
    //update measurement covariance matrix - laser
    ekf_.R_ = R_laser_;

    
    ekf_.H_ = MatrixXd(2, 4);
    // Measurement matrix
    ekf_.H_ << 1, 0, 0, 0,
               0, 1, 0, 0;

    // Correct states with measurement
    ekf_.Update(measurement_pack.raw_measurements_);

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}

void FusionEKF::CalculateJacobianAndMeas(const VectorXd &x_state) {

  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // check division by zero
  if ((px<0.0001) && (py<0.0001)) {
      //cout << "Error: px and py are 0 - Discarded meas" << endl << endl;
      err_meas_ = true;
    return;
    };
  err_meas_=false;
  // compute the Jacobian matrix
  float powerP = pow(px,2) + pow(py,2);
  float den = sqrt(powerP);
  float den2 = pow(den,3);

  Hj_ << px/den,                     py/den,                      0,      0,
        -py/powerP,                  px/powerP,                   0,      0,
        py*(vx*py - vy*px)/den2,     px*(vy*px - vx*py)/den2,     px/den, py/den;

  //Update measurement vector

  float atanNorm =  atan2(py,px);

  ekf_.h_ << den, atanNorm,  (px*vx +py*vy) / den;
  //ekf_.h_= ekf_.h_ +  Hj_*x_state;

  ekf_.H_ =  Hj_;


  return;
}