#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
#include "math.h"

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

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  F_ = MatrixXd(4, 4);
  Q_ = MatrixXd(4, 4);
  Qv_ = MatrixXd(2, 2);
  G_ = MatrixXd(4, 2);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;
  
  // Set the process and measurement noises
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
  
  // set the acceleration noise components
  noise_ax = 9;
  noise_ay = 9;
  Qv_ << noise_ax, 0,
         0,        noise_ay;
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
    cout << "Started.." << endl;

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1000, 0,
              0, 0, 0, 1000;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      float rho_dot = measurement_pack.raw_measurements_[2]; 
      ekf_.x_ << rho*sin(phi), rho*cos(phi), 0., 0.;

      // Initialize covariance matrix
      Hj_ << tools.CalculateJacobian(ekf_.x_);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // Initialize state.
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0., 0.;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    previous_timestamp_ = measurement_pack.timestamp_;
    return;
  }

  // if (measurement_pack.sensor_type_ == MeasurementPackage::LASER){
  //   return;
  // }

  /**
   * Prediction
   */
  // cout << "Predicting.." << endl;

  /**
   * Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   */
  float dt = (measurement_pack.timestamp_ - previous_timestamp_)/1e6;
  // cout << "dt:\t" << dt << endl;

  F_ << 1, 0, dt, 0,
        0, 1, 0, dt,
        0, 0, 1, 0,
        0, 0, 0, 1;

  /* 
   * Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  G_ << 0.5*dt*dt, 0,
        0,         0.5*dt*dt,
        dt,        0,
        0,         dt;
  Q_ << G_ * Qv_ * G_.transpose();

  // Init EKF
  ekf_.Init(ekf_.x_, ekf_.P_, F_, H_laser_, R_laser_, Q_);
  ekf_.Predict();

  /**
   * Update
   */
  // cout << "Updating.." << endl;

  /**
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Init EKF
    ekf_.Init(ekf_.x_, ekf_.P_, F_, Hj_, R_radar_, Q_);

    // Radar updates
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    // cout << "RADAR UPDATED.." << endl;

  } else {
    // Init EKF
    ekf_.Init(ekf_.x_, ekf_.P_, F_, H_laser_, R_laser_, Q_);

    // Laser updates
    ekf_.Update(measurement_pack.raw_measurements_);
    // cout << "LASER UPDATED.." << endl;

  }

  // Update Jacobian
  if ((ekf_.x_[0]*ekf_.x_[0] + ekf_.x_[1]*ekf_.x_[1]) > 1e-4){
    Hj_ << tools.CalculateJacobian(ekf_.x_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;

  previous_timestamp_ = measurement_pack.timestamp_;
}
