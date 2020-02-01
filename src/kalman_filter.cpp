#include "kalman_filter.h"
#include "math.h"
#include <iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * predict the state
   */
  std::cout << "In Predict.." <<std::endl;
  // std::cout << "X" << x_ << std::endl;
  // std::cout << "F" << F_ << std::endl;
  // std::cout << "P" << P_ << std::endl;
  // std::cout << "Q" << Q_ << std::endl;

  x_ = F_ * x_;
  std::cout << "In Predict 2.." <<std::endl;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * update the state by using Kalman Filter equations
   */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * update the state by using Extended Kalman Filter equations
   */

  VectorXd h_x;
  h_x = VectorXd(3);

  float mag_p = sqrt(pow(x_[0], 2) + pow(x_[1], 2));
  h_x << mag_p, atan2(x_[1], x_[0]), (x_[0]*x_[2]+x_[1]*x_[3])/mag_p;

  // VectorXd y = z - measurement_fn(x_);
  VectorXd y = z - h_x;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

// VectorXd measurement_fn(VectorXd x){
//   VectorXd h_x;
//   h_x = VectorXd(3);

//   float mag_p = sqrt(pow(x[0], 2) + pow(x[1], 2));
//   h_x << mag_p, atan2(x[1], x[0]), (x[0]*x[2]+x[1]*x[3])/mag_p;

//   return h_x;
// }