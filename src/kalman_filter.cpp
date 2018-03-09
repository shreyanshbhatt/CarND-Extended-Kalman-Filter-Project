#include "kalman_filter.h"
#include <iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;
#define PI 3.14159265

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * update the state by using Kalman Filter equations
   */
  // Update for LIDAR
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;

  // Compute Kalaman Filter
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  // Update x_
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  // Update P_
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * update the state by using Extended Kalman Filter equations
  */
  VectorXd z_pred = VectorXd(3);
  if (x_(0) == 0. && x_(1) == 0.)
    return;
  double ro = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  double phi = atan2(x_(1), x_(0));
  double rho_dot = 0.0;
  rho_dot = (x_(0)*x_(2) + x_(1)*x_(3))/ro;
  z_pred << ro,phi,rho_dot;
  VectorXd y = z - z_pred;
  if( y[1] > PI )
    y[1] -= 2.f*PI;
  if( y[1] < -PI )
    y[1] += 2.f*PI;
  //std::cout << "\t radar error: \n" << phi << std::endl;
  // compute Kalaman filter
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  // Update new x_
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  // Update P_
  P_ = (I - K * H_) * P_;
}
