#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
//  x_prev.fill(0.0);
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
    x_prev = x_;
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
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
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
    VectorXd h_mu = VectorXd(3);
//    F_inv = (F_.transpose()*F_).
//    VectorXd x_prev = F_.inverse()*x_;
    h_mu(0) = sqrt(x_prev(0)*x_prev(0) + x_prev(1)*x_prev(1));
    h_mu(1) = atan2(x_prev(1),x_prev(0));
//    double pi= 3.14159;//265358979323846
    while (h_mu(1)> M_PI) h_mu(1)-=2.*M_PI;
    while (h_mu(1)<-M_PI) h_mu(1)+=2.*M_PI;
    
    if (fabs(x_prev(0))<1E-6 && fabs(x_prev(1))<1E-6)
    {
        h_mu(2) = 0;
    }
    else
    {
        h_mu(2) = (x_prev(0)*x_prev(2) + x_prev(1)*x_prev(3))/h_mu(0);
    }
    VectorXd z_pred = h_mu + H_ * (x_-x_prev);
//    VectorXd z_pred = H_ * x_;
    
//    while (z(1)> M_PI) z(1)-=2.*M_PI;
//    while (z(1)<-M_PI) z(1)+=2.*M_PI;

    while (z_pred(1)> M_PI) z_pred(1)-=2.*M_PI;
    while (z_pred(1)<-M_PI) z_pred(1)+=2.*M_PI;
    
    VectorXd y = z - z_pred;
    while (y(1)> M_PI) y(1)-=2.*M_PI;
    while (y(1)<-M_PI) y(1)+=2.*M_PI;

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
