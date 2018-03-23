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
}

void KalmanFilter::UpdateQ(float dt) {

	//Pre-calculate components for readability
	
	float dt2 = dt*dt;
	float dt3 = dt2*dt;
	float dt4 = dt3*dt;
		
	// Update Q_
	
	Q_ << dt4 * noise_ax_/4, 0, dt3 * noise_ax_/2, 0,
		  0, dt4 * noise_ay_/4, 0, dt3 * noise_ay_/2,
		  dt3 * noise_ax_/2, 0, dt2 * noise_ax_, 0,
		  0, dt3 * noise_ay_/2, 0, dt2 * noise_ay_;
		  
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
  
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  
  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd K = P_ * Ht * S.inverse();
  
  MatrixXd I = MatrixXd::Identity(4,4);
  
  x_ = x_ + K * y;
  P_ = (I - K * H_) * P_;
  
  
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  
  float x = x_(0);
  float y = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  
  float rho = sqrt(x*x + y*y);
  float theta = atan2(y, x);
  float rhodot = (x*vx + y*vy)/rho;
  VectorXd hx = VectorXd(3);
  hx << rho, theta, rhodot;

  VectorXd diff = z - hx;
  diff(1) = atan2(sin(diff(1)), cos(diff(1)));

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_; 
  MatrixXd K = P_ * Ht * S.inverse();
  
  x_ = x_ + K * diff;
  
  MatrixXd I = MatrixXd::Identity(4,4);
  P_ = (I - K * H_) * P_;
  
}
