#include "kalman_filter.h"
#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

# define TOL 0.0001
# define SIGN(a) ((0<a)-(a<0))
# define CHECK_ZERO(a) (fabs(a)<TOL ? SIGN(a)*TOL : a) 

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict(float delta_T) {
  /**
    * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
  */
  F_(0,2) = delta_T;
  F_(1,3) = delta_T; 
  MatrixXd Ft = F_.transpose();

  /**
    * Update the process noise covariance matrix.
  */
  float dt_2 = delta_T * delta_T;
  float dt_3 = dt_2 * delta_T;
  float dt_4 = dt_3 * delta_T;
  
  Q_ <<  dt_4/4*noise_ax_, 0, dt_3/2*noise_ax_, 0,
		 0, dt_4/4*noise_ay_, 0, dt_3/2*noise_ay_,
		 dt_3/2*noise_ax_, 0, dt_2*noise_ax_, 0,
		 0, dt_3/2*noise_ay_, 0, dt_2*noise_ay_;

  /**
    * Predict the state x
  */
  x_ = F_ * x_;
  P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_laser_ * x_;
  VectorXd y = z - z_pred;

  UpdateEstimate(y, H_laser_, R_laser_); 
}

VectorXd KalmanFilter::CartesianToPolar(const VectorXd &x) {
  float px = x(0);
  float py = x(1);
  float vx = x(2);
  float vy = x(3);
  float c1 = sqrt(px*px +py*py);
  float phi = (px==0&&py==0) ? 1.0 : atan2(py,px);

  VectorXd hx = VectorXd(3);
  hx << c1,
       phi,
       (px*vx+py*vy)/CHECK_ZERO(c1);
    
}

MatrixXd KalmanFilter::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
	MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//pre-compute a set of terms to avoid repeated calculation
	float c1 = CHECK_ZERO(px*px+py*py);
	float c2 = sqrt(c1);
	float c1c2 = CHECK_ZERO(c1*c2);
    float pxc2 = px/c2;
    float pyc2 = py/c2;

	//compute the Jacobian matrix
	Hj << pxc2, pyc2, 0, 0,
		  -py/c1, px/c1, 0, 0,
		  py*(vx*py - vy*px)/c1c2, px*(px*vy - py*vx)/c1c2, pxc2, pyc2;

	return Hj;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
    * update the state by using Extended Kalman Filter equations
  */

  VectorXd y = z - CartesianToPolar(x_); 

  // normalize angle to be between -pi and pi
  while (y[1] < -M_PI) y[1] += 2 * M_PI;
  while (y[1] > M_PI) y[1] -= 2 * M_PI;
  
  MatrixXd Hj = CalculateJacobian(x_);

  UpdateEstimate(y, Hj, R_radar_);  
}

void KalmanFilter::UpdateEstimate(const VectorXd &y, const MatrixXd &H, const MatrixXd &R) {

  MatrixXd Ht = H.transpose();
  MatrixXd S = H * P_ * Ht + R;
  MatrixXd K = P_ * Ht * S.inverse();

  x_ += K * y;
  P_ -= K * H * P_;

}



