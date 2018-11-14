#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <cmath>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  ekf_.R_laser_ = MatrixXd(2, 2);
  ekf_.R_radar_ = MatrixXd(3, 3);
  ekf_.H_laser_ = MatrixXd(2, 4);

  //measurement covariance matrix - laser
  ekf_.R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  ekf_.R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  ekf_.H_laser_ << 1, 0, 0, 0,
			      0, 1, 0, 0;
  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */


}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    previous_timestamp_ = measurement_pack.timestamp_;

    //P_in: used to initialize state covariance matrix P
    MatrixXd P_in = MatrixXd(4,4);
	P_in << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1000, 0,
			0, 0, 0, 1000;

	//F_in: used to initialize transition matrix F_
	MatrixXd F_in = MatrixXd(4, 4);
	F_in << 1, 0, 1, 0,
			0, 1, 0, 1,
			0, 0, 1, 0,
			0, 0, 0, 1;

    VectorXd x_in = VectorXd(4);
    MatrixXd Q_in = MatrixXd(4,4); //process covariance

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      double rho = measurement_pack.raw_measurements_(0);
      double theta = measurement_pack.raw_measurements_(1);
      x_in << rho*cos(theta), rho*sin(theta), 0, 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      x_in << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

    // Use noise_ax = 9 and noise_ay = 9 for Q matrix.
    ekf_.noise_ax_ = 9;
    ekf_.noise_ay_ = 9;

    ekf_.Init(x_in, P_in, F_in, Q_in);

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
     * Update the process noise covariance matrix.
   */
  float delta_T = (measurement_pack.timestamp_ - previous_timestamp_)/ 1000000.0;;
  previous_timestamp_ = measurement_pack.timestamp_;
 
  ekf_.Predict(delta_T);

  /*std::cout << "After Predict: " << std::endl;			   
  std::cout << "dt = " << delta_T << std::endl;		   
  std::cout << "Q_= " << ekf_.Q_ << std::endl;
  std::cout << "x_= " << ekf_.x_ << std::endl;
  std::cout << "P_= " << ekf_.P_ << std::endl;*/
  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates (Extended Kalman Filter)
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Laser updates (regular Kalman Filter, since lidar uses linear equations)
    ekf_.Update(measurement_pack.raw_measurements_);

  }


  // print the output
  // std::cout << "After Update: " << std::endl;
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
