#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

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
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd::Zero(3, 4);
  //Tools tools;

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        		0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        		0, 0.0009, 0,
        		0, 0, 0.09;
  //section 10 in lesson 5
  H_laser_ << 1, 0, 0, 0,
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
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 0, 0; // important for RMSE

    ekf_.P_ = MatrixXd(4, 4); // Covariance matrix
  	ekf_.P_ << 1, 0, 0, 0,
			  0, 1, 0, 0,
			  0, 0, 1000, 0,
			  0, 0, 0, 1000;



    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
    	float rho = measurement_pack.raw_measurements_[0];
    	float phi = measurement_pack.raw_measurements_[1];
    	float rho_dot = measurement_pack.raw_measurements_[2];

    	float px = rho * cos(phi);
    	float py = rho * sin(phi);
    	//float vx = rho_dot * cos(phi);
    	//float vy = rho_dot * sin(phi);

    	ekf_.x_ << px, py, 0, 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
    	float px = measurement_pack.raw_measurements_[0];
    	float py = measurement_pack.raw_measurements_[1];

    	ekf_.x_ << px, py, 0, 0;
    }
    	
    if (fabs(ekf_.x_(0)) < 0.001 || fabs(ekf_.x_(1)) < 0.001){
    	ekf_.x_(0)=0.001;
    	ekf_.x_(1)=0.001;
    }


    previous_timestamp_ = measurement_pack.timestamp_;


    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  //set the acceleration noise components
  float noise_ax = 9;//provided in the quiz as 9 in section 13 of lesson 5
  float noise_ay = 9;//provided in the quiz as 9


  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; //in sec's
  previous_timestamp_ = measurement_pack.timestamp_;

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  ekf_.F_ = MatrixXd(4, 4); // State transition
  ekf_.F_ << 1, 0, 1, 0,
			  0, 1, 0, 1,
			  0, 0, 1, 0,
			  0, 0, 0, 1;

  //Modify F matrix so time is integrated. Lesson 5.8
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  //Set the process covariance matrix Q. Lesson 5.9
  ekf_.Q_ = MatrixXd::Zero(4,4); //Initialize matrix
  ekf_.Q_ << dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
  			0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay, 
  			dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
  			0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

  //if (dt > 0.0001){
  ekf_.Predict();
  //}

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    Hj_ = tools.CalculateJacobian(ekf_.x_);//set to Hj - calculated jacobian from tools file
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
  	// print the output
  	ekf_.H_ = H_laser_;
  	ekf_.R_ = R_laser_;
  	ekf_.Update(measurement_pack.raw_measurements_);
  }

  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
