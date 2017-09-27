#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

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
  TODO:
    * predict the state
  */
	x_ = F_*x_;//lesson 5.8
	MatrixXd Ft = F_.transpose(); //lesson 5.9
	P_ = F_*P_*Ft+Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations. Lesson 5.7
  */
	VectorXd z_pred = H_*x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd PHt = P_*Ht;
	MatrixXd S = H_* PHt + R_;
	MatrixXd Si = S.inverse();
	MatrixXd K =  PHt * Si;

	//new state
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
	//lesson 5.14
	float px = x_(0);
	float py = x_(1);
	float vx = x_(2);
	float vy = x_(3);



	float rho = sqrt(px*px+py*py);

	float eps = 0.000001;  // Make sure we don't divide by 0.
  	if (fabs(px) < eps || fabs(rho) < eps){
  		cout << "UpdateEKF - Division by zero error" << endl;
	  	return;
  	}

	float theta = atan(py/px);
	float ro_dot = (px*vx+py*vy)/rho;

	VectorXd z_pred = VectorXd(3);
	//z_pred << 1,1,1; //initialize
	z_pred << rho,theta,ro_dot;

	VectorXd y = z - z_pred;

	//lesson 5.7
	MatrixXd Ht = H_.transpose();
	MatrixXd PHt = P_*Ht;
	MatrixXd S = H_*PHt + R_;
	MatrixXd Si = S.inverse();
	MatrixXd K =  PHt * Si;


	//Update state & covariance matrix
	x_ = x_ + (K * y);
  	long x_size = x_.size();
  	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}
