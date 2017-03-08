#include "kalman_filter.h"
#include "tools.h"
#include <iostream>

using namespace std;

KalmanFilter::KalmanFilter() {
}

KalmanFilter::~KalmanFilter() {
}

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
	cout << "predict x_, P_ " << x_ << ", " << P_ << endl;
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
	cout << "P_ " << P_ << endl;
}

void KalmanFilter::Update(const VectorXd &z) {
	cout << "update z: " << z << endl;
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
//	long x_size = x_.size();
	long x_size = 4;
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
	cout << "update P_: " << P_ << endl;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
	/**
	 TODO:
	 * update the state by using Extended Kalman Filter equations
	 */

//	// convert from polar to cartesian
//	VectorXd z_cartesian = H_ * z;
//
//	VectorXd z_pred = H_ * x_;
//	VectorXd y = z_cartesian - z_pred;
//	MatrixXd Ht = H_.transpose();
//	MatrixXd S = H_ * P_ * Ht + R_;
//	MatrixXd Si = S.inverse();
//	MatrixXd PHt = P_ * Ht;
//	MatrixXd K = PHt * Si;
//
//	//new estimate
//	x_ = x_ + (K * y);
////	long x_size = x_.size();
//	long x_size = 4;
//	MatrixXd I = MatrixXd::Identity(x_size, x_size);
//	P_ = (I - K * Ht) * P_;
}
