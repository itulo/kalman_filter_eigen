/* Track car state with Kalman filter as in Examples 4.3 of the book

   Simo Sarkka (2013), Bayesian Filtering and Smoothing,
   Cambridge University Press. 

   http://becs.aalto.fi/~ssarkka/pub/cup_book_online_20131111.pdf
*/

#include <iostream>
#include "Eigen/Dense"
#include "kalman.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// global vars
KalmanFilter kf;
MatrixXd A,H,Q,x0,X,Y;
const double s = 0.5;
const int steps = 100;

/* Initialize Kalman Filter */
void initialize(){
	const int q = 1;
	const double dt = 0.1;
	MatrixXd R,P0;

	A = MatrixXd::Identity(4,4);
	A(0,2) = A(1,3) = dt;

	H = MatrixXd::Zero(2,4);
	H(0,0) = H(1,1) = 1;

	Q = MatrixXd::Zero(4,4);
	Q(0,0) = Q(1,1) = pow(dt,3)/3;
	Q(0,2) = Q(1,3) = Q(2,0) = Q(3,1) = pow(dt,2)/2;
	Q(2,2) = Q(3,3) = dt;
	Q *= q;

	R = pow(s,2) * MatrixXd::Identity(2,2);

	x0 = VectorXd::Zero(4);
	x0(2) = -1;
	x0(3) = 1;

	P0 = MatrixXd::Identity(4,4);

	kf = KalmanFilter(A,H,Q,R,P0);
	kf.init(x0);
}

/* Simulate position and velocity */
void simulateData(){
	int ARows = A.rows();
	X = MatrixXd::Zero(ARows,steps);
	Y = MatrixXd::Zero(H.rows(),steps);
	MatrixXd q;
	VectorXd x = x0;
	VectorXd y;
	Eigen::LLT<MatrixXd> lltOfQ(Q); // compute the Cholesky decomposition of Q
	MatrixXd cholQTranspose = lltOfQ.matrixL().transpose(); // retrieve factor L in the decomposition

	for(int i=0;i<steps;i++){
		q = cholQTranspose * MatrixXd::Random(ARows,1);
		x = A * x + q;
		y = H * x + s * MatrixXd::Random(2,1);
		X.col(i) = x;
		Y.col(i) = y;
	}
}

/* Run the Kalman Filter for estimation */
void estimate(){
	VectorXd yCurr;
	MatrixXd yDiff;
	MatrixXd xEst = MatrixXd::Zero(X.rows(),X.cols());

	for(int i=0;i<steps;i++){
		yCurr = Y.col(i);
		xEst.col(i) = kf.update(Y.col(i));

		cout<<"Inputs: "<<yCurr.transpose()<<"\t\tEstimation: "<<xEst.col(i).transpose()<<endl;
	}

	yDiff = Y-xEst.block<2,steps>(0,0);
	cout<<"RMSE on position: "<<sqrt(yDiff.row(0).array().square().mean())<<endl;
	cout<<"RMSE on velocity: "<<sqrt(yDiff.row(1).array().square().mean())<<endl;
}

int main() {
	initialize();
	simulateData();
	estimate();

	return 0;
}
