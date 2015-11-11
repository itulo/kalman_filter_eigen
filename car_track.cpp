#include <iostream>
#include "Eigen/Dense"
#include "kalman.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// global vars
KalmanFilter kf;
MatrixXd A,H,Q,x0,X,Y;
const double dt = 0.1;
const double s = 0.5;
const int steps = 100;

void initialize(){
	const int q = 1;
	MatrixXd R,P0;

	A = MatrixXd::Identity(4,4);
	A(0,2) = A(1,3) = dt;
	//cout<<"A:\n"<<A<<endl;

	H = MatrixXd::Zero(2,4);
	H(0,0) = H(1,1) = 1;
	//cout<<"H:\n"<<H<<endl;

	Q = MatrixXd::Zero(4,4);
	Q(0,0) = Q(1,1) = pow(dt,3)/3;
	Q(0,2) = Q(1,3) = Q(2,0) = Q(3,1) = pow(dt,2)/2;
	Q(2,2) = Q(3,3) = dt;
	Q *= q;
	//cout<<"Q:\n"<<Q<<endl;

	R = pow(s,2) * MatrixXd::Identity(2,2);
	//cout<<"R:\n"<<R<<endl;

	x0 = VectorXd::Zero(4);
	x0(2) = -1;
	x0(3) = 1;
	//cout<<"m0:\n"<<m0<<endl;

	P0 = MatrixXd::Identity(4,4);
	//cout<<"P0:\n"<<P0<<endl;

	kf = KalmanFilter(dt,A,H,Q,R,P0);
	kf.init(0,x0);
}

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
		x = A*x + q;
		y = H*x + s*MatrixXd::Random(2,1);
		//cout<<x<<endl;
		//cout<<endl;
		//cout<<y<<endl;
		X.col(i) = x;
		Y.col(i) = y;
	}
	//cout<<X<<endl;
	//cout<<endl;
	//cout<<Y<<endl;
}

void estimate(){
	VectorXd yCurr;
	for(int i=0;i<steps;i++){
		yCurr = Y.col(i);
		cout<<"Inputs: "<<yCurr.transpose()<<" Estimation: "<<kf.update(Y.col(i)).transpose()<<endl;
	}
}

int main() {
	initialize();
	simulateData();
	estimate();
}
