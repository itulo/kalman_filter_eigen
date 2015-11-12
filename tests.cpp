#include <limits.h>
#include "gtest/gtest.h"
#include "Eigen/Dense"
#include "kalman.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

int steps = 100;

// Single-variable example from http://greg.czerniak.info/guides/kalman1/
TEST(SingleVariable, One) {
	double dt = 0.1;
	double noisyY;

	KalmanFilter kf = KalmanFilter( dt,
									VectorXd::Ones(1),
									VectorXd::Ones(1),
									VectorXd::Ones(1)*0.00001,
									VectorXd::Ones(1)*0.1,
									VectorXd::Ones(1)
								  );
	kf.init(0, VectorXd::Ones(1)*3);

	for(int i=0;i<steps-1;i++){
		noisyY = 1 + ((double) rand() / RAND_MAX) * (i%2==0 ? 1 : -1);
		kf.update(VectorXd::Ones(1)*noisyY);
	}
	VectorXd last = kf.update(VectorXd::Ones(1)*noisyY);

 	EXPECT_LT(noisyY-last(0), 1);	// expect difference between last estimation and input to be less than 1
}
