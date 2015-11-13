#include <limits.h>
#include "gtest/gtest.h"
#include "Eigen/Dense"
#include "kalman.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

int steps = 100;

// Check the implementation of the Kalman Filter throws an exception when it's supposed to
TEST(KalmanFilter, ThrowsExceptionIfNotInitialized) {
	KalmanFilter kf = KalmanFilter();
	
 	EXPECT_THROW(
 			kf.update(VectorXd::Ones(1)),
 			std::runtime_error);
}

// Single-variable example from http://greg.czerniak.info/guides/kalman1/
TEST(KalmanFilter, SingleVariable) {
	double noisyY;

	KalmanFilter kf = KalmanFilter(	VectorXd::Ones(1),
									VectorXd::Ones(1),
									VectorXd::Ones(1)*0.00001,
									VectorXd::Ones(1)*0.1,
									VectorXd::Ones(1)
								  );
	kf.init(VectorXd::Ones(1)*3);

	for(int i=0;i<steps-1;i++){
		noisyY = 1 + ((double) rand() / RAND_MAX) * (i%2==0 ? 1 : -1);
		kf.update(VectorXd::Ones(1)*noisyY);
	}
	VectorXd last = kf.update(VectorXd::Ones(1)*noisyY);

 	EXPECT_LT(noisyY-last(0), 1);	// expect difference between last estimation and input to be less than 1
}
