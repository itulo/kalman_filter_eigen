#include "Eigen/Dense"

#pragma once

using Eigen::MatrixXd;
using Eigen::VectorXd;

class KalmanFilter {

public:

  /**
  * Create a Kalman filter with the specified matrices.
  *   A - System dynamics matrix
  *   C - Output matrix
  *   Q - Process noise covariance
  *   R - Measurement noise covariance
  *   P - Estimate error covariance
  */
  KalmanFilter(
      double dt,
      const MatrixXd& A,
      const MatrixXd& H,
      const MatrixXd& Q,
      const MatrixXd& R,
      const MatrixXd& P
  );

  /**
  * Create a blank estimator.
  */
  KalmanFilter();

  /**
  * Initialize the filter with initial states as zero.
  */
  void init();

  /**
  * Initialize the filter with a guess for initial states.
  */
  void init(double t0, const VectorXd& x0);

  /**
  * Update the estimated state based on measured values. The
  * time step is assumed to remain constant.
  */
  VectorXd update(const VectorXd& y);

  /**
  * Update the estimated state based on measured values,
  * using the given time step and dynamics matrix.
  */
  VectorXd update(const VectorXd& y, double dt, const MatrixXd A);

  /**
  * Return the current state and time.
  */
  VectorXd state() { return x_hat; };
  double time() { return t; };

private:

  // Matrices for computation
  MatrixXd A, H, Q, R, P, K, P0, S,ATranspose, HTranspose;

  // System dimensions
  int m, n;

  // Initial and current time
  double t0, t;

  // Discrete time step
  double dt;

  // Is the filter initialized?
  bool initialized;

  // Estimated states
  VectorXd x_hat, x_hat_new;
};
