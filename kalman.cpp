#include <iostream>
#include "kalman.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter(
    const MatrixXd& A,
    const MatrixXd& H,
    const MatrixXd& Q,
    const MatrixXd& R,
    const MatrixXd& P)
  : A(A), H(H), Q(Q), R(R), P0(P),
    ATranspose(A.transpose()),HTranspose(H.transpose()),
    m(H.rows()), n(A.rows()), initialized(false),
    x_hat(n), x_hat_new(n)
{}

KalmanFilter::KalmanFilter() {}

void KalmanFilter::init(const Eigen::VectorXd& x0) {
  x_hat = x0;
  P = P0;
  initialized = true;
}

void KalmanFilter::init() {
  x_hat.setZero();
  P = P0;
  initialized = true;
}

VectorXd KalmanFilter::update(const VectorXd& y) {

  if(!initialized)
    throw std::runtime_error("Filter is not initialized!");

  x_hat_new = A * x_hat;
  P = A*P*ATranspose + Q;
  S = H*P*HTranspose + R;
  K = P*HTranspose*S.inverse();
  x_hat_new += K * (y - H*x_hat_new);
  P = P - K*S*K.transpose();
  x_hat = x_hat_new;

  return x_hat;
}
