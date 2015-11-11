#include <iostream>
#include "kalman.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter(
    double dt,
    const MatrixXd& A,
    const MatrixXd& H,
    const MatrixXd& Q,
    const MatrixXd& R,
    const MatrixXd& P)
  : A(A), H(H), Q(Q), R(R), P0(P),
    ATranspose(A.transpose()),HTranspose(H.transpose()),
    m(H.rows()), n(A.rows()), dt(dt), initialized(false),
    x_hat(n), x_hat_new(n)
{}

KalmanFilter::KalmanFilter() {}

void KalmanFilter::init(double t0, const Eigen::VectorXd& x0) {
  x_hat = x0;
  P = P0;
  this->t0 = t0;
  t = t0;
  initialized = true;
}

void KalmanFilter::init() {
  x_hat.setZero();
  P = P0;
  t0 = 0;
  t = t0;
  initialized = true;
}

VectorXd KalmanFilter::update(const VectorXd& y) {

  if(!initialized){
    std::cout<<"Filter is not initialized!"<<std::endl;
    return VectorXd::Zero(1);
  }

  x_hat_new = A * x_hat;
  P = A*P*ATranspose + Q;
  S = H*P*HTranspose + R;
  K = P*HTranspose*S.inverse();
  x_hat_new += K * (y - H*x_hat_new);
  P = P - K*S*K.transpose();
  x_hat = x_hat_new;

  t += dt;

  return x_hat;
}

VectorXd KalmanFilter::update(const VectorXd& y, double dt, const MatrixXd A) {
  this->A = A;
  this->dt = dt;
  return update(y);
}
