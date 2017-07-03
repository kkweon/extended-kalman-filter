#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
   * Predict `x_` and `P_`
   */
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::UpdateLIDAR(const VectorXd &z) {
  /**
   * Update with Laser Measurements(LIDAR)
   *
   * Args:
   *    z: LIDAR measurements of <p_x, p_y>
   *
   * Notes:
   *    z: 2-D Array of shape (2, 1)
   *    H_: 2-D Array of shape (2, 4)
   *    P_: 2-D Array of shape (4, 4)
   */
  VectorXd y = z - H_ * x_;
  Update(y);
}

void KalmanFilter::UpdateRADAR(const VectorXd &z) {
  /**
   * Update with Radar Measurements
   *
   * Notes:
   *    H_ (2-D Array): Jacobian matrix of shape (3, 4)
   */
  auto square = [](double value) { return std::pow(value, 2); };
  VectorXd H_x(3);
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);

  if (fabs(px) < constant::eps || std::sqrt(square(px) + square(py)) < constant::eps) {
    // No update when zero division
    return;
  }

  H_x << std::sqrt(square(px) + square(py)),
      std::atan2(py, px),
      (px * vx + py * vy) / std::sqrt(square(px) + square(py));

  VectorXd y = z - H_x;

  // because range is between -pi and pi
  while (y(1) > M_PI) {
    y(1) -= M_PI;
  }

  while (y(1) < -M_PI) {
    y(1) += M_PI;
  }

  Update(y);
}

void KalmanFilter::Update(const Eigen::VectorXd &y) {
  /**
   * Common update step
   *
   * Args:
   *    y (1-D Array): shape (2,) LIDAR or (3,) RADAR
   *
   * Notes:
   *    H_ and R_ were updated accordingly in advance
   */

  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  MatrixXd I = MatrixXd::Identity(4, 4);

  x_ = x_ + K * y;
  P_ = (I - K * H_) * P_;

}
