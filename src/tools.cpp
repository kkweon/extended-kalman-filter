#include <iostream>
#include "tools.h"
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  // rmse = <px, py, vx, vy>
  VectorXd rmse(4);
  rmse.setZero();

  if (estimations.size() != ground_truth.size() || estimations.size() < 1) {
    std::cout << estimations.size() << " != " << ground_truth.size() << "\n";
    return rmse;
  }

  for (auto i = 0; i < estimations.size(); ++i) {

    VectorXd temp = estimations[i] - ground_truth[i];
    temp = temp.array().square();
    rmse += temp;

  }

  rmse /= estimations.size();
  rmse = rmse.array().sqrt();

  return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state) {
  /**
    * Calculate a Jacobian here.
  */

  MatrixXd jacobian(3, 4);

  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);
  auto square = [](double val) { return std::pow(val, 2); };

  double c1 = square(px) + square(py);
  double c2 = sqrt(c1);
  double c3 = c1 * c2;
  double c4 = px * vy - py * vx;
  double c5 = py * vx - px * vy;
  //pre-compute a set of terms to avoid repeated calculation


  if (c1 < constant::eps) {
    c1 += constant::eps;
  }

  if (c2 < constant::eps) {
    c2 += constant::eps;
  }

  if (c3 < constant::eps) {
    c3 += constant::eps;
  }

  jacobian << px / c2, py / c2, 0, 0,
      -py / c1, px / c1, 0, 0,
      py * c5 / c3, px * c4 / c3, px / c2, py / c2;

  return jacobian;
}
