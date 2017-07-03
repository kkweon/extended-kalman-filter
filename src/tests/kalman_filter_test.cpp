//
// Created by kkweon on 6/21/17.
//
#include "catch.hpp"
#include "kalman_filter.h"
#include "measurement_package.h"

using namespace Eigen;

/**
 * Initialize Kalman Filter with random values
 *
 * This is a helper method used for kalman_filter_test
 * @param kf Kalman Filter
 */
void InitializeKalmanFilter(KalmanFilter &kf) {
  double dt = 1.0;
  double noise_ax = 9;
  double noise_ay = 9;

  VectorXd x_in(4);
  x_in << 1, 2, 3, 4;
  MatrixXd P_in(4, 4);
  P_in.setIdentity();
  P_in *= 1000;

  MatrixXd F_in(4, 4);
  F_in << 1, 0, dt, 0,
      0, 1, 0, dt,
      0, 0, 1, 0,
      0, 0, 0, 1;

  MatrixXd H_in(2, 4);
  H_in << 1, 0, 0, 0,
      0, 1, 0, 0;

  MatrixXd R_in(2, 2);
  R_in << 0.001, 0,
      0, 0.001;

  MatrixXd Q_in(4, 4);
  Q_in << pow(dt, 4) / 4 * noise_ax, 0, pow(dt, 3) / 2 * noise_ax, 0,
      0, pow(dt, 4) / 4 * noise_ay, 0, pow(dt, 4) / 2 * noise_ay,
      pow(dt, 3) / 2 * noise_ax, 0, pow(dt, 2) * noise_ax, 0,
      0, pow(dt, 3) / 2 * noise_ay, 0, pow(dt, 2) * noise_ay;

  kf.Init(x_in, P_in, F_in, H_in, R_in, Q_in);
}

TEST_CASE("Kalman Filter unit test", "[kalman_filter.h]") {
  SECTION("initialization") {
    KalmanFilter temp;
    REQUIRE(true);
  }

  SECTION("filter prediction") {
    KalmanFilter temp;
    InitializeKalmanFilter(temp);

    auto old_x = temp.x_;
    auto old_p = temp.P_;

    temp.Predict();

    auto new_x = temp.x_;
    auto new_p = temp.P_;

    REQUIRE(old_x != new_x);
    REQUIRE(old_p != new_p);
  }

  SECTION("LIDAR update") {
    KalmanFilter temp;
    InitializeKalmanFilter(temp);

    auto x_before = temp.x_;

    MeasurementPackage z;
    z.sensor_type_ = MeasurementPackage::LASER;
    z.timestamp_ = std::time(0);
    z.raw_measurements_.setZero(2);
    z.raw_measurements_ << 100, 100;
    temp.UpdateLIDAR(z.raw_measurements_);

    auto x_after = temp.x_;
    REQUIRE(x_after != x_before);
  }

  SECTION("finding the true position when LIDAR is given for multiple times") {
    // INITIALIZE a Kalman Filter, temp
    // DECLARE a LIDAR measurement z with p_x = 100, p_y = 100
    // count <- 0
    // REPEAT
    //     temp.UpdateLIDAR(z)
    //     count <- count + 1
    // UNTIL COUNT > many times
    // REQUIRE temp.x_ is close to (100, 100, 0, 0)
    KalmanFilter temp;
    InitializeKalmanFilter(temp);

    REQUIRE(temp.x_(0) != 100);
    REQUIRE(temp.x_(1) != 100);

    MeasurementPackage z;
    z.sensor_type_ = MeasurementPackage::LASER;

    for (int i = 0; i < 1000; ++i) {
      temp.Predict();
      z.timestamp_ = std::time(0);
      z.raw_measurements_.setZero(2);
      z.raw_measurements_ << 100, 100;
      temp.UpdateLIDAR(z.raw_measurements_);
    }

    VectorXd expected(4);
    expected << 100, 100, 0, 0;

    auto diff = sqrt((temp.x_ - expected).squaredNorm());
    REQUIRE(diff < 1e-8);

  }

}
