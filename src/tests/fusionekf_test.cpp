//
// Created by kkweon on 6/21/17.
//
#include "catch.hpp"
#include "FusionEKF.h"
#include <iostream>

SCENARIO("FusionEKF unit test", "[FusionEKF]") {

  GIVEN("a single EKF temp") {
    FusionEKF temp;

    WHEN("EKF initializes with LASER initial measurements") {
      MeasurementPackage initial_measurement;
      initial_measurement.sensor_type_ = MeasurementPackage::SensorType::LASER;
      // px, py initial measurements
      initial_measurement.raw_measurements_.setOnes(2);
      initial_measurement.raw_measurements_ *= 1000;

      THEN("EKF.x_ value has to be updated") {
        auto before = temp.ekf_.x_;
        temp.ProcessMeasurement(initial_measurement);
        auto after = temp.ekf_.x_;
        REQUIRE(before != after);
      }
    }

    WHEN("EKF initializes with RADAR initial measurements") {
      MeasurementPackage initial_measurement;
      initial_measurement.sensor_type_ = MeasurementPackage::SensorType::RADAR;
      // px, py initial measurements
      initial_measurement.raw_measurements_.setOnes(3);
      initial_measurement.raw_measurements_ *= 1000;

      THEN("EKF.x_ value has to be updated") {
        auto before = temp.ekf_.x_;
        temp.ProcessMeasurement(initial_measurement);
        auto after = temp.ekf_.x_;
        REQUIRE(before != after);
      }
    }

    WHEN("there are two measurements from LIDAR & LIDAR") {
      // INITIALIZE a lidar with p_x, p_y with some random values
      // temp.ProcessMeasurement(lidar);
      // INITIALIZE another lidar with p_x, p_y with some random values
      // temp.ProcesMeasurement(lidar);
      // REQUIRE no error
      auto x_before = temp.ekf_.x_;
      MeasurementPackage lidar;
      lidar.timestamp_ = std::time(0);
      lidar.sensor_type_ = MeasurementPackage::LASER;
      lidar.raw_measurements_.setZero(2);
      lidar.raw_measurements_ << 100, 200;

      temp.ProcessMeasurement(lidar);

      lidar.timestamp_ = std::time(0);
      lidar.sensor_type_ = MeasurementPackage::LASER;
      lidar.raw_measurements_ << 200, 300;

      temp.ProcessMeasurement(lidar);

      THEN("EKF.x_ has to be changed") {
        auto x_after = temp.ekf_.x_;
        REQUIRE(x_before != x_after);
      }
    }

    WHEN("there are two measurements from LIDAR and RADAR") {
      auto x_before = temp.ekf_.x_;
      MeasurementPackage lidar;
      MeasurementPackage radar;

      lidar.timestamp_ = std::time(0);
      lidar.sensor_type_ = MeasurementPackage::LASER;
      lidar.raw_measurements_.setZero(2);
      lidar.raw_measurements_ << 100, 100;
      temp.ProcessMeasurement(lidar);

      radar.timestamp_ = std::time(0);
      radar.sensor_type_ = MeasurementPackage::RADAR;
      radar.raw_measurements_.setZero(3);
      radar.raw_measurements_ << 1, 1, 1;

      temp.ProcessMeasurement(radar);

      THEN("EKF.x_ has to be updated") {
        auto x_after = temp.ekf_.x_;
        REQUIRE(x_after != x_before);
      }
    }

    WHEN("the same measurements of LIDAR are given for many times") {
      MeasurementPackage lidar;

      lidar.timestamp_ = std::time(0);
      lidar.sensor_type_ = MeasurementPackage::LASER;
      lidar.raw_measurements_.setZero(2);
      lidar.raw_measurements_ << 100, 200;

      REQUIRE(temp.ekf_.x_(0) != 100);
      REQUIRE(temp.ekf_.x_(1) != 200);

      for (int i = 0; i < 1000; ++i) {
        lidar.timestamp_++;
        temp.ProcessMeasurement(lidar);
      }

      THEN("EKF.x_ should be close to the LIDAR values") {
        VectorXd expected(4);
        expected << 100, 200, 0, 0;
        double diff = sqrt((expected - temp.ekf_.x_).squaredNorm());
        REQUIRE(diff < 1e-8);
      }
    }
  }
}
