#include "FusionEKF.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_.setZero(2, 2);
  R_radar_.setZero(3, 3);
  H_laser_.setZero(2, 4);
  Hj_.setZero(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
      0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
      0, 0.0009, 0,
      0, 0, 0.09;

  H_laser_ << 1, 0, 0, 0,
      0, 1, 0, 0;

  // All of these values will be re-initialized
  // Thus, I'm going to initialize with some garbage values.
  const double uncertain = 1000;
  const double certain = 0.01;
  VectorXd x_in(4);
  MatrixXd P_in(4, 4);
  P_in << certain, 0, 0, 0,
      0, certain, 0, 0,
      0, 0, uncertain, 0,
      0, 0, 0, uncertain;
  MatrixXd F_in(4, 4);
  MatrixXd Q_in(4, 4);

  ekf_.Init(x_in, P_in, F_in, H_laser_, R_laser_, Q_in);

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
     * IF sensor_type_ == RADAR
     *   CALCULATE coordinates (px, py)
     *   CALCULATE velocity (vx, vy)
     * ELSE
     *   px = measurements[0]
     *   py = measurements[1]
     *   vx, vy = 0, 0
     * ENDIF
     * ekf_.x_ = (px, py, vx, vy)
     * is_initiazlied = true
     * STORE timestamp
     */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_.setZero(4);

    double px, py, vx, vy;
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      double rho = measurement_pack.raw_measurements_[0];
      double phi = measurement_pack.raw_measurements_[1];
      double rho_dot = measurement_pack.raw_measurements_[2];

      px = rho * std::cos(phi);
      py = rho * std::sin(phi);

      vx = rho_dot * std::cos(phi);
      vy = rho_dot * std::sin(phi);

    } else {
      // LIDAR
      px = measurement_pack.raw_measurements_[0];
      py = measurement_pack.raw_measurements_[1];

      vx = 0;
      vy = 0;
    }
    if (fabs(px) < constant::eps) {
      px = constant::eps;
    }

    if (fabs(py) < constant::eps) {
      py = constant::eps;
    }

    ekf_.x_ << px, py, vx, vy;
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }
  /*****************************************************************************
   *  Prediction
   *
   * CALCULATE a time difference in seconds (dt)
   * UPDATE a state transition matrix `ekf.F_`
   * UPDATE a process noise matrix `ekf.Q_`
   * PREDICT `x_` and `P_` using Kalman Filter
   * previous_timestamp = measurement.timestamp
  ****************************************************************************/
  double dt = ComputeDeltaT(previous_timestamp_, measurement_pack.timestamp_);
  UpdateStateTransition(dt);
  UpdateProcessNoiseMatrix(dt, noise_ax, noise_ay);
  ekf_.Predict();
  previous_timestamp_ = measurement_pack.timestamp_;
  /*****************************************************************************
   *  Update
   *
   * IF measurement type == RADAR
   *   DO Extended Kalman Filter Update (Jacobian)
   * ELSE IF measurement type == LASER
   *   DO Regular Kalman Filter Update
   * ENDIF
  ****************************************************************************/
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    ekf_.UpdateRADAR(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.UpdateLIDAR(measurement_pack.raw_measurements_);
  }
  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}

void FusionEKF::UpdateStateTransition(double dt) {
  ekf_.F_ << 1, 0, dt, 0,
      0, 1, 0, dt,
      0, 0, 1, 0,
      0, 0, 0, 1;
}

void FusionEKF::UpdateProcessNoiseMatrix(double dt, double noise_ax, double noise_ay) {
  double dt_4 = std::pow(dt, 4) / 4.0;
  double dt_3 = std::pow(dt, 3) / 2.0;
  double dt_2 = std::pow(dt, 2);

  ekf_.Q_ << dt_4 * noise_ax, 0, dt_3 * noise_ax, 0,
      0, dt_4 * noise_ay, 0, dt_3 * noise_ay,
      dt_3 * noise_ax, 0, dt_2 * noise_ax, 0,
      0, dt_3 * noise_ay, 0, dt_2 * noise_ay;
}

double FusionEKF::ComputeDeltaT(double previous_timestamp, double current_timestamp) {
  return (current_timestamp - previous_timestamp) / 1'000'000.0;
}

