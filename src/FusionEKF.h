#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"
#include "constants.h"


class FusionEKF {
 public:
  /**
  * Constructor.
  */
  FusionEKF();

  /**
  * Destructor.
  */
  virtual ~FusionEKF();

  /**
  * Run the whole flow of the Kalman Filter from here.
  */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
   * Update `ekf_.F_` with `dt`
   * @param dt delta_t indicates how much time passes
   */
  void UpdateStateTransition(double dt);

  /**
   * Update `ekf_.Q_' with `noise_ax` and `noise_ay`
   * @param dt
   * @param noise_ax
   * @param noise_ay
   */
  void UpdateProcessNoiseMatrix(double dt, double noise_ax, double noise_ay);

  /**
   * Returns a delta_t in seconds
   *
   * @param previous_timestamp
   * @param current_timestamp
   * @return delta_t (in seconds)
   */
  double ComputeDeltaT(double previous_timestamp, double current_timestamp);

  /**
  * Kalman Filter update and prediction math lives in here.
  */
  KalmanFilter ekf_;

 private:
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long long previous_timestamp_;

  // tool object used to compute Jacobian and RMSE
  Tools tools;
  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;
  Eigen::MatrixXd H_laser_;
  Eigen::MatrixXd Hj_;

  const double noise_ax = 9;
  const double noise_ay = 9;
};

#endif /* FusionEKF_H_ */
