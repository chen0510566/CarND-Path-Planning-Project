//
// Created by chen on 2021/5/2.
//

#include "other_vehicle.h"

OtherVehicle::OtherVehicle(double s, double d, double vs) {
  state_ = Eigen::VectorXd::Zero(4);
  state_ << s, d, vs, 0.0;
}

Eigen::MatrixXd OtherVehicle::PredictedTrajectory(double delta_time, double duration) {
  const auto steps = static_cast<size_t>(duration / delta_time);
  Eigen::MatrixXd result(steps, 4);

  auto s = state_(0);
  for (int i = 0; i < steps; ++i) {
    s += state_(2) * delta_time;
    result.row(i) << s, state_(1), state_(2), 0.0;
  }
  return result;
}
