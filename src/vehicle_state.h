//
// Created by chen on 2021/5/2.
//

#ifndef PATH_PLANNING_SRC_VEHICLE_STATE_H_
#define PATH_PLANNING_SRC_VEHICLE_STATE_H_

#include <vector>
#include <tuple>
#include "types.h"
#include "trajectory.h"
#include "sensor_fusion.h"
#include "other_vehicle.h"

class VehicleState {
 public:
  VehicleState();

  virtual ~VehicleState();

  virtual std::tuple<VehicleState*, TrajectoryPtr> OptimalTrajectory(const Eigen::VectorXd& cur_state_x6, double cur_time, const SensorFusion& sensor_fusion) = 0;

 protected:
  double GetCorrectedVelocity(double velocity, int lane_number) const;

 protected:
  constexpr static double speed_limit_ = (80-4)*0.44704;
  constexpr static double horizon_prediton_ = 10.0;//seconds
  constexpr static double time_step_ = 0.1;
};

#endif //PATH_PLANNING_SRC_VEHICLE_STATE_H_
