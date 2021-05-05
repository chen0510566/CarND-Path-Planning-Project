//
// Created by chen on 2021/5/2.
//

#ifndef PATH_PLANNING_SRC_TRAJECTORY_POOL_H_
#define PATH_PLANNING_SRC_TRAJECTORY_POOL_H_

#include <deque>
#include <limits>
#include <memory>

#include "../include/Eigen-3.3/Eigen/Core"
#include "other_vehicle.h"
#include "trajectory.h"

class TrajectoryPool {
 public:
  using TPool = std::deque<TrajectoryPtr>;

  TrajectoryPool(double speed_limit, double time_horizon);

  ~TrajectoryPool();

  void SetOtherVehicles(const TOtherVehiclesTrajectory &other_trajectories);

  void AddOtherVehicles(const TOtherVehiclesTrajectory &other_trajectories);

  void AddTrajectory(TrajectoryPtr traj);

  TrajectoryPtr GetOptimalTrajectory() const;

 public:
  TPool traj_pool_;
 private:
  TOtherVehiclesTrajectory other_trajectories_;

  double speed_limit_;
  double horizon_prediction_;

  constexpr static double max_jerks_ = 9;

  static constexpr double jerk_cost_weight = 0.2;
  static constexpr double acc_cost_weight = 1.0;
  static constexpr double velocity_cost_weight_ = 0.5;
  static constexpr double safety_dist_cost_weight = 1000;
  static constexpr double lane_offset_cost_weight = 0.5;
 private:
  void CalcTrajectoryCost(TrajectoryPtr traj);
};

#endif //PATH_PLANNING_SRC_TRAJECTORY_POOL_H_
