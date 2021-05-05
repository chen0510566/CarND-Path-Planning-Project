//
// Created by chen on 2021/5/2.
//

#include "trajectory_pool.h"

#include <iostream>
#include <iostream>

TrajectoryPool::TrajectoryPool(double speed_limit, double time_horizon)
    : speed_limit_(speed_limit), horizon_prediction_(time_horizon) {
}

TrajectoryPool::~TrajectoryPool() {

}

void TrajectoryPool::SetOtherVehicles(const TOtherVehiclesTrajectory &other_trajectories) {
  other_trajectories_.insert(other_trajectories_.end(), other_trajectories.begin(), other_trajectories.end());
}

void TrajectoryPool::AddOtherVehicles(const TOtherVehiclesTrajectory &other_trajectories) {
  other_trajectories_.insert(other_trajectories_.end(), other_trajectories.begin(), other_trajectories.end());
}

void TrajectoryPool::AddTrajectory(TrajectoryPtr traj) {
  traj_pool_.emplace_back(traj);
  CalcTrajectoryCost(traj);
}

TrajectoryPtr TrajectoryPool::GetOptimalTrajectory() const {
  TrajectoryPtr best_traj;
  double min_cost = std::numeric_limits<double>::max();

  for (const auto &traj:traj_pool_) {
//    std::cout << "traj start :" << traj->GetStartS() << ", " << traj->GetStartD() << ", target:" << traj->GetTargetS()
//              << "," << traj->GetTargetD() << ", cost:" << traj->cost_.transpose() << ", " << traj->cost_.sum()
//              << std::endl;
    const auto cost = traj->GetTotalCost();
    if (cost < min_cost) {
      min_cost = cost;
      best_traj = traj;
    }
  }
  std::cout << "best start :" << best_traj->GetStartS() << ", " << best_traj->GetStartD() << ", target:"
            << best_traj->GetTargetS() << "," << best_traj->GetTargetD() << ", cost:" << best_traj->cost_.transpose()
            << ", " << best_traj->cost_.sum() << std::endl;

  return best_traj;
}

void TrajectoryPool::CalcTrajectoryCost(TrajectoryPtr traj) {
  double max_js, max_jd;

  double jerk_cost = jerk_cost_weight * traj->CalcJerkCost(horizon_prediction_, max_js, max_jd);
  double acc_cost = acc_cost_weight * traj->CalcAccelCost(horizon_prediction_);
  double lane_offset_cost = lane_offset_cost_weight * traj->CalcLaneOffsetCost(horizon_prediction_);

  const auto min_max_velocity = traj->MinMaxVelocity_S();
  auto velocity_cost = velocity_cost_weight_ * traj->CalcVelocityCost(speed_limit_, horizon_prediction_);

  //penalize invalid trajectories
  if (min_max_velocity.first < 0.0 || min_max_velocity.second > speed_limit_) {
    velocity_cost += 10000;
  }

  /**@bug max_jd compared to max jerk s*/
  if (std::abs(max_js) > max_jerks_ || std::abs(max_jd) > max_jerks_) {
    jerk_cost += 10000;
  }

  double max_safe_dist_cost = 0;
  for (const auto &other_traj : other_trajectories_) {
    const double safety_dist_cost = traj->CalcSafetyDistanceCost(other_traj, horizon_prediction_);
    max_safe_dist_cost = std::max(max_safe_dist_cost, safety_dist_cost);
  }

  double safety_dist_cost = safety_dist_cost_weight * max_safe_dist_cost;

  traj->SetJerkCost(jerk_cost);
  traj->SetAccelCost(acc_cost);
  traj->SetVelocityCost(velocity_cost);
  traj->SetSafetyDistanceCost(safety_dist_cost);
  traj->SetLaneOffsetCost(lane_offset_cost);
}


