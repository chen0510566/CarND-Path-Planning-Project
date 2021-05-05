//
// Created by chen on 2021/5/2.
//

#include <iostream>
#include <memory>
#include "path_planner.h"
#include "lane_keeping.h"

PathPlanner::PathPlanner() {
  states_.reserve(2);
  states_.emplace_back(std::unique_ptr<LaneKeeping>(new LaneKeeping));
}

TrajectoryPtr PathPlanner::OptimalTrajectory(const Eigen::VectorXd &current_state_x6,
                                             double cur_time,
                                             const SensorFusion &sensor_fusion) {

  VehicleState* new_state_ptr = nullptr;
  TrajectoryPtr optimal_traj_ptr;
  VehicleState* cur_state = states_.back().get();

  std::tie(new_state_ptr, optimal_traj_ptr) = cur_state->OptimalTrajectory(current_state_x6, cur_time, sensor_fusion);

  if(!new_state_ptr){
    states_.pop_back();
  }else if(new_state_ptr!=cur_state){
    states_.emplace_back(VehicleStatePtr(new_state_ptr));
  }
  return optimal_traj_ptr;
}