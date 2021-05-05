//
// Created by chen on 2021/5/2.
//

#ifndef PATH_PLANNING_SRC_LANE_KEEPING_H_
#define PATH_PLANNING_SRC_LANE_KEEPING_H_

#include <iostream>
#include "vehicle_state.h"
#include "types.h"
#include "trajectory_pool.h"
#include "lane_changing.h"

class LaneKeeping : public VehicleState {
 public:
  LaneKeeping();

  virtual std::tuple<VehicleState *, TrajectoryPtr> OptimalTrajectory(const Eigen::VectorXd &cur_state_x6,
                                                                      double cur_time,
                                                                      const SensorFusion &sensor_fusion) override;
};

#endif //PATH_PLANNING_SRC_LANE_KEEPING_H_
