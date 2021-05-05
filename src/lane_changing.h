//
// Created by chen on 2021/5/2.
//

#ifndef PATH_PLANNING_SRC_LANE_CHANGING_H_
#define PATH_PLANNING_SRC_LANE_CHANGING_H_

#include "vehicle_state.h"
#include "trajectory_pool.h"

class LaneChanging :public VehicleState{
 public:
  LaneChanging(int start_lane_num, int target_lane_num);

  ~LaneChanging();

  virtual std::tuple<VehicleState *, TrajectoryPtr> OptimalTrajectory(const Eigen::VectorXd &cur_state_x6, double cur_time, const SensorFusion &sensor_fusion) override;
 private:
  int start_lane_num_;
  int target_lane_num_;
};

#endif //PATH_PLANNING_SRC_LANE_CHANGING_H_
