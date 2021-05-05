//
// Created by chen on 2021/5/2.
//

#ifndef PATH_PLANNING_SRC_PATH_PLANNER_H_
#define PATH_PLANNING_SRC_PATH_PLANNER_H_

#include <vector>
#include <memory>
#include "types.h"
#include "trajectory_pool.h"
#include "sensor_fusion.h"
#include "vehicle_state.h"

class PathPlanner {
 public:
  PathPlanner();

  TrajectoryPtr OptimalTrajectory(const Eigen::VectorXd &current_state_x6,
                                  double cur_time,
                                  const SensorFusion &sensor_fusion);

 private:
  using VehicleStatePtr = std::unique_ptr<VehicleState>;
  std::vector<VehicleStatePtr> states_;
};

#endif //PATH_PLANNING_SRC_PATH_PLANNER_H_
