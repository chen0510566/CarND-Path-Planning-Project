//
// Created by chen on 2021/5/1.
//

#ifndef PATH_PLANNING_SRC_VEHICLE_H_
#define PATH_PLANNING_SRC_VEHICLE_H_

#include <iostream>
#include "types.h"
#include "way_points.h"
#include "utils.h"
#include "sensor_fusion.h"
#include "path_planner.h"
class Vehicle {
 public:
 public:
  Vehicle(const WayPoints& wps);

  ~Vehicle();

  std::vector<SensorFusionData>& SensorFusionStorage();

  void UpdateTrajectory(const CarLocalizationData& new_state, const std::vector<double>& prev_path_x, const std::vector<double>& prev_path_y);

  std::vector<double>& GetNextXValues();

  std::vector<double>& GetNextYValues();
 private:
  const WayPoints& way_points_;

  std::vector<SensorFusionData> sensor_fusion_data_;

  std::vector<double> next_x_vals_;
  std::vector<double> next_y_vals_;

  bool is_init_done_;
  Eigen::VectorXd cur_state_;

  double cur_time_ = 0.0;
  //active prediction path size is pred - react;
  const double predict_horizon_t_ = 2.0;// prediction horizon time, in secs;
  const double react_horizon_t_ = 1.0;//reactive horizon time, in secs;

  SensorFusion sensor_fusion_;
  PathPlanner path_planner_;

};

#endif //PATH_PLANNING_SRC_VEHICLE_H_
