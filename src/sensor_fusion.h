//
// Created by chen on 2021/5/2.
//

#ifndef PATH_PLANNING_SRC_SENSOR_FUSION_H_
#define PATH_PLANNING_SRC_SENSOR_FUSION_H_

#include <unordered_map>
#include <vector>
#include "way_points.h"
#include "types.h"
#include "other_vehicle.h"

class SensorFusion {
 public:
  SensorFusion();

  void Predict(double delta_time);

  void Update(const std::vector<SensorFusionData> &sensor_fusion_data, double cur_s, const WayPoints &map);

  /**@brief
   * get other leading vehicles in the lane which is defined by SDC vehicle state;
   * @param sdc_state_v6
   * @param lane_num
   * @param time_duration
   * @param time_step
   * @return
   */
  TOtherVehiclesTrajectory GetLeadingVehicleTrajectoryInLane(const Eigen::VectorXd &sdc_state_v6,
                                                             int lane_num,
                                                             double time_duration,
                                                             double time_step) const;

  TOtherVehiclesTrajectory GetOtherVehicleTrajectoryInLane(const Eigen::VectorXd &sdc_state_v6,
                                                           int lane_num,
                                                           double time_duration,
                                                           double time_step) const;

  /**@brief
   * get front vehicles;
   * @param sdc_state_v6
   * @param lane_num
   * @param only_nearest
   * @return
   */
  TOtherVehicles GetLeadingVehiclesInLane(const Eigen::VectorXd& sdc_state_v6, int lane_num, bool only_nearest = false) const;

  /**@brief
   * get all vehicles in the lane which are close to sdc s position, [sdc(0)-delta_s, sdc(0)+delta_s];
   * @param sdc_state_v6
   * @param lane_num
   * @param delta_s
   * @return
   */
  TOtherVehicles GetNearestVehiclesInLane(const Eigen::VectorXd& sdc_state_v6, int lane_num, double delta_s = 200) const;
 private:
  std::unordered_map<int, bool> tracked_vehicles_;
  std::unordered_map<int, OtherVehicle> map_vehicles_;
};

#endif //PATH_PLANNING_SRC_SENSOR_FUSION_H_
