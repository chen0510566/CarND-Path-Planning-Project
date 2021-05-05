//
// Created by chen on 2021/5/2.
//

#include "sensor_fusion.h"

SensorFusion::SensorFusion() {

}

void SensorFusion::Predict(double delta_time) {
  for (auto &elem:map_vehicles_) {
    elem.second.Predict(delta_time);
  }
}

void SensorFusion::Update(const std::vector<SensorFusionData> &sensor_fusion_data,
                          double cur_s,
                          const WayPoints &map) {
  tracked_vehicles_.clear();

  for (const auto &vehicle: map_vehicles_) {
    tracked_vehicles_[vehicle.first] = false;
  }

  const auto count = sensor_fusion_data.size();

  for (const auto &sf:sensor_fusion_data) {
    const auto v = sqrt(sf.vx_ * sf.vx_ + sf.vy_ * sf.vy_);
    const Eigen::Vector2d sd = map.CalcFrenet(Point(sf.x_, sf.y_), sf.s_);
    double s = sd(0);

    const double max_s = map.MaxS();
    const double max_s2 = max_s / 2;

    while (cur_s - s > max_s2) {
      s += max_s;
    }

    while (cur_s - s < -max_s2) {
      s -= max_s;
    }

    auto it = map_vehicles_.find(sf.id_);
    if (it == map_vehicles_.end()) {
      map_vehicles_[sf.id_] = OtherVehicle(s, sd(1), v);
    } else {
      it->second.Update(s, sd(1), v);
    }

    tracked_vehicles_[sf.id_] = true;
  }

  //clear vheicles not tracked;
  for (const auto &vehicle:tracked_vehicles_) {
    if (!vehicle.second) {
      map_vehicles_.erase(vehicle.first);
    }
  }
}

TOtherVehiclesTrajectory SensorFusion::GetLeadingVehicleTrajectoryInLane(const Eigen::VectorXd &sdc_state_v6,
                                                                         int lane_num,
                                                                         double time_duration,
                                                                         double time_step) const {
  TOtherVehiclesTrajectory other_vehicles_trajectories;
  auto other_vehicles = GetLeadingVehiclesInLane(sdc_state_v6, lane_num);
  for (auto &vehicle:other_vehicles) {
    other_vehicles_trajectories.emplace_back(vehicle.PredictedTrajectory(time_step, time_duration));
  }
  return other_vehicles_trajectories;
}

TOtherVehiclesTrajectory SensorFusion::GetOtherVehicleTrajectoryInLane(const Eigen::VectorXd &sdc_state_v6,
                                                                       int lane_num,
                                                                       double time_duration,
                                                                       double time_step) const {
  TOtherVehiclesTrajectory other_vehicles_trajectories;
  auto other_vehicles = GetNearestVehiclesInLane(sdc_state_v6, lane_num);
  for (auto& vehicle:other_vehicles) {
    other_vehicles_trajectories.emplace_back(vehicle.PredictedTrajectory(time_step, time_duration));
  }
  return other_vehicles_trajectories;
}

TOtherVehicles SensorFusion::GetLeadingVehiclesInLane(const Eigen::VectorXd &sdc_state_v6,
                                                      int lane_num,
                                                      bool only_nearest) const {
  TOtherVehicles leading_vheicles;
  leading_vheicles.reserve(20);

  const auto s = sdc_state_v6(0);

  for (const auto &item:map_vehicles_) {
    const auto &vehicle = item.second;
    if (vehicle.GetS() > s && vehicle.IsInLane(lane_num)) {
      leading_vheicles.emplace_back(vehicle);
    }
  }

  std::sort(leading_vheicles.begin(),
            leading_vheicles.end(),
            [](const OtherVehicle &l, const OtherVehicle &r) { return l.GetS() < r.GetS(); });

  if (only_nearest) {
    leading_vheicles.resize(1);
  }
  return leading_vheicles;
}

TOtherVehicles SensorFusion::GetNearestVehiclesInLane(const Eigen::VectorXd &sdc_state_v6,
                                                      int lane_num,
                                                      double delta_s) const {
  TOtherVehicles leading_vehicles;
  const double s = sdc_state_v6(0);

  for (const auto &item:map_vehicles_) {
    const auto &vehicle = item.second;
    if (vehicle.IsInLane(lane_num) && distance(vehicle.GetS(), s) < delta_s) {
      leading_vehicles.push_back(vehicle);
    }
  }

  return leading_vehicles;
}