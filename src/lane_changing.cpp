//
// Created by chen on 2021/5/2.
//

#include "lane_changing.h"

LaneChanging::LaneChanging(int start_lane_num, int target_lane_num)
    : start_lane_num_(start_lane_num), target_lane_num_(target_lane_num) {

}

LaneChanging::~LaneChanging() noexcept {

}

std::tuple<VehicleState *, TrajectoryPtr> LaneChanging::OptimalTrajectory(const Eigen::VectorXd &cur_state_x6,
                                                                          double cur_time,
                                                                          const SensorFusion &sensor_fusion) {
  const int max_T = 10.0;

  const double current_d = cur_state_x6(3);
  const double start_d = LaneNumbertoD(start_lane_num_);
  const double target_d = LaneNumbertoD(target_lane_num_);
  const double changing_lane_time = 4.0;//换到目标车道的时间（横向）
  const int cur_lane_num = DtoLaneNumber(cur_state_x6(3));
  const double max_velocity_corrected = GetCorrectedVelocity(speed_limit_, cur_lane_num);

  TrajectoryPool traj_pool(max_velocity_corrected, horizon_prediton_);

  //获取自车当前车道前方的车辆
  traj_pool.SetOtherVehicles(sensor_fusion.GetLeadingVehicleTrajectoryInLane(cur_state_x6, start_lane_num_, horizon_prediton_, time_step_));

  //获取目标车道内附近的车辆
  traj_pool.AddOtherVehicles(sensor_fusion.GetOtherVehicleTrajectoryInLane(cur_state_x6, target_lane_num_, horizon_prediton_, time_step_));

  //设定不同的目标速度以及到达目标速度的时间
  for (double v=0; v<speed_limit_; v+= 1.0) {
    for (double T=1; T<max_T; T+=1) {
      traj_pool.AddTrajectory(Trajectory::VelocityKeepSTrajectory(cur_state_x6, target_d, v, cur_time, T, changing_lane_time));
    }
  }

  VehicleState* next_state = this;
  TrajectoryPtr  optimial_traj = traj_pool.GetOptimalTrajectory();

  if(std::abs(target_d-current_d)<0.1){
    next_state = nullptr;
  }

  return std::tuple<VehicleState*, TrajectoryPtr>(next_state, optimial_traj);
}