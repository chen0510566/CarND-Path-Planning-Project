//
// Created by chen on 2021/5/2.
//

#include "lane_keeping.h"

LaneKeeping::LaneKeeping() {

}

std::tuple<VehicleState *, TrajectoryPtr> LaneKeeping::OptimalTrajectory(const Eigen::VectorXd &cur_state_x6,
                                                                         double cur_time,
                                                                         const SensorFusion &sensor_fusion) {
  bool keep_lane_on = true;
  bool changing_left_on = true;
  bool changing_right_on = true;
  bool changing_test_mode = false;

  const double max_T = 10.0;
  const double lane_changing_time = 4.0;

  const double current_s = cur_state_x6(0);
  const double current_d = cur_state_x6(3);
  const int current_lane_num = DtoLaneNumber(current_d);

  TrajectoryPool trajectory_pool(speed_limit_, horizon_prediton_);
  TOtherVehiclesTrajectory leading_vehicle_trajs =
      sensor_fusion.GetLeadingVehicleTrajectoryInLane(cur_state_x6, current_lane_num, horizon_prediton_, time_step_);

  //生成保持当前车道的轨迹
  if (keep_lane_on) {
    trajectory_pool.SetOtherVehicles(leading_vehicle_trajs);
    for (double v = 0.0; v < speed_limit_; v += 1.0) {
      for (double T = 1.0; T < max_T; T += 1.0) {
        trajectory_pool.AddTrajectory(Trajectory::VelocityKeepSTrajectory(cur_state_x6, current_d, v, cur_time, T, 0));
      }
    }
  }

  //generate trajectories to change to left lane
  if (current_lane_num <= 1 && changing_left_on) {
    const int target_lane_num = current_lane_num + 1;
    const double target_d = LaneNumbertoD(target_lane_num);

    if (!changing_test_mode) {
      trajectory_pool.SetOtherVehicles(leading_vehicle_trajs);
      trajectory_pool.AddOtherVehicles(sensor_fusion.GetOtherVehicleTrajectoryInLane(cur_state_x6,
                                                                                     target_lane_num,
                                                                                     horizon_prediton_,
                                                                                     time_step_));
    }

    for (double v = 0.0; v < speed_limit_; v += 1.0) {
      for (double T = 1.0; T < max_T; T += 1.0) {
        TrajectoryPtr  trajectory = Trajectory::VelocityKeepSTrajectory(cur_state_x6, target_d, v, cur_time, T, lane_changing_time);
        trajectory_pool.AddTrajectory(trajectory);
      }
    }
  }

  //generate trajectories to turn right
  if(current_lane_num>=1 && changing_right_on){
    const int target_lane_num = current_lane_num - 1;
    const double target_d = LaneNumbertoD(target_lane_num);

    if(!changing_test_mode){
      trajectory_pool.SetOtherVehicles(leading_vehicle_trajs);
      trajectory_pool.AddOtherVehicles(sensor_fusion.GetOtherVehicleTrajectoryInLane(cur_state_x6, target_lane_num, horizon_prediton_, time_step_));
    }

    for (double v=0.0; v<speed_limit_; v+=1.0) {
      for (double T=1.0; T<max_T; T+=1.0) {
        TrajectoryPtr trajectory_ptr = Trajectory::VelocityKeepSTrajectory(cur_state_x6, target_d, v, cur_time, T, lane_changing_time);
        trajectory_pool.AddTrajectory(trajectory_ptr);
      }
    }
  }

  VehicleState* next_state = this;
  TrajectoryPtr optimal_traj = trajectory_pool.GetOptimalTrajectory();
  std::cout<<trajectory_pool.traj_pool_.size()<<" candidate trajectories"<<std::endl;
  const auto target_d = optimal_traj->GetTargetD();

  if(current_d!=target_d){
    next_state = new LaneChanging(current_lane_num, DtoLaneNumber(target_d));
  }


  return std::tuple<VehicleState*, TrajectoryPtr>(next_state, optimal_traj);
}