//
// Created by chen on 2021/5/1.
//

#include "vehicle.h"

Vehicle::Vehicle(const WayPoints &wps):way_points_(wps), is_init_done_(false) {
  cur_state_ = Eigen::VectorXd::Zero(6);
}

Vehicle::~Vehicle() {

}

std::vector<SensorFusionData> & Vehicle::SensorFusionStorage() {
  return sensor_fusion_data_;
}

std::vector<double> & Vehicle::GetNextXValues() {
  return next_x_vals_;
}

std::vector<double> & Vehicle::GetNextYValues() {
  return next_y_vals_;
}

void Vehicle::UpdateTrajectory(const CarLocalizationData &new_state,
                               const std::vector<double> &prev_path_x,
                               const std::vector<double> &prev_path_y) {
  const int prediction_path_size = int(predict_horizon_t_/0.02);
  const int reactive_path_size = int(react_horizon_t_/0.02);

  next_x_vals_.clear();
  next_y_vals_.clear();

  if(!is_init_done_){
    const double velocity = new_state.speed_*0.44704;//mph to m/s;
    cur_time_ = 0.0;

    //x,y->s,d
    const Eigen::Vector2d& initial_frenet = way_points_.CalcFrenet(Point(new_state.x_, new_state.y_), new_state.s_);
    cur_state_<<initial_frenet(0), velocity, 0.0, initial_frenet(1), 0.0, 0.0 ;
    is_init_done_ = true;
  }

  const auto prev_path_size = static_cast<int>(prev_path_x.size());
  int prev_pred_path_size = prev_path_size - (prediction_path_size - reactive_path_size);
  prev_pred_path_size = std::max(0, prev_pred_path_size);
  for (int i = 0; i < prev_pred_path_size; ++i) {
    next_x_vals_.emplace_back(prev_path_x[i]);
    next_y_vals_.emplace_back(prev_path_y[i]);
  }

  //predict position of vehicles after delay_time;
  const double delay_time = prev_pred_path_size*k_delta_t;
  sensor_fusion_.Update(sensor_fusion_data_, cur_state_(0), way_points_);
  sensor_fusion_.Predict(delay_time);


  std::cout<<"current time:"<<cur_time_<<std::endl;
  TrajectoryPtr optimal_traj = path_planner_.OptimalTrajectory(cur_state_, cur_time_, sensor_fusion_);

  double cur_time = cur_time_;
  Eigen::VectorXd cur_state = cur_state_;

  std::cout<<"new size:"<<prediction_path_size - prev_pred_path_size<<", time:"<<(prediction_path_size-prev_pred_path_size)*0.02<<std::endl;
  std::cout<<"1st:"<<cur_state.transpose()<<std::endl;
  for (int i = prev_pred_path_size; i < prediction_path_size; ++i) {
    if(i==reactive_path_size) {
      cur_time_ = cur_time;
      cur_state_ = cur_state;
    }

    const auto pt = way_points_.GetXYInterpolated(cur_state(0), cur_state(3));
    cur_time += k_delta_t;
    cur_state = optimal_traj->EvaluateStateAt(cur_time);

    next_x_vals_.emplace_back(pt.x_);
    next_y_vals_.emplace_back(pt.y_);
  }
  std::cout<<"last:"<<cur_state.transpose()<<std::endl;
  std::cout<<"new current time:"<<cur_time_<<std::endl;


}