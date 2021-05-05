//
// Created by chen on 2021/5/2.
//

#ifndef PATH_PLANNING_SRC_OTHER_VEHICLE_H_
#define PATH_PLANNING_SRC_OTHER_VEHICLE_H_

#include "utils.h"
#include "types.h"

class OtherVehicle {
 public:

 public:
  OtherVehicle(double s = 0.0, double d = 0.0, double vs = 0.0);

  double GetS() const {
    return state_(0);
  }

  double GetD() const {
    return state_(1);
  }

  double GetV() const {
    return state_(2);
  }

  int GetLane() const {
    return DtoLaneNumber(GetD());
  }

  bool IsInLane(int lane) const {
    return lane == GetLane() && lane != -1;
  }

  void Predict(double delta_time){
    state_(0) += state_(2)*delta_time;
  }

  void Update(double s, double d, double vs){
    state_<<s, d, vs, 0;
  }

  //generates trajectory as matrix of rows [s, d, vs, vd], number of rows = duration/delta_time;
  Eigen::MatrixXd PredictedTrajectory(double delta_time, double duration);
 private:
  Eigen::VectorXd state_;
};
using TOtherVehicles = std::vector<OtherVehicle>;
using TOtherVehiclesTrajectory = std::vector<Eigen::MatrixXd>;

#endif //PATH_PLANNING_SRC_OTHER_VEHICLE_H_
