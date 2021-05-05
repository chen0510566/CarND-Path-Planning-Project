//
// Created by chen on 2021/5/2.
//

#include "vehicle_state.h"

VehicleState::VehicleState() {

}

VehicleState::~VehicleState() {

}

double VehicleState::GetCorrectedVelocity(double velocity, int lane_number) const {
  //different lanes has different max speed;
  static const double k_max_speed_factors[3] = {0.95, 0.975, 1.0};
  return velocity * k_max_speed_factors[lane_number];
}