//
// Created by chen on 2021/5/2.
//

#ifndef PATH_PLANNING_SRC_UTILS_H_
#define PATH_PLANNING_SRC_UTILS_H_

#include <vector>
#define _USE_MATH_DEFINES
#include <math.h>
#include <array>
#include <iostream>
#include "../include/Eigen-3.3/Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

constexpr double k_max_double_eval = 1.0e10;
constexpr double k_min_double_eval = -1.0e10;
constexpr double k_delta_t = 0.02;//0.02s between waypoints;
constexpr double k_lane_width = 4.0;

inline double distance(double x1, double x2) {
  return std::abs(x2 - x1);
}

inline int DtoLaneNumber(double d) {
  if (d > 0 || d < -12) {
    return -1;
  }

  if (d < -8) {
    return 0;
  } else if (d < -4) {
    return 1;
  }
  return 2;
}

inline double LaneNumbertoD(int lane_number) {
  constexpr std::array<double, 3> lane_centers = {-9.75, -6.0, -2.0};
  assert(lane_number >= 0 && lane_number < 3);
  return lane_centers[lane_number];
}

#endif //PATH_PLANNING_SRC_UTILS_H_
