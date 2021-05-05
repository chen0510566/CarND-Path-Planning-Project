//
// Created by chen on 2021/5/1.
//

#ifndef PATH_PLANNING_SRC_TYPES_H_
#define PATH_PLANNING_SRC_TYPES_H_

#include <limits>
#include "../include/Eigen-3.3/Eigen/Eigen"
#include "../include/Eigen-3.3/Eigen/Core"

struct Point {
  double x_;
  double y_;

  Point(double x = 0, double y = 0) : x_(x), y_(y) {

  }

  Point &operator*(double n) {
    x_ *= n;
    y_ *= n;
    return *this;
  }

  Point &operator+(const Point &r) {
    if (this != &r) {
      x_ += r.x_;
      y_ += r.y_;
      return *this;
    }
  }
};

struct WayPoint {
  double x_;
  double y_;
  double s_;
  double d_x_;
  double d_y_;

  WayPoint(double x = 0.0, double y = 0.0, double s = 0.0, double d_x = 0.0, double d_y = 0.0) :
      x_(x), y_(y), s_(s), d_x_(d_x), d_y_(d_y) {
  }
};

struct CarLocalizationData {
  double x_;
  double y_;
  double s_;
  double d_;
  double yaw_;
  double speed_;

  CarLocalizationData(double x = 0.0, double y = 0.0, double s = 0.0, double d = 0.0,
                      double yaw = 0.0, double speed = 0.0) : x_(x), y_(y), s_(s), d_(d), yaw_(yaw), speed_(speed) {

  }
};

struct SensorFusionData {
  int id_;
  double x_;
  double y_;
  double vx_;
  double vy_;
  double s_;
  double d_;
};
#endif //PATH_PLANNING_SRC_TYPES_H_
