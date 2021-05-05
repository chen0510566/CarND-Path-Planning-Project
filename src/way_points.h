//
// Created by chen on 2021/5/1.
//

#ifndef PATH_PLANNING_SRC_WAY_POINTS_H_
#define PATH_PLANNING_SRC_WAY_POINTS_H_

#include <vector>
#include "../include/Eigen-3.3/Eigen/Core"
#include "spline.h"
#include "types.h"
class WayPoints {
 public:
 public:
  WayPoints();

  ~WayPoints();

  Point GetXYInterpolated(double s, double d) const;

  double MaxS() const {
    return max_s_;
  }
  Eigen::Vector2d  CalcFrenet(const Point& pt, double s_start) const;
 private:
  const double max_s_ = 6945.554;

  std::vector<WayPoint> way_pts_;

  tk::spline x_spline_;
  tk::spline y_spline_;
 private:
  void FitSpline();

  //calculates derivative d/ds of error function [(x-x_spline)^2 + (y-y_spline)^2]
  double ErrorDeriv(const Point& pt, double s) const;

  double NormalizeS(double s) const;

  Eigen::Vector2d GetNoramlAt(double s) const;
};

#endif //PATH_PLANNING_SRC_WAY_POINTS_H_
