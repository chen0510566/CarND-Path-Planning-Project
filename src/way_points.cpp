//
// Created by chen on 2021/5/1.
//

#include <string>
#include <fstream>
#include <algorithm>
#include <iostream>
#include <glog/logging.h>
#include "way_points.h"

WayPoints::WayPoints() {
  const std::string file_path = "../data/highway_map.csv";
  std::ifstream in_map(file_path.c_str(), std::ifstream::in);

  WayPoint way_pt;
  std::string line;
  while (getline(in_map, line)) {
    std::istringstream iss(line);
    iss >> way_pt.x_;
    iss >> way_pt.y_;
    iss >> way_pt.s_;
    iss >> way_pt.d_x_;
    iss >> way_pt.d_y_;

    way_pts_.emplace_back(way_pt);
  }
  std::cout << "load " << way_pts_.size() << " points from " << file_path;

  FitSpline();
}

WayPoints::~WayPoints() {

}

void WayPoints::FitSpline() {
#if 1
  //------------------------------------
  // Fix of kink at the end of the track!
  WayPoint wpF = way_pts_.front();
  WayPoint wpL = way_pts_.back();

  WayPoint wp1 = wpL;
  wp1.s_ -= max_s_;

  WayPoint wp2 = wpF;
  wp2.s_ += max_s_;

  way_pts_.insert(way_pts_.begin(), wp1);
  way_pts_.push_back(wp2);
  //------------------------------------
#endif
  std::vector<double> x, y, s;
  x.resize(way_pts_.size());
  y.resize(way_pts_.size());
  s.resize(way_pts_.size());
  for (int i = 0; i < way_pts_.size(); ++i) {
    const auto &way_pt = way_pts_[i];
    x[i] = way_pt.x_;
    y[i] = way_pt.y_;
    s[i] = way_pt.s_;
  }
  x_spline_.set_points(s, x);
  y_spline_.set_points(s, y);
}

Eigen::Vector2d WayPoints::CalcFrenet(const Point &pt, double s_start) const {
  const double eps = 1.0e-6;
  const double gamma = 0.001;
  const double precision = 1e-12;
  double s = s_start;
  double delta_step = s;

  //采用梯度下降法在x,y样条曲线上查找距离pt最近的点的s
  while (delta_step> precision){
    const auto prev_s = s;
    s -= gamma* ErrorDeriv(pt, prev_s);
    delta_step = std::abs(s - prev_s);
  }

  Eigen::Vector2d p(2);
  p<<pt.x_, pt.y_;

  const Eigen::Vector2d spline_pt(x_spline_(s), y_spline_(s));
  const Eigen::Vector2d p_delta = (p - spline_pt).array()/ GetNoramlAt(s).array();

  const double d = 0.5*(p_delta(0) + p_delta(1));
  return Eigen::Vector2d(s, d);
}

double WayPoints::ErrorDeriv(const Point &pt, double s) const {
  return -2.0 * (pt.x_ - x_spline_(s)) * x_spline_.deriv(1, s) - 2.0 * (pt.y_ - y_spline_(s)) * y_spline_.deriv(1, s);
}

Point WayPoints::GetXYInterpolated(double s, double d) const {
  s = NormalizeS(s);

  Point pt(x_spline_(s), y_spline_(s));

  Point normal_vector(-y_spline_.deriv(1, s), x_spline_.deriv(1, s));
  return pt + normal_vector*d;
}

double WayPoints::NormalizeS(double s) const {
  return (s>max_s_) ? (s -= max_s_):(s<-max_s_?(s+=max_s_):s);
}

Eigen::Vector2d WayPoints::GetNoramlAt(double s) const {
  return Eigen::Vector2d(-y_spline_.deriv(1, s), x_spline_.deriv(1, s));
}