//
// Created by chen on 2021/5/2.
//

#include "trajectory.h"

Trajectory::Trajectory()
    : time_start_(0.0),
      duration_s_(0.0),
      duration_d_(0.0),
      target_velocity_(0.0),
      start_state_(Eigen::VectorXd::Zero(6)),
      target_state_(Eigen::VectorXd::Zero(6)),
      S_coeffs_(Eigen::VectorXd::Zero(6)),
      D_coeffs_(Eigen::VectorXd::Zero(6)) {

}

Trajectory::Trajectory(const SDState &start_state,
                       const SDState &target_state,
                       double duration_s,
                       double duration_d,
                       double time_start)
    : time_start_(time_start),
      duration_s_(duration_s),
      duration_d_(duration_d),
      target_velocity_(0.0),
      start_state_(start_state),
      target_state_(target_state),
      S_coeffs_(Eigen::VectorXd::Zero(6)),
      D_coeffs_(Eigen::VectorXd::Zero(6)) {
  S_coeffs_ = CalcQuinticPolynomialCoeffs(start_state_.head(3), target_state_.head(3), duration_s_);
  D_coeffs_ = CalcQuinticPolynomialCoeffs(start_state_.segment(3, 3), target_state_.segment(3, 3), duration_d_);
}

TrajectoryPtr Trajectory::VelocityKeepSTrajectory(const SDState &cur_state,
                                                  double end_d,
                                                  double target_velocity,
                                                  double cur_time,
                                                  double time_duration_s,
                                                  double time_duration_d) {
  TrajectoryPtr traj_ptr = std::make_shared<Trajectory>();
  traj_ptr->time_start_ = cur_time;
  traj_ptr->duration_s_ = time_duration_s;

  traj_ptr->duration_d_ = time_duration_d;

  traj_ptr->start_state_ = cur_state;
  traj_ptr->target_state_(3) = end_d;
  traj_ptr->target_state_(4) = 0.0;
  traj_ptr->target_state_(5) = 0.0;
  traj_ptr->target_velocity_ = target_velocity;

  traj_ptr->S_coeffs_ = CalcSPolynomialVelocityKeeping(cur_state.head(3), target_velocity, time_duration_s);

  const double cur_d = cur_state(3);
  if (cur_d == end_d) {
    traj_ptr->D_coeffs_(0) = cur_state(3);
  } else {
    traj_ptr->D_coeffs_ = CalcQuinticPolynomialCoeffs(traj_ptr->start_state_.segment(3, 3),
                                                      traj_ptr->target_state_.segment(3, 3),
                                                      time_duration_d);
  }
  return traj_ptr;
}

SDState Trajectory::EvaluateStateAt(double time) const {
  double t = time - time_start_;
  VectorXd state_S = CalcPolynomialAt(S_coeffs_, std::min(t, duration_s_));
  VectorXd state_D = CalcPolynomialAt(D_coeffs_, std::min(t, duration_d_));

  //model with acceleration = 0.0;
  if (t > duration_s_) {
    state_S(0) += state_S(1) * (t - duration_s_);
  }

  if (t > duration_d_) {
    state_D(0) += state_D(1) * (t - duration_d_);
  }

  Eigen::VectorXd state(6);
  state << state_S, state_D;
  return state;
}

PolyCoeffs Trajectory::CalcQuinticPolynomialCoeffs(const SDState &start_state_x3,
                                                   const SDState &target_state_x3,
                                                   double T) {
  const auto s_i = start_state_x3[0];
  const auto s_i_d = start_state_x3[1];
  const auto s_i_dd = start_state_x3[2];

  const auto s_f = target_state_x3[0];
  const auto s_f_d = target_state_x3[1];
  const auto s_f_dd = target_state_x3[2];

  const auto T2 = T * T;
  const auto T3 = T2 * T;
  const auto T4 = T3 * T;
  const auto T5 = T4 * T;

  Eigen::MatrixXd A = Eigen::MatrixXd(3, 3);
  A << T3, T4, T5, 3 * T2, 4 * T3, 5 * T4, 6 * T, 12 * T2, 20 * T3;

  Eigen::VectorXd b = Eigen::VectorXd(3);
  b << s_f - (s_i + s_i_d * T + 0.5 * s_i_dd * T2), s_f_d - (s_i_d + s_i_dd * T), s_f_dd - s_i_dd;

  Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);
  Eigen::VectorXd coeffs = Eigen::VectorXd(6);
  coeffs << s_i, s_i_d, 0.5 * s_i_dd, x[0], x[1], x[2];
  return coeffs;
}

Eigen::VectorXd Trajectory::CalcPolynomialAt(const PolyCoeffs &coeffs_x6, double t) {
  //distance: s(t) = a0 + a1 * t + a2 * t^2 + a3 * t^3 + a4 * t^4 + a5 * t^5
  //velocity: s_d(t) = a1 + 2*a2 * t + 3*a3 * t^2 + 4*a4 * t^3 + 5*a5 * t^4
  //acceleration: s_dd(t) = 2*a2 + 6*a3 * t^1 + 12*a4 * t^2 + 20*a5 * t^3

  Eigen::VectorXd state = Eigen::VectorXd::Zero(3);
  const auto t2 = t * t;
  const auto t3 = t2 * t;
  const auto t4 = t3 * t;
  const auto t5 = t4 * t;
  state(0) = coeffs_x6(0) + coeffs_x6(1) * t + coeffs_x6(2) * t2 + coeffs_x6(3) * t3 + coeffs_x6(4) * t4
      + coeffs_x6(5) * t5;
  state(1) =
      coeffs_x6(1) + 2 * coeffs_x6(2) * t + 3 * coeffs_x6(3) * t2 + 4 * coeffs_x6(4) * t3 + 5 * coeffs_x6(5) * t4;
  state(2) = 2 * coeffs_x6(2) + 6 * coeffs_x6(3) * t + 12 * coeffs_x6(4) * t2 + 20 * coeffs_x6(5) * t3;
  return state;
}

PolyCoeffs Trajectory::CalcSPolynomialVelocityKeeping(const Eigen::VectorXd &start_state_v3,
                                                      double target_velocity,
                                                      double T) {
  const double T2 = T * T;
  const double T3 = T2 * T;
  Eigen::MatrixXd A(2, 2);
  A << 3 * T2, 4 * T3, 6 * T, 12 * T2;

  Eigen::VectorXd b(2);
  b << target_velocity - (start_state_v3(1) + start_state_v3(2) * T), 0.0 - start_state_v3(2);

  Eigen::Vector2d x = A.colPivHouseholderQr().solve(b);

  VectorXd S_coeffs(6);
  S_coeffs << start_state_v3(0), start_state_v3(1), 0.5 * start_state_v3(2), x, 0.0;
  return S_coeffs;
}

double Trajectory::CalcJerkCost(double time_duration, double &max_js, double max_jd) const {
  max_js = k_min_double_eval;
  max_jd = k_min_double_eval;

  double js_integral = 0.0;
  double jd_integral = 0.0;

  const double weight_s = 1.0;
  const double weight_d = 1.0;

  const int pts_num = static_cast<int>(time_duration / cost_dt_);

  for (double t = 0.0; t < time_duration; t += cost_dt_) {
    const double js = CalcPolynomialJerkAt(S_coeffs_, std::min(t, duration_s_));
    const double jd = CalcPolynomialJerkAt(D_coeffs_, std::min(t, duration_d_));

    js_integral += js * js;
    jd_integral += jd * jd;

    max_js = std::max(js, max_js);
    max_jd = std::max(jd, max_jd);
  }

  return (weight_s * js_integral + weight_d * jd_integral) / pts_num;
}

double Trajectory::CalcAccelCost(double time_duration) const {
  double cost = 0.0;
  const double pts_num = time_duration / cost_dt_;

  for (double t = 0.0; t < time_duration; t += cost_dt_) {
    const double as = CalcPolynomialAccelAt(S_coeffs_, std::min(t, duration_s_));
    const double ad = CalcPolynomialAccelAt(D_coeffs_, std::min(t, duration_d_));

    cost += as * as + ad * ad;
  }
  return cost / pts_num;
}

std::pair<double, double> Trajectory::MinMaxVelocity_S() const {
  std::pair<double, double> min_max_vel(k_max_double_eval, k_min_double_eval);
  for (double t = 0.0; t < duration_s_; t += delta_t) {
    const double v = CalcPolynomialVelocityAt(S_coeffs_, t);
    min_max_vel.first = std::min(v, min_max_vel.first);
    min_max_vel.second = std::max(v, min_max_vel.second);
  }
  return min_max_vel;
}

double Trajectory::CalcVelocityCost(double target_velocity, double time_duration) const {
  double cost = 0.0;
  double t = 0.0;
  const int pts_num = static_cast<int>(time_duration / cost_dt_);

  for (int i = 0; i < pts_num; ++i) {
    const double vs = CalcPolynomialVelocityAt(S_coeffs_, std::min(t, duration_s_));
    const double vd = CalcPolynomialVelocityAt(D_coeffs_, std::min(t, duration_d_));

    double v = std::sqrt(vs * vs + vd * vd);

    if (v >= target_velocity) {
      v = 1e6;
    }

    cost += std::pow(target_velocity - v, 2.0);
    t += cost_dt_;
  }
  return cost / pts_num;
}

double Trajectory::CalcSafetyDistanceCost(const Eigen::MatrixXd &s2, double time_duration) const {
  double cost = 0;
  double t = time_start_;
  const int pts_num = static_cast<int>(time_duration / cost_dt_);

  for (int i = 0; i < pts_num; ++i) {
    const Eigen::VectorXd s1 = EvaluateStateAt(t);
    const double S_dist = distance(s1(0), s2(i, 0));
    const double D_dist = distance(s1(3), s2(i, 1));

    double velocity = s1(1);
    cost += CalcSafetyDistanceCost(S_dist, D_dist, velocity);
    t += cost_dt_;
  }
  return cost / pts_num;
}

static double CostFunction(double dist, double min_dist, double safety_dist, double koeff = 2){
  const double a = -koeff;
  const double eMD = std::exp(a*min_dist);
  const double eSD = std::exp(a*safety_dist);
  const double eD = std::exp(a*dist);

  return (0.95*eD + 0.05*eMD - eSD)/(eMD - eSD);
}

inline double CalcLongitudialDistCost(double dist, double velocity){
  const double long_min_dist = 10;
  const double long_safety_dist = 1.2*velocity;
  if(dist>long_safety_dist){
    return 0.0;
  }

  return CostFunction(dist, long_min_dist, long_safety_dist);
}

inline double CalcLateralDistanceCost(double dist){
  const double lat_min_dist = 2.0;
  const double lat_safety_dist = 3.8;
  if(dist>lat_safety_dist){
    return 0.0;
  }
  return CostFunction(dist, lat_min_dist, lat_safety_dist);
}

double Trajectory::CalcSafetyDistanceCost(double S_dist, double D_dist, double velocity) const {
  double longitudial_cost = CalcLongitudialDistCost(S_dist, velocity);
  double lateral_cost = CalcLateralDistanceCost(D_dist);

  //other vehicle in the next lane;
  if(D_dist>3.8){
    return 0.0;
  }

  if(S_dist<4.5){
    if(D_dist>2){
      return lateral_cost;
    }else{
      //already crashed;
      return 1000;
    }
  }

  return longitudial_cost;
}

inline double CalcLaneOffsetCostbyDist(double d){
  const int lane_number = DtoLaneNumber(d);
  if(lane_number==-1){
    return 1e6;
  }

  const double lane_center = LaneNumbertoD(lane_number);
  const double dist_to_center = distance(d, lane_center);
  const double min_dist = k_lane_width/2.0;
  const double safety_dist = 0.25;
  return CostFunction(dist_to_center, min_dist, safety_dist);
}

double Trajectory::CalcLaneOffsetCost(double time_duration) const {
  double cost = 0.0;
  double t = time_start_;

  const int pts_num = static_cast<int>(time_duration/cost_dt_);

  for (int i = 0; i < pts_num; ++i) {
    const double d = CalcPolynomialDistanceAt(D_coeffs_, std::min(t, duration_d_));
    cost += CalcLaneOffsetCostbyDist(d);

    t+= cost_dt_;
  }
  return cost/pts_num;
}

double Trajectory::CalcPolynomialJerkAt(const PolyCoeffs &coeffs_x6, double t) const {
  //distance: s(t) = a0 + a1 * t + a2 * t^2 + a3 * t^3 + a4 * t^4 + a5 * t^5
  //velocity: s_d(t) = a1 + 2*a2 * t + 3*a3 * t^2 + 4*a4 * t^3 + 5*a5 * t^4
  //acceleration: s_dd(t) = 2*a2 + 6*a3 * t^1 + 12*a4 * t^2 + 20*a5 * t^3
  //jerk: s_ddd(t) = 6*a3 + 24*a4*t + 60*a5*t^2

  //calculate s_ddd(t)
  return 6 * coeffs_x6(3) + 24 * coeffs_x6(4) * t + 60 * coeffs_x6(5) * t * t;
}

double Trajectory::CalcPolynomialAccelAt(const PolyCoeffs &coeffs_x6, double t) const {
  //distance: s(t) = a0 + a1 * t + a2 * t^2 + a3 * t^3 + a4 * t^4 + a5 * t^5
  //velocity: s_d(t) = a1 + 2*a2 * t + 3*a3 * t^2 + 4*a4 * t^3 + 5*a5 * t^4
  //acceleration: s_dd(t) = 2*a2 + 6*a3 * t^1 + 12*a4 * t^2 + 20*a5 * t^3
  //jerk: s_ddd(t) = 6*a3 + 24*a4*t + 60*a5*t^2

  const auto t2 = t * t;
  const auto t3 = t2 * t;

  // calculate s_dd(t)
  return 2 * coeffs_x6(2) + 6 * coeffs_x6(3) * t + 12 * coeffs_x6(4) * t2 + 20 * coeffs_x6(5) * t3;
}

double Trajectory::CalcPolynomialVelocityAt(const PolyCoeffs &coeffs_x6, double t) const {
  //distance: s(t) = a0 + a1 * t + a2 * t^2 + a3 * t^3 + a4 * t^4 + a5 * t^5
  //velocity: s_d(t) = a1 + 2*a2 * t + 3*a3 * t^2 + 4*a4 * t^3 + 5*a5 * t^4
  //acceleration: s_dd(t) = 2*a2 + 6*a3 * t^1 + 12*a4 * t^2 + 20*a5 * t^3
  //jerk: s_ddd(t) = 6*a3 + 24*a4*t + 60*a5*t^2

  const auto t2 = t * t;
  const auto t3 = t2 * t;
  const auto t4 = t3 * t;

  //calculate s_d(t)
  return coeffs_x6(1) + 2 * coeffs_x6(2) * t + 3 * coeffs_x6(3) * t2 + 4 * coeffs_x6(4) * t3 + 5 * coeffs_x6(5) * t4;
}

double Trajectory::CalcPolynomialDistanceAt(const PolyCoeffs &coeffs_x6, double t) const {
  //distance: s(t) = a0 + a1 * t + a2 * t^2 + a3 * t^3 + a4 * t^4 + a5 * t^5
  //velocity: s_d(t) = a1 + 2*a2 * t + 3*a3 * t^2 + 4*a4 * t^3 + 5*a5 * t^4
  //acceleration: s_dd(t) = 2*a2 + 6*a3 * t^1 + 12*a4 * t^2 + 20*a5 * t^3
  //jerk: s_ddd(t) = 6*a3 + 24*a4*t + 60*a5*t^2

  const auto t2 = t * t;
  const auto t3 = t2 * t;
  const auto t4 = t3 * t;
  const auto t5 = t4 * t;

  //calculate s(t)
  return coeffs_x6(0) + coeffs_x6(1) * t + coeffs_x6(2) * t2 + coeffs_x6(3) * t3 + coeffs_x6(4) * t4
      + coeffs_x6(5) * t5;
}



