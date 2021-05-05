//
// Created by chen on 2021/5/2.
//

#ifndef PATH_PLANNING_SRC_TRAJECTORY_H_
#define PATH_PLANNING_SRC_TRAJECTORY_H_

#include <vector>
#include <memory>
#include <cmath>
#include <iostream>
#include <utility>
#include "../include/Eigen-3.3/Eigen/Core"
#include "types.h"
#include "utils.h"

class Trajectory;
using TrajectoryPtr = std::shared_ptr<Trajectory>;

using SDState = Eigen::VectorXd;

using PolyCoeffs = Eigen::VectorXd;
class Trajectory {
 public:
  Trajectory();

  /**@brief
   *
   * @param start_state [IN]: start state, vector of size 6: [s, s_d, s_dd, d, d_d, d_dd]
   * @param end_state [IN]: end state, vector of size 6: [s, s_d, s_dd, d, d_d, d_dd]
   * @param duration_s
   * @param duration_d
   * @param time_start
   */
  Trajectory(const SDState &start_state,
             const SDState &end_state,
             double duration_s,
             double duration_d,
             double time_start);

  /**@brief calculate jerk minimizing trajectory for the velocity lanekeeping state;
   *
   * @param cur_state [IN]: current state vector, [s, s_d, s_dd, d, d_d, d_dd]
   * @param end_d]
   * @param target_velocity
   * @param cur_time
   * @param time_duration_s
   * @param time_duration_d
   * @return
   */
  static TrajectoryPtr VelocityKeepSTrajectory(const SDState &cur_state,
                                               double end_d,
                                               double target_velocity,
                                               double cur_time,
                                               double time_duration_s,
                                               double time_duration_d);

  /**@brief
   * return vector state at specified time;
   * @param time
   * @return
   */
  SDState EvaluateStateAt(double time) const;

  /**@brief
   *
   * @param time_duration
   * @param max_js
   * @param max_jd
   * @return
   */
  double CalcJerkCost(double time_duration, double &max_js, double max_jd) const;

  double CalcAccelCost(double time_duration) const;

  double CalcVelocityCost(double target_velocity, double time_duration) const;

  double CalcSafetyDistanceCost(const Eigen::MatrixXd &s2, double time_duration) const;

  double CalcLaneOffsetCost(double time_duration) const;

  std::pair<double, double> MinMaxVelocity_S() const;

  double GetDurationS() const {
    return duration_s_;
  }

  double GetDurationD() const {
    return duration_d_;
  }

  SDState GetStartState() const {
    return start_state_;
  }

  double GetStartS() const{
    return start_state_(0);
  }

  double GetTargetS() const {
    return target_state_(0);
  }

  double GetTargetD() const {
    return target_state_(3);
  }

  double GetStartD() const {
    return start_state_(3);
  }

  double GetTotalCost() const {
    return cost_.sum();
  }

  void SetJerkCost(double cost) {
    cost_(0) = cost;
  }

  void SetAccelCost(double cost) {
    cost_(1) = cost;
  }

  void SetVelocityCost(double cost) {
    cost_(2) = cost;
  }

  void SetSafetyDistanceCost(double cost) {
    cost_(3) = cost;
  }

  void SetLaneOffsetCost(double cost) {
    cost_(4) = cost;
  }

  double GetJerkCost() const {
    return cost_(0);
  }

  double GetAccelCost(double cost) {
    return cost_(1);
  }

  double GetVelocityCost(double cost) {
    return cost_(2);
  }

  double GetSafetyDistanceCost() const {
    return cost_(3);
  }

  double GetLaneOffsetCost() const {
    return cost_(4);
  }

  double GetTargetVelocity() const {
    return target_velocity_;
  }

  void PrintInfo();
 public:
  PolyCoeffs S_coeffs_;
  PolyCoeffs D_coeffs_;
  Eigen::VectorXd cost_{Eigen::VectorXd::Zero(5)};
 private:
  double time_start_;
  double duration_s_;//duration t of s state in secs;
  double duration_d_;//duration t of d state in secs;
  double delta_t = k_delta_t;
  double cost_dt_ = 0.1;//time step along a trajectory used during evaluation of trajectory cost;
  double target_velocity_;


  SDState start_state_;
  SDState target_state_;

 private:
  double CalcSafetyDistanceCost(double S_dist, double D_dist, double velocity) const;

  /**@brief calculate jerk minimizing trajectory for state state, end state and duration T;
   *
   * @param start_state_x3 [IN]: start state of size 3, [s, s_d, s_dd]
   * @param target_state_x3 [IN]: target state of size 3, [s, s_d, s_dd]
   * @param T time;
   * @return polynomial coefficients;
   */
  static PolyCoeffs CalcQuinticPolynomialCoeffs(const SDState &start_state_x3, const SDState &target_state_x3, double T);

  /**@brief
   * calculate polynomial value at time t and returns 3-dim vector;
   * @param coeffs_x6
   * @param t
   * @return 3-dim vector, [s, s_d, s_dd]
   */
  static Eigen::VectorXd CalcPolynomialAt(const PolyCoeffs &coeffs_x6, double t);

  inline double CalcPolynomialJerkAt(const PolyCoeffs &coeffs_x6, double t) const;

  inline double CalcPolynomialAccelAt(const PolyCoeffs &coeffs_x6, double t) const;

  inline double CalcPolynomialVelocityAt(const PolyCoeffs &coeffs_x6, double t) const;

  inline double CalcPolynomialDistanceAt(const PolyCoeffs &coeffs_x6, double t) const;

  /**@brief
   * calculate jerk optimial polynomial for s-trajectory with velocity keeping;
   * @param start_state_v3 [IN]: current state, [s, s_d, s_dd]
   * @param target_velocity
   * @param T
   * @return polynomial cofficients;
   */
  static PolyCoeffs CalcSPolynomialVelocityKeeping(const Eigen::VectorXd &start_state_v3,
                                                   double target_velocity,
                                                   double T);
};

#endif //PATH_PLANNING_SRC_TRAJECTORY_H_
