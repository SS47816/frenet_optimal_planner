/** frenet.cc
 *
 * Copyright (C) 2019 SS47816 & Advanced Robotics Center, National University of Singapore & Micron Technology
 *
 * Construction of frenet coordinates
 * Conversion between Frenet frame and Cartesian frame
 */

#include "frenet.h"

namespace fop
{

FrenetPath::FrenetPath() {}
FrenetPath::FrenetPath(const FrenetState& end_state) : is_generated(false)
{
  this->lane_id = end_state.lane_id;
  // Pass the pre-computed costs to trajectory
  this->fix_cost = end_state.fix_cost;
  this->est_cost = end_state.est_cost;
  this->dyn_cost = 0.0;
  this->final_cost = end_state.final_cost;
}

void FrenetPath::generateTrajectory(const FrenetState& start_state, const FrenetState& end_state, const double tick_t)
{
  // generate lateral quintic polynomial
  QuinticPolynomial lateral_quintic_poly = QuinticPolynomial(start_state, end_state);

  // store the this lateral trajectory into frenet_traj
  for (double t = 0.0; t <= end_state.T; t += tick_t)
  {
    this->t.emplace_back(t);
    this->d.emplace_back(lateral_quintic_poly.calculatePoint(t));
    this->d_d.emplace_back(lateral_quintic_poly.calculateFirstDerivative(t));
    this->d_dd.emplace_back(lateral_quintic_poly.calculateSecondDerivative(t));
    this->d_ddd.emplace_back(lateral_quintic_poly.calculateThirdDerivative(t));
  }

  // generate longitudinal quartic polynomial
  QuarticPolynomial longitudinal_quartic_poly = QuarticPolynomial(start_state, end_state);

  // store the this longitudinal trajectory into frenet_traj
  for (double t = 0.0; t <= end_state.T; t += tick_t)
  {
    this->s.emplace_back(longitudinal_quartic_poly.calculatePoint(t));
    this->s_d.emplace_back(longitudinal_quartic_poly.calculateFirstDerivative(t));
    this->s_dd.emplace_back(longitudinal_quartic_poly.calculateSecondDerivative(t));
    this->s_ddd.emplace_back(longitudinal_quartic_poly.calculateThirdDerivative(t));
  }
}

FrenetState getFrenet(const VehicleState& current_state, const Lane& lane)
{
  // std::cout << "getFrenet() Break 0" << std::endl;

  int next_wp_id = nextWaypoint(current_state, lane);
  // std::cout << "getFrenet() Break 1" << std::endl;

  // if it reaches the end of the waypoint list
  if (next_wp_id >= lane.points.size())
  {
    next_wp_id = lane.points.size() - 1;
  }

  const int prev_wp_id = next_wp_id - 1;

  // vector n from previous waypoint to next waypoint
  const double n_x = lane.points[next_wp_id].point.x - lane.points[prev_wp_id].point.x;
  const double n_y = lane.points[next_wp_id].point.y - lane.points[prev_wp_id].point.y;

  // vector x from previous waypoint to current position
  const double x_x = current_state.x - lane.points[prev_wp_id].point.x;
  const double x_y = current_state.y - lane.points[prev_wp_id].point.y;
  const double x_yaw = atan2(x_y, x_x);
  // std::cout << "getFrenet() Break 2" << std::endl;

  // find the projection of x on n
  const double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  const double proj_x = proj_norm * n_x;
  const double proj_y = proj_norm * n_y;

  FrenetState state;
  state.d = distance(x_x, x_y, proj_x, proj_y);

  const double wp_yaw = lane.points[prev_wp_id].point.yaw;
  const double delta_yaw = unifyAngleRange(current_state.yaw - wp_yaw);

  if (wp_yaw >= x_yaw)
  {
    state.d *= -1;
  }
  
  // calculate s value
  state.s = 0;
  for (int i = 0; i < prev_wp_id; i++)
  {
    state.s += distance(lane.points[i].point.x, lane.points[i].point.y, lane.points[i+1].point.x, lane.points[i+1].point.y);
  }

  state.s += distance(0.0, 0.0, proj_x, proj_y);

  // calculate s_d and d_d
  state.s_d = current_state.v * cos(delta_yaw);
  state.d_d = current_state.v * sin(delta_yaw);

  state.s_dd = 0.0;
  state.s_ddd = 0.0;
  state.d_dd = 0.0;
  state.d_ddd = 0.0;

  return state;
}

FrenetState getFrenet(const VehicleState& current_state, const Path& path)
{
  // std::cout << "getFrenet() Break 0" << std::endl;
  int next_wp_id = nextWaypoint(current_state, path);
  // if it reaches the end of the waypoint list
  if (next_wp_id >= path.x.size())
  {
    next_wp_id = path.x.size() - 1;
  }
  int prev_wp_id = next_wp_id - 1;
  if (prev_wp_id < 0)
  {
    prev_wp_id = 0;
  }

  // std::cout << "getFrenet() Break 1" << std::endl;

  // std::vector n from previous waypoint to next waypoint
  const double n_x = path.x[next_wp_id] - path.x[prev_wp_id];
  const double n_y = path.y[next_wp_id] - path.y[prev_wp_id];
  // std::vector x from previous waypoint to current position
  const double x_x = current_state.x - path.x[prev_wp_id];
  const double x_y = current_state.y - path.y[prev_wp_id];

  // std::cout << "getFrenet() Break 2" << std::endl;

  // find the projection of x on n
  const double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  const double proj_x = proj_norm * n_x;
  const double proj_y = proj_norm * n_y;

  FrenetState state;
  state.d = distance(x_x, x_y, proj_x, proj_y);

  // get the normal std::vector d
  const double wp_yaw = path.yaw[prev_wp_id];
  const double delta_yaw = fop::unifyAngleRange(current_state.yaw - wp_yaw);

  // std::cout << "getFrenet() Break 3" << std::endl;

  // find the yaw of std::vector x
  const double x_yaw = atan2(x_y, x_x);
  const double yaw_x_n = fop::unifyAngleRange(x_yaw - wp_yaw);

  if (yaw_x_n < 0.0)
  {
    state.d *= -1;
  }

  // calculate s value
  state.s = 0;
  for (int i = 0; i < prev_wp_id; i++)
  {
    state.s += distance(path.x[i], path.y[i], path.x[i + 1], path.y[i + 1]);
  }
  state.s += distance(0.0, 0.0, proj_x, proj_y);

  // calculate s_d and d_d
  state.s_d = current_state.v * cos(delta_yaw);
  state.d_d = current_state.v * sin(delta_yaw);
  // Give default values to the rest of the attributes
  state.s_dd = 0.0;
  state.d_dd = 0.0;
  state.s_ddd = 0.0;
  state.d_ddd = 0.0;

  return state;
}

}  // end of namespace fop
