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

FrenetState getFrenet(VehicleState current_state, const Map& map)
{
  // std::cout << "getFrenet() Break 0" << std::endl;

  int next_wp_id = nextWaypoint(current_state, map);
  int prev_wp_id = next_wp_id - 1;

  // std::cout << "getFrenet() Break 1" << std::endl;

  // if it reaches the end of the waypoint list
  if (next_wp_id >= map.x.size())
  {
    prev_wp_id = map.x.size() - 1;
  }

  // std::vector n from previous waypoint to next waypoint
  double n_x = map.x.at(next_wp_id) - map.x.at(prev_wp_id);
  double n_y = map.y.at(next_wp_id) - map.y.at(prev_wp_id);
  // std::vector x from previous waypoint to current position
  double x_x = current_state.x - map.x.at(prev_wp_id);
  double x_y = current_state.y - map.y.at(prev_wp_id);

  // std::cout << "getFrenet() Break 2" << std::endl;

  // find the projection of x on n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  FrenetState state;
  state.d = distance(x_x, x_y, proj_x, proj_y);

  // get the normal std::vector d
  double d_x = map.dx.at(prev_wp_id);
  double d_y = map.dy.at(prev_wp_id);
  double wp_yaw = atan2(d_y, d_x) + M_PI / 2;
  double delta_yaw = current_state.yaw - wp_yaw;
  while (delta_yaw > M_PI)
  {
    delta_yaw -= (2 * M_PI);
  }
  while (delta_yaw < -M_PI)
  {
    delta_yaw += (2 * M_PI);
  }

  // std::cout << "getFrenet() Break 3" << std::endl;

  // find the projection of x on d
  double proj_x_d = x_x * d_x + x_y * d_y;

  if (proj_x_d >= 0)
  {
    state.d *= -1;
  }

  // calculate s value
  state.s = 0;
  for (int i = 0; i < prev_wp_id; i++)
  {
    state.s += distance(map.x.at(i), map.y.at(i), map.x.at(i + 1), map.y.at(i + 1));
  }

  state.s += distance(0.0, 0.0, proj_x, proj_y);

  // calculate s_d and d_d
  state.s_d = current_state.v * cos(delta_yaw);
  state.d_d = current_state.v * sin(delta_yaw);

  return state;
}

FrenetState getFrenet(VehicleState current_state, const Path& path)
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
  const double n_x = path.x.at(next_wp_id) - path.x.at(prev_wp_id);
  const double n_y = path.y.at(next_wp_id) - path.y.at(prev_wp_id);
  // std::vector x from previous waypoint to current position
  const double x_x = current_state.x - path.x.at(prev_wp_id);
  const double x_y = current_state.y - path.y.at(prev_wp_id);

  // std::cout << "getFrenet() Break 2" << std::endl;

  // find the projection of x on n
  const double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  const double proj_x = proj_norm * n_x;
  const double proj_y = proj_norm * n_y;

  FrenetState state;
  state.d = distance(x_x, x_y, proj_x, proj_y);

  // get the normal std::vector d
  const double wp_yaw = path.yaw.at(prev_wp_id);
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
    state.s += distance(path.x.at(i), path.y.at(i), path.x.at(i + 1), path.y.at(i + 1));
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
