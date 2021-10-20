/** lane.cc
 *
 * Copyright (C) 2019 SS47816 & Advanced Robotics Center, National University of Singapore & Micron Technology
 *
 * Helper functions related to lanes
 */

#include "lane.h"

namespace fop
{

Map::Map(const frenet_optimal_planner::LaneInfo::ConstPtr& lane_info)
{
  for (auto waypoint : lane_info->waypoints)
  {
    this->x.push_back(waypoint.x);
    this->y.push_back(waypoint.y);
    this->dx.push_back(waypoint.dx);
    this->dy.push_back(waypoint.dy);
    this->s.push_back(waypoint.s);
    this->left_widths.push_back(waypoint.left_width);
    this->right_widths.push_back(waypoint.right_width);
    this->far_right_widths.push_back(waypoint.far_right_width);
    this->special_points.push_back(waypoint.special_point);
  }
}

void Map::clear()
{
  x.clear();
  y.clear();
  dx.clear();
  dy.clear();
  s.clear();
  left_widths.clear();
  right_widths.clear();
  far_right_widths.clear();
  special_points.clear();
}

void Path::clear()
{
  x.clear();
  y.clear();
  yaw.clear();
}

int closestWaypoint(VehicleState current_state, const Path& path)
{
  double closest_dist = 100000.0;  // start with a large number
  int closest_waypoint = 0;

  for (int i = 0; i < path.x.size(); i++)
  {
    double dist = distance(current_state.x, current_state.y, path.x.at(i), path.y.at(i));
    if (dist < closest_dist)
    {
      closest_dist = dist;
      closest_waypoint = i;
    }
  }

  return closest_waypoint;
}

int closestWaypoint(VehicleState current_state, const Map& map)
{
  double closest_dist = 100000.0;  // start with a large number
  int closest_waypoint = 0;

  for (int i = 0; i < map.x.size(); i++)
  {
    double dist = distance(current_state.x, current_state.y, map.x.at(i), map.y.at(i));
    if (dist < closest_dist)
    {
      closest_dist = dist;
      closest_waypoint = i;
    }
  }

  return closest_waypoint;
}

int nextWaypoint(VehicleState current_state, const Path& path)
{
  int closest_waypoint = closestWaypoint(current_state, path);
  double heading = atan2((path.y.at(closest_waypoint) - current_state.y), (path.x.at(closest_waypoint) - current_state.x));

  double angle = fabs(current_state.yaw - heading);
  angle = std::min(2 * pi() - angle, angle);

  if (angle > pi() / 2)
  {
    closest_waypoint++;
  }

  return closest_waypoint;
}

int nextWaypoint(VehicleState current_state, const Map& map)
{
  int closest_waypoint = closestWaypoint(current_state, map);
  double heading = atan2((map.y.at(closest_waypoint) - current_state.y), (map.x.at(closest_waypoint) - current_state.x));

  double angle = fabs(current_state.yaw - heading);
  angle = std::min(2 * pi() - angle, angle);

  if (angle > pi() / 2)
  {
    closest_waypoint++;
  }

  return closest_waypoint;
}

int lastWaypoint(VehicleState current_state, const Path& path)
{
  return nextWaypoint(current_state, path) - 1;
}

int lastWaypoint(VehicleState current_state, const Map& map)
{
  return nextWaypoint(current_state, map) - 1;
}

}  // end of namespace fop