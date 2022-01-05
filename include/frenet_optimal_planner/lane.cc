/** lane.cc
 *
 * Copyright (C) 2019 SS47816 & Advanced Robotics Center, National University of Singapore & Micron Technology
 *
 * Helper functions related to lanes
 */

#include "lane.h"

namespace fop
{

Waypoint::Waypoint() {}
Waypoint::Waypoint(const double x, const double y, const double yaw)
  : x(x), y(y), yaw(yaw) {}
Waypoint::Waypoint(const tf::Pose& pose)
{
  this->x = pose.getOrigin().x();
  this->y = pose.getOrigin().y();
  this->yaw = tf::getYaw(pose.getRotation());
}
Waypoint::Waypoint(const geometry_msgs::Pose& pose_msg)
{
  tf::Pose pose;
  tf::poseMsgToTF(pose_msg, pose);
  this->x = pose.getOrigin().x();
  this->y = pose.getOrigin().y();
  this->yaw = tf::getYaw(pose.getRotation());
}

LanePoint::LanePoint() {};
LanePoint::LanePoint(const Waypoint& point, const double left_width, const double right_width, const double far_left_width, const double far_right_width)
  : point(point), left_width(left_width), right_width(right_width), far_left_width(far_left_width), far_right_width(far_right_width) {}
LanePoint::LanePoint(const geometry_msgs::Pose& pose, const double left_width, const double right_width, const double far_left_width, const double far_right_width)
  : left_width(left_width), right_width(right_width), far_left_width(far_left_width), far_right_width(far_right_width)
{
  this->point = Waypoint(pose);
}


// Lane::Lane(const frenet_optimal_planner::LaneInfo::ConstPtr& lane_info)
// {
//   for (auto& waypoint : lane_info->waypoints)
//   {
//     this->x.emplace_back(waypoint.x);
//     this->y.emplace_back(waypoint.y);
//     this->dx.emplace_back(waypoint.dx);
//     this->dy.emplace_back(waypoint.dy);
//     // this->s.emplace_back(waypoint.s);
//     this->left_widths.emplace_back(waypoint.left_width);
//     this->right_widths.emplace_back(waypoint.right_width);
//     this->far_right_widths.emplace_back(waypoint.far_right_width);
//     this->special_points.emplace_back(waypoint.special_point);
//   }
// }

Lane::Lane(const nav_msgs::Path::ConstPtr& ref_path, const double lane_width, const double left_lane_width, const double right_lane_width)
{
  for (auto& ref_pose : ref_path->poses)
  {
    this->points.emplace_back(LanePoint(ref_pose.pose, lane_width/2, lane_width/2, lane_width/2 + left_lane_width, lane_width/2 + right_lane_width));
    // this->left_widths.emplace_back(lane_width/2);
    // this->right_widths.emplace_back(lane_width/2);
    // this->far_left_widths.emplace_back(lane_width/2 + left_lane_width);
    // this->far_right_widths.emplace_back(lane_width/2 + right_lane_width);
    // this->special_points.emplace_back(waypoint.special_point);
  }
}

void Lane::clear()
{
  points.clear();
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
    double dist = distance(current_state.x, current_state.y, path.x[i], path.y[i]);
    if (dist < closest_dist)
    {
      closest_dist = dist;
      closest_waypoint = i;
    }
  }

  return closest_waypoint;
}

int closestWaypoint(VehicleState current_state, const Lane& lane)
{
  double closest_dist = 100000.0;  // start with a large number
  int closest_waypoint = 0;

  for (int i = 0; i < lane.points.size(); i++)
  {
    double dist = distance(current_state.x, current_state.y, lane.points[i].point.x, lane.points[i].point.y);
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
  double heading = atan2((path.y[closest_waypoint] - current_state.y), (path.x[closest_waypoint] - current_state.x));

  double angle = fabs(current_state.yaw - heading);
  angle = std::min(2 * pi() - angle, angle);

  if (angle > pi() / 2)
  {
    closest_waypoint++;
  }

  return closest_waypoint;
}

int nextWaypoint(VehicleState current_state, const Lane& lane)
{
  int closest_waypoint = closestWaypoint(current_state, lane);
  double heading = atan2((lane.points[closest_waypoint].point.y - current_state.y), (lane.points[closest_waypoint].point.x - current_state.x));

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

int lastWaypoint(VehicleState current_state, const Lane& lane)
{
  return nextWaypoint(current_state, lane) - 1;
}

}  // end of namespace fop