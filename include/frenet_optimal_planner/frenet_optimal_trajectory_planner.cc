/* frenet_optimal_trajectory_planner.cc

  Copyright (C) 2019 SS47816 & Advanced Robotics Center, National University of Singapore & Micron Technology

  Implementation of Optimal trajectory planning in Frenet Coordinate Algorithm
  Using the algorithm described in this paper, https://ieeexplore.ieee.org/document/5509799
*/

#include "frenet_optimal_trajectory_planner.h"
#include <ros/ros.h>

namespace fop
{
FrenetOptimalTrajectoryPlanner::FrenetOptimalTrajectoryPlanner(FrenetOptimalTrajectoryPlanner::Setting settings)
{
  this->settings_ = settings;
}

std::pair<Path, Spline2D> FrenetOptimalTrajectoryPlanner::generateReferenceCurve(const fop::Lane& lane)
{
  Path ref_path = Path();
  auto cubic_spline = fop::Spline2D(lane);

  std::vector<double> s;
  for (double i = 0; i < cubic_spline.s_.back(); i += 0.1)
  {
    s.emplace_back(i);
  }

  for (int i = 0; i < s.size(); i++)
  {
    fop::VehicleState state = cubic_spline.calculatePosition(s[i]);
    ref_path.x.emplace_back(state.x);
    ref_path.y.emplace_back(state.y);
    ref_path.yaw.emplace_back(cubic_spline.calculateYaw(s[i]));
  }

  return std::pair<Path, Spline2D>{ref_path, cubic_spline};
}

std::vector<fop::FrenetPath> FrenetOptimalTrajectoryPlanner::frenetOptimalPlanning(
    fop::Spline2D& cubic_spline, const fop::FrenetState& frenet_state, double center_offset,
    double left_width, double right_width, const autoware_msgs::DetectedObjectArray& obstacles, double desired_speed,
    double current_speed, int path_size)
{
  // Sample a list of FrenetPaths
  std::vector<fop::FrenetPath> frenet_traj_list = generateFrenetPaths(frenet_state, center_offset, left_width, right_width, desired_speed, current_speed);
  const int num_paths_generated = frenet_traj_list.size();
  std::cout << "Total Paths Generated: " << num_paths_generated << std::endl;

  // Convert to global paths
  frenet_traj_list = calculateGlobalPaths(frenet_traj_list, cubic_spline);
  std::cout << "Paths Converted to Global Frame: " << frenet_traj_list.size() << std::endl;

  // Check the constraints
  std::cout << "Start to Check Paths Constraints" << std::endl;
  checkConstraints(frenet_traj_list);
  std::cout << "Constraints Checked for all Paths, Starting Collision Checking..." << std::endl;

  // Compute costs
  computeCosts(frenet_traj_list, current_speed);

  // Check collisions
  const auto begin = ros::WallTime::now().toSec();
  checkCollisions(frenet_traj_list, obstacles, false);
  const auto end = ros::WallTime::now().toSec();
  std::cout << num_paths_generated << " Paths Passed Collision Check in " << (end - begin) << "secs, "
            << frenet_traj_list.size() << " paths passed check" << std::endl;

  // Find the path with minimum costs
  std::vector<fop::FrenetPath> best_path_list = findBestPaths(frenet_traj_list);

  return best_path_list;
}

std::vector<fop::FrenetPath> FrenetOptimalTrajectoryPlanner::generateFrenetPaths(const fop::FrenetState& frenet_state, double center_offset, 
  double left_bound, double right_bound, double desired_speed, double current_speed)
{
  // list of frenet paths generated
  std::vector<fop::FrenetPath> frenet_trajs;
  std::vector<double> goal_ds;

  // generate different goals with a lateral offset
  for (double d = 0.0 + center_offset; d <= left_bound; d += settings_.delta_width)  // left being positive
  {
    goal_ds.push_back(d);
  }
  for (double d = 0.0 + center_offset - settings_.delta_width; d >= right_bound; d -= settings_.delta_width)  // right being negative
  {
    goal_ds.push_back(d);
  }

  for (auto goal_d : goal_ds)
  {
    // generate d_t polynomials
    int t_count = 0;
    for (double T = settings_.min_t; T <= settings_.max_t; T += settings_.delta_t)
    {
      t_count++;
      
      fop::FrenetPath frenet_traj = fop::FrenetPath();
      // left lane
      if (goal_d >= -left_bound)
      {
        frenet_traj.lane_id = 1;
      }
      // transition area
      else if (goal_d >= right_bound + 2 * left_bound)
      {
        frenet_traj.lane_id = 0;
      }
      // right lane
      else if (goal_d >= right_bound)
      {
        frenet_traj.lane_id = 2;
      }
      // fault lane
      else
      {
        frenet_traj.lane_id = -1;
      }

      // start lateral state [d, d_d, d_dd]
      std::vector<double> start_d;
      start_d.emplace_back(frenet_state.d);
      start_d.emplace_back(frenet_state.d_d);
      start_d.emplace_back(frenet_state.d_dd);

      // end lateral state [d, d_d, d_dd]
      std::vector<double> end_d;
      end_d.emplace_back(goal_d);
      end_d.emplace_back(0.0);
      end_d.emplace_back(0.0);

      // generate lateral quintic polynomial
      fop::QuinticPolynomial lateral_quintic_poly = fop::QuinticPolynomial(start_d, end_d, T);

      // store the this lateral trajectory into frenet_traj
      for (double t = 0.0; t <= T; t += settings_.tick_t)
      {
        frenet_traj.t.emplace_back(t);
        frenet_traj.d.emplace_back(lateral_quintic_poly.calculatePoint(t));
        frenet_traj.d_d.emplace_back(lateral_quintic_poly.calculateFirstDerivative(t));
        frenet_traj.d_dd.emplace_back(lateral_quintic_poly.calculateSecondDerivative(t));
        frenet_traj.d_ddd.emplace_back(lateral_quintic_poly.calculateThirdDerivative(t));
      }

      // generate longitudinal quartic polynomial
      const double min_sample_speed = settings_.target_speed - (settings_.num_speed_sample - 1)/2*settings_.delta_speed;
      const double max_sample_speed = settings_.target_speed + (settings_.num_speed_sample - 1)/2*settings_.delta_speed;
      for (double sample_speed = min_sample_speed; sample_speed <= max_sample_speed; sample_speed += settings_.delta_speed)
      {
        if (sample_speed <= 0)  // ensure target speed is positive
        {
          ROS_WARN("target speed too low, increasing value");
          continue;
        }

        // copy the longitudinal path over
        fop::FrenetPath target_frenet_traj = frenet_traj;

        // start longitudinal state [s, s_d, s_dd]
        std::vector<double> start_s;
        start_s.emplace_back(frenet_state.s);
        start_s.emplace_back(frenet_state.s_d);
        start_s.emplace_back(0.0);

        // end longitudinal state [s_d, s_dd]
        std::vector<double> end_s;
        end_s.emplace_back(sample_speed);
        end_s.emplace_back(0.0);

        // generate longitudinal quartic polynomial
        fop::QuarticPolynomial longitudinal_quartic_poly = fop::QuarticPolynomial(start_s, end_s, T);

        // store the this longitudinal trajectory into target_frenet_traj
        for (double t = 0.0; t <= T; t += settings_.tick_t)
        {
          target_frenet_traj.s.emplace_back(longitudinal_quartic_poly.calculatePoint(t));
          target_frenet_traj.s_d.emplace_back(longitudinal_quartic_poly.calculateFirstDerivative(t));
          target_frenet_traj.s_dd.emplace_back(longitudinal_quartic_poly.calculateSecondDerivative(t));
          target_frenet_traj.s_ddd.emplace_back(longitudinal_quartic_poly.calculateThirdDerivative(t));
        }
        
        frenet_trajs.emplace_back(target_frenet_traj);
      }
    }
  }

  return frenet_trajs;
}

std::vector<fop::FrenetPath> FrenetOptimalTrajectoryPlanner::calculateGlobalPaths(std::vector<fop::FrenetPath>& frenet_traj_list, fop::Spline2D& cubic_spline)
{
  for (int i = 0; i < frenet_traj_list.size(); i++)
  {
    // calculate global positions
    for (int j = 0; j < frenet_traj_list[i].s.size(); j++)
    {
      fop::VehicleState state = cubic_spline.calculatePosition(frenet_traj_list[i].s[j]);
      double i_yaw = cubic_spline.calculateYaw(frenet_traj_list[i].s[j]);
      const double di = frenet_traj_list[i].d[j];
      const double frenet_x = state.x + di * cos(i_yaw + M_PI / 2.0);
      const double frenet_y = state.y + di * sin(i_yaw + M_PI / 2.0);
      if (!fop::isLegal(frenet_x) || !fop::isLegal(frenet_y))
      {
        break;
      }
      else
      {
        frenet_traj_list[i].x.emplace_back(frenet_x);
        frenet_traj_list[i].y.emplace_back(frenet_y);
      }
    }
    // calculate yaw and ds
    for (int j = 0; j < frenet_traj_list[i].x.size() - 1; j++)
    {
      const double dx = frenet_traj_list[i].x[j+1] - frenet_traj_list[i].x[j];
      const double dy = frenet_traj_list[i].y[j+1] - frenet_traj_list[i].y[j];
      frenet_traj_list[i].yaw.emplace_back(atan2(dy, dx));
      frenet_traj_list[i].ds.emplace_back(sqrt(dx * dx + dy * dy));
    }

    frenet_traj_list[i].yaw.emplace_back(frenet_traj_list[i].yaw.back());
    frenet_traj_list[i].ds.emplace_back(frenet_traj_list[i].ds.back());

    // calculate curvature
    for (int j = 0; j < frenet_traj_list[i].yaw.size() - 1; j++)
    {
      double yaw_diff = fop::unifyAngleRange(frenet_traj_list[i].yaw[j+1] - frenet_traj_list[i].yaw[j]);
      frenet_traj_list[i].c.emplace_back(yaw_diff / frenet_traj_list[i].ds[j]);
    }
  }

  return frenet_traj_list;
}

/**
 * @brief Checks whether frenet paths are safe to execute based on constraints and whether there is a collision along a
 * path
 *
 * @param frenet_traj_list the vector of paths to check
 * @param obstacles obstacles to check against for collision
 * @return vector containing the safe paths. If there are no safe paths, a dummy vector is returned.
 */

void FrenetOptimalTrajectoryPlanner::checkConstraints(std::vector<fop::FrenetPath>& frenet_traj_list)
{
  for (auto& frenet_traj : frenet_traj_list)
  {
    bool passed = true;
    for (int j = 0; j < frenet_traj.c.size(); j++)
    {
      if (!fop::isLegal(frenet_traj.x[j]) || !fop::isLegal(frenet_traj.y[j]))
      {
        passed = false;
        // std::cout << "Condition 0: Contains ilegal values" << std::endl;
        break;
      }
      else if (frenet_traj.s_d[j] > settings_.max_speed)
      {
        passed = false;
        // std::cout << "Condition 1: Exceeded Max Speed" << std::endl;
        break;
      }
      else if (frenet_traj.s_dd[j] > settings_.max_accel || frenet_traj.s_dd[j] < settings_.max_decel)
      {
        passed = false;
        // std::cout << "Condition 2: Exceeded Max Acceleration" << std::endl;
        break;
      }
      else if (std::abs(frenet_traj.c[j]) > settings_.max_curvature)
      {
        passed = false;
        std::cout << "Exceeded max curvature = " << settings_.max_curvature
                  << ". Curr curvature = " << (frenet_traj.c[j]) << std::endl;
        break;
      }
    }

    frenet_traj.constraint_passed = passed;

    // if (safe)
    // {
    //   //! Do curvature check only on waypoints to be put into path
    //   for (int j = 0; j < frenet_traj.c.size(); j++)
    //   {
    //     if (std::abs(frenet_traj.c[j] - frenet_traj.c[j-1]) > settings_.max_curvature)
    //     {
    //       // frenet_traj.curvature_check = false;
    //       std::cout << "Exceeded max curvature = " << settings_.max_curvature
    //                 << ". Curr curvature = " << (frenet_traj.c[j]) << std::endl;
    //       break;
    //     }
    //   }

    //   if (curvature_passed)
    //   {
    //     passed_constraints_paths.emplace_back(frenet_traj);
    //   }
    // }
    // else
    // {
    //   unsafe_paths.emplace_back(frenet_traj);
    // }
  }
}

void FrenetOptimalTrajectoryPlanner::computeCosts(std::vector<fop::FrenetPath>& frenet_trajs, const double curr_speed)
{
  for (auto& traj : frenet_trajs)
  {
    // calculate the costs
    double jerk_s = 0.0;
    double jerk_d = 0.0;
    // calculate total squared jerks
    for (int i = 0; i < traj.t.size(); i++)
    {
      jerk_s += pow(traj.s_ddd[i], 2);
      jerk_d += pow(traj.d_ddd[i], 2);
    }

    // encourage driving inbetween the desired speed and current speed
    const double speed_diff = pow(settings_.target_speed - traj.s_d.back(), 2) + 0.5*pow(curr_speed - traj.s_d.back(), 2);

    // encourage longer planning time
    const double planning_time_cost = settings_.k_time * (1 - traj.t.size()*settings_.tick_t / settings_.max_t);

    traj.cd = settings_.k_jerk * jerk_d + planning_time_cost + settings_.k_diff * pow(traj.d.back() - settings_.centre_offset, 2);
    traj.cs = settings_.k_jerk * jerk_s + planning_time_cost + settings_.k_diff * speed_diff;
    traj.cf = settings_.k_lateral * traj.cd + settings_.k_longitudinal * traj.cs;
  }
}


void FrenetOptimalTrajectoryPlanner::checkCollisions(std::vector<fop::FrenetPath>& frenet_traj_list, const autoware_msgs::DetectedObjectArray& obstacles, const bool use_async)
{
  /* ------------------- Check paths for collisions (Async) ------------------- */

  // std::vector<std::future<bool>> collision_checks;
  // std::vector<std::future<bool>> backup_collision_checks;

  // for (auto const& frenet_traj : passed_constraints_paths)
  // {
  //   collision_checks.push_back(std::async(std::launch::async, &FrenetOptimalTrajectoryPlanner::checkTrajCollision, this,
  //                                         frenet_traj, obstacles, "no"));
  // }
  // std::cout << "Async Collision Checking Done!" << std::endl;

  // std::vector<bool> collision_checks(passed_constraints_paths.size(), bool{true});

  std::vector<bool> collision_checks;
  for (auto& frenet_traj : frenet_traj_list)
  {
    // collision_checks.push_back(checkTrajCollision(frenet_traj, obstacles, 0.0));
    frenet_traj.collision_passed = checkTrajCollision(frenet_traj, obstacles, 0.0);
  }
  std::cout << "Collision Checking Done!" << std::endl;
}

/**
 * @brief Check for collisions at each point along a frenet path
 *
 * @param frenet_traj the path to check
 * @param obstacles obstacles to check against for collision
 * @param margin collision margin in [m]
 * @return false if there is a collision along the path. Otherwise, true
 */
bool FrenetOptimalTrajectoryPlanner::checkTrajCollision(const fop::FrenetPath& frenet_traj,
                                                        const autoware_msgs::DetectedObjectArray& obstacles,
                                                        const double margin)
{
  // ROS_INFO("Collision checking start");

  geometry_msgs::Polygon vehicle_rect, obstacle_rect;
  for (auto const& object : obstacles.objects)
  {
    // Predict Object's Trajectory
    const int num_steps = frenet_traj.x.size();
    fop::Path object_traj = predictTrajectory(object, settings_.tick_t, num_steps);
    
    // Check for collisions between ego and obstacle trajectories
    for (int i = 0; i < num_steps; i++)
    {
      double cost = 0;

      double vehicle_center_x = frenet_traj.x[i] + fop::Vehicle::Lr() * cos(frenet_traj.yaw[i]);
      double vehicle_center_y = frenet_traj.y[i] + fop::Vehicle::Lr() * sin(frenet_traj.yaw[i]);

      vehicle_rect = sat_collision_checker_instance.construct_rectangle(vehicle_center_x, vehicle_center_y, frenet_traj.yaw[i], 
                                                                        settings_.vehicle_length, settings_.vehicle_width, margin);
      obstacle_rect = sat_collision_checker_instance.construct_rectangle(object_traj.x[i], object_traj.y[i], object_traj.yaw[i], 
                                                                         object.dimensions.x, object.dimensions.y, 0.0);

      if (sat_collision_checker_instance.check_collision(vehicle_rect, obstacle_rect))
      {
        return false;
      }
    }
  }

  return true;
}

fop::Path FrenetOptimalTrajectoryPlanner::predictTrajectory(const autoware_msgs::DetectedObject& obstacle, const double tick_t, const int steps)
{
  fop::Path obstacle_trajectory;

  obstacle_trajectory.x.push_back(obstacle.pose.position.x);
  obstacle_trajectory.y.push_back(obstacle.pose.position.y);
  tf2::Quaternion q_tf2(obstacle.pose.orientation.x, obstacle.pose.orientation.y,
                        obstacle.pose.orientation.z, obstacle.pose.orientation.w);
  tf2::Matrix3x3 m(q_tf2.normalize());
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  obstacle_trajectory.yaw.push_back(yaw);
  const double v = fop::magnitude(obstacle.velocity.linear.x, obstacle.velocity.linear.y, obstacle.velocity.linear.z);
  obstacle_trajectory.v.push_back(v);
  
  for (size_t i = 1; i < steps; i++)
  {
    obstacle_trajectory.x.push_back(obstacle_trajectory.x.back() + v*tick_t*std::cos(yaw));
    obstacle_trajectory.x.push_back(obstacle_trajectory.y.back() + v*tick_t*std::sin(yaw));
    obstacle_trajectory.yaw.push_back(yaw);
    obstacle_trajectory.v.push_back(v);
  }

  return obstacle_trajectory;
}

/**
 * @brief Find the path with the lowest cost in each area (transiton area, left lane, right lane)
 * NOTE: This function only works properly when sampling from both lanes
 *
 * @param frenet_traj_list vector of paths to sample from
 * @return vector containing the 3 best paths
 */
std::vector<fop::FrenetPath>
FrenetOptimalTrajectoryPlanner::findBestPaths(const std::vector<fop::FrenetPath>& frenet_traj_list)
{
  std::vector<fop::FrenetPath> best_path_list;
  best_path_list.emplace_back(findBestPath(frenet_traj_list, 0));  // transition area
  best_path_list.emplace_back(findBestPath(frenet_traj_list, 1));  // left lane
  best_path_list.emplace_back(findBestPath(frenet_traj_list, 2));  // right lane

  return best_path_list;
}

fop::FrenetPath FrenetOptimalTrajectoryPlanner::findBestPath(const std::vector<fop::FrenetPath>& frenet_traj_list, int target_lane_id)
{
  // if best paths list isn't empty
  if (!frenet_traj_list.empty())
  {
    bool found_path_in_target_lane = false;

    double min_cost = 1000000000.0;  // start with a large number
    int best_path_id = 0;
    for (int i = 0; i < frenet_traj_list.size(); i++)
    {
      if (!frenet_traj_list[i].constraint_passed || !frenet_traj_list[i].collision_passed)
      {
        continue;
      }
      else
      {
        if (frenet_traj_list[i].lane_id == target_lane_id)
        {
          found_path_in_target_lane = true;

          if (min_cost >= frenet_traj_list[i].cf)
          {
            min_cost = frenet_traj_list[i].cf;
            best_path_id = i;
          }
        }
      }
    }

    if (!found_path_in_target_lane)
    {
      ROS_WARN("NO PATH WITH LANE ID: %d", target_lane_id);
    }
    // std::cout << "Best Path ID: " << best_path_id << std::endl;

    return frenet_traj_list[best_path_id];
  }

  // if best paths list is empty
  else
  {
    // std::cout << "Best Path Is Empty" << std::endl;
    return fop::FrenetPath();
  }
}

}  // namespace fop