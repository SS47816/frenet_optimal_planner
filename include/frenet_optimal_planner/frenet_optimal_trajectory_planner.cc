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

FrenetOptimalTrajectoryPlanner::ResultType FrenetOptimalTrajectoryPlanner::generateReferenceCurve(const fop::Lane& lane)
{
  FrenetOptimalTrajectoryPlanner::ResultType result = FrenetOptimalTrajectoryPlanner::ResultType();
  result.cubic_spline = fop::Spline2D(lane);

  std::vector<double> s;
  for (double i = 0; i < result.cubic_spline.s_.back(); i += 0.1)
  {
    s.push_back(i);
  }

  for (int i = 0; i < s.size(); i++)
  {
    fop::VehicleState state = result.cubic_spline.calculatePosition(s[i]);
    result.rx.push_back(state.x);
    result.ry.push_back(state.y);
    result.ryaw.push_back(result.cubic_spline.calculateYaw(s[i]));
    result.rk.push_back(result.cubic_spline.calculateCurvature(s[i]));
  }

  return result;
}

std::vector<fop::FrenetPath> FrenetOptimalTrajectoryPlanner::frenetOptimalPlanning(
    fop::Spline2D& cubic_spline, const fop::FrenetState& frenet_state, double center_offset,
    double left_width, double right_width, const autoware_msgs::DetectedObjectArray& obstacles, double desired_speed,
    double current_speed, int path_size)
{
  // Sample a list of FrenetPaths
  std::vector<fop::FrenetPath> frenet_paths_list = generateFrenetPaths(frenet_state, center_offset, left_width, right_width, desired_speed, current_speed);
  const int num_paths_generated = frenet_paths_list.size();
  std::cout << "Total Paths Generated: " << frenet_paths_list.size() << std::endl;

  // Convert to global paths
  frenet_paths_list = calculateGlobalPaths(frenet_paths_list, cubic_spline);
  std::cout << "Paths Converted to Global Frame: " << frenet_paths_list.size() << std::endl;

  // Check the constraints
  const auto begin = ros::WallTime::now().toSec();
  std::cout << "Start to Check Paths Constraints" << std::endl;
  frenet_paths_list = checkPaths(frenet_paths_list, obstacles, path_size);
  const auto end = ros::WallTime::now().toSec();
  std::cout << num_paths_generated << " Paths Passed Collision Check in " << (end - begin) << "secs, "
            << frenet_paths_list.size() << " paths passed check" << frenet_paths_list.size() << std::endl;

  // Find the path with minimum costs
  std::vector<fop::FrenetPath> best_path_list = findBestPaths(frenet_paths_list);

  return best_path_list;
}

std::vector<fop::FrenetPath> FrenetOptimalTrajectoryPlanner::generateFrenetPaths(
    const fop::FrenetState& frenet_state, double center_offset, double left_bound, double right_bound,
    double desired_speed, double current_speed)
{
  // list of frenet paths generated
  std::vector<fop::FrenetPath> frenet_paths;
  std::vector<double> goal_ds;

  // generate different goals with a lateral offset
  for (double d = 0.0 + center_offset; d <= left_bound; d += settings_.delta_width)  // left being positive
  {
    goal_ds.push_back(d);
  }
  for (double d = 0.0 + center_offset - settings_.delta_width; d >= right_bound;
       d -= settings_.delta_width)  // right being negative
  {
    goal_ds.push_back(d);
  }

  // for (double goal_d = right_bound; goal_d <= left_bound; goal_d += settings_.delta_width)
  for (auto goal_d : goal_ds)
  {
    // generate d_t polynomials
    int t_count = 0;
    for (double T = settings_.min_t; T <= settings_.max_t; T += settings_.delta_t)
    {
      t_count++;
      // std::cout << T << std::endl;
      fop::FrenetPath frenet_path = fop::FrenetPath();

      // left lane
      if (goal_d >= -left_bound)
      {
        frenet_path.lane_id = 1;
      }
      // transition area
      else if (goal_d >= right_bound + 2 * left_bound)
      {
        frenet_path.lane_id = 0;
      }
      // right lane
      else if (goal_d >= right_bound)
      {
        frenet_path.lane_id = 2;
      }
      // fault lane
      else
      {
        frenet_path.lane_id = -1;
      }

      // start lateral state [d, d_d, d_dd]
      std::vector<double> start_d;
      start_d.push_back(frenet_state.d);
      start_d.push_back(frenet_state.d_d);
      start_d.push_back(frenet_state.d_dd);

      // end lateral state [d, d_d, d_dd]
      std::vector<double> end_d;
      end_d.push_back(goal_d);
      end_d.push_back(0.0);
      end_d.push_back(0.0);

      // generate lateral quintic polynomial
      fop::QuinticPolynomial lateral_quintic_poly = fop::QuinticPolynomial(start_d, end_d, T);

      // store the this lateral trajectory into frenet_path
      for (double t = 0.0; t <= T; t += settings_.tick_t)
      {
        frenet_path.t.push_back(t);
        frenet_path.d.push_back(lateral_quintic_poly.calculatePoint(t));
        frenet_path.d_d.push_back(lateral_quintic_poly.calculateFirstDerivative(t));
        frenet_path.d_dd.push_back(lateral_quintic_poly.calculateSecondDerivative(t));
        frenet_path.d_ddd.push_back(lateral_quintic_poly.calculateThirdDerivative(t));
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
        fop::FrenetPath target_frenet_path = frenet_path;

        // start longitudinal state [s, s_d, s_dd]
        std::vector<double> start_s;
        start_s.push_back(frenet_state.s);
        start_s.push_back(frenet_state.s_d);
        start_s.push_back(0.0);

        // end longitudinal state [s_d, s_dd]
        std::vector<double> end_s;
        end_s.push_back(sample_speed);
        end_s.push_back(0.0);

        // generate longitudinal quartic polynomial
        fop::QuarticPolynomial longitudinal_quartic_poly = fop::QuarticPolynomial(start_s, end_s, T);

        // store the this longitudinal trajectory into target_frenet_path
        for (double t = 0.0; t <= T; t += settings_.tick_t)
        {
          target_frenet_path.s.push_back(longitudinal_quartic_poly.calculatePoint(t));
          target_frenet_path.s_d.push_back(longitudinal_quartic_poly.calculateFirstDerivative(t));
          target_frenet_path.s_dd.push_back(longitudinal_quartic_poly.calculateSecondDerivative(t));
          target_frenet_path.s_ddd.push_back(longitudinal_quartic_poly.calculateThirdDerivative(t));
        }

        // calculate the costs
        double speed_diff = 0.0;
        double jerk_s = 0.0;
        double jerk_d = 0.0;

        // encourage driving inbetween the desired speed and current speed
        speed_diff = pow(desired_speed - target_frenet_path.s_d.back(), 2) +
                     0.5 * pow(current_speed - target_frenet_path.s_d.back(), 2);  //! 0.5 factor is to incentivize the speed to be closer to desired speed

        // calculate total squared jerks
        for (int i = 0; i < target_frenet_path.t.size(); i++)
        {
          jerk_s += pow(target_frenet_path.s_ddd[i], 2);
          jerk_d += pow(target_frenet_path.d_ddd[i], 2);
        }

        // encourage longer planning time
        const double planning_time_cost = settings_.k_time * (1 - T / settings_.max_t);

        target_frenet_path.cd = settings_.k_jerk * jerk_d + planning_time_cost +
                                settings_.k_diff * pow(target_frenet_path.d.back() - center_offset, 2);
        target_frenet_path.cs = settings_.k_jerk * jerk_s + planning_time_cost + settings_.k_diff * speed_diff;
        target_frenet_path.cf = settings_.k_lateral * target_frenet_path.cd + settings_.k_longitudinal * target_frenet_path.cs;

        //! Assign the speed of the path
        target_frenet_path.speed = sample_speed;

        //! Initialize curvature check safe before check
        target_frenet_path.curvature_check = true;

        frenet_paths.push_back(target_frenet_path);
      }
    }
  }

  return frenet_paths;
}

std::vector<fop::FrenetPath> FrenetOptimalTrajectoryPlanner::calculateGlobalPaths(
    std::vector<fop::FrenetPath>& frenet_paths_list, fop::Spline2D& cubic_spline)
{
  for (int i = 0; i < frenet_paths_list.size(); i++)
  {
    // std::cout << "Break 1" << std::endl;
    // calculate global positions
    for (int j = 0; j < frenet_paths_list[i].s.size(); j++)
    {
      // std::cout << "Break 1.1" << std::endl;
      fop::VehicleState state = cubic_spline.calculatePosition(frenet_paths_list[i].s[j]);
      // std::cout << "Break 1.2" << std::endl;
      double i_yaw = cubic_spline.calculateYaw(frenet_paths_list[i].s[j]);
      // std::cout << "Break 1.3" << std::endl;
      double di = frenet_paths_list[i].d[j];
      double frenet_x = state.x + di * cos(i_yaw + M_PI / 2.0);
      double frenet_y = state.y + di * sin(i_yaw + M_PI / 2.0);
      frenet_paths_list[i].x.push_back(frenet_x);
      frenet_paths_list[i].y.push_back(frenet_y);
    }
    // calculate yaw and ds
    for (int j = 0; j < frenet_paths_list[i].x.size() - 1; j++)
    {
      double dx = frenet_paths_list[i].x[j+1] - frenet_paths_list[i].x[j];
      double dy = frenet_paths_list[i].y[j+1] - frenet_paths_list[i].y[j];
      frenet_paths_list[i].yaw.push_back(atan2(dy, dx));
      frenet_paths_list[i].ds.push_back(sqrt(dx * dx + dy * dy));
    }

    frenet_paths_list[i].yaw.push_back(frenet_paths_list[i].yaw.back());
    frenet_paths_list[i].ds.push_back(frenet_paths_list[i].ds.back());

    // calculate curvature
    for (int j = 0; j < frenet_paths_list[i].yaw.size() - 1; j++)
    {
      double yaw_diff = frenet_paths_list[i].yaw[j+1] - frenet_paths_list[i].yaw[j];
      yaw_diff = fop::unifyAngleRange(yaw_diff);
      frenet_paths_list[i].c.push_back(yaw_diff / frenet_paths_list[i].ds[j]);
    }
  }

  return frenet_paths_list;
}

/**
 * @brief Check for collisions at each point along a frenet path
 *
 * @param frenet_path the path to check
 * @param obstacles obstacles to check against for collision
 * @return false if there is a collision along the path. Otherwise, true
 */
bool FrenetOptimalTrajectoryPlanner::checkPathCollision(const fop::FrenetPath& frenet_path,
                                                        const autoware_msgs::DetectedObjectArray& obstacles,
                                                        const std::string& margin)
{
  // ROS_INFO("Collision checking start");

  geometry_msgs::Polygon buggy_rect;
  // geometry_msgs::Polygon buggy_hard_margin;
  geometry_msgs::Polygon buggy_soft_margin;

  // double begin = ros::WallTime::now().toSec();

  for (int i = 0; i < frenet_path.x.size(); i++)
  {
    double cost = 0;

    double buggy_center_x = frenet_path.x[i] + fop::Vehicle::Lf() * cos(frenet_path.yaw[i]);
    double buggy_center_y = frenet_path.y[i] + fop::Vehicle::Lf() * sin(frenet_path.yaw[i]);

    buggy_rect = sat_collision_checker_instance.construct_rectangle(buggy_center_x, buggy_center_y,
                                                                    frenet_path.yaw[i], settings_.vehicle_length,
                                                                    settings_.vehicle_width, TRUE_SIZE_MARGIN);

    // buggy_hard_margin = sat_collision_checker_instance.construct_rectangle(frenet_path.x[i], frenet_path.y[i],
    //                                                                 frenet_path.yaw[i], settings_.vehicle_length,
    //                                                                 vehicle_width, HARD_MARGIN);

    buggy_soft_margin = sat_collision_checker_instance.construct_rectangle(
        buggy_center_x, buggy_center_y, frenet_path.yaw[i], settings_.vehicle_length, settings_.vehicle_width,
        settings_.soft_safety_margin);

    for (auto const& object : obstacles.objects)
    {
      if (margin == "no")
      {
        if (sat_collision_checker_instance.check_collision(buggy_rect, object.convex_hull.polygon))
        {
          return false;
        }
      }
      else if (margin == "soft")
      {
        if (sat_collision_checker_instance.check_collision(buggy_soft_margin, object.convex_hull.polygon))
        {
          return false;
        }
      }
    }
  }

  // double end = ros::WallTime::now().toSec();
  // ROS_INFO("Collision checking done in: %f secs", end - begin);
  // std::cout << "END COLLISION CHECK" << count << std::endl;
  return true;
}

/**
 * @brief Checks whether frenet paths are safe to execute based on constraints and whether there is a collision along a
 * path
 *
 * @param frenet_paths_list the vector of paths to check
 * @param obstacles obstacles to check against for collision
 * @return vector containing the safe paths. If there are no safe paths, a dummy vector is returned.
 */
std::vector<fop::FrenetPath>
FrenetOptimalTrajectoryPlanner::checkPaths(const std::vector<fop::FrenetPath>& frenet_paths_list,
                                           const autoware_msgs::DetectedObjectArray& obstacles, int path_size)
{
  safest_paths.clear();
  unsafe_paths.clear();
  close_proximity_paths.clear();

  backup_unchecked_paths.clear();
  backup_safest_paths.clear();
  backup_close_proximity_paths.clear();
  backup_unsafe_paths.clear();

  std::vector<fop::FrenetPath> passed_constraints_paths;
  std::vector<fop::FrenetPath> safe_paths;
  std::vector<fop::FrenetPath> backup_paths;

  bool using_backup_paths = false;

  /* --------------------- Check paths against constraints -------------------- */

  for (auto& frenet_path : frenet_paths_list)
  {
    bool safe = true;
    bool curvature_passed = true;
    for (int j = 0; j < frenet_path.c.size(); j++)
    {
      if (!fop::isLegal(frenet_path.x[j]) || !fop::isLegal(frenet_path.y[j]))
      {
        safe = false;
        std::cout << "Condition 0: Contains ilegal values" << std::endl;
        break;
      }
      else if (frenet_path.s_d[j] > settings_.max_speed)
      {
        safe = false;
        std::cout << "Condition 1: Exceeded Max Speed" << std::endl;
        break;
      }
      else if (frenet_path.s_dd[j] > settings_.max_accel || frenet_path.s_dd[j] < settings_.max_decel)
      {
        safe = false;
        std::cout << "Condition 2: Exceeded Max Acceleration" << std::endl;
        break;
      }
    }

    if (safe)
    {
      //! Do curvature check only on waypoints to be put into path
      for (int j = 0; j < frenet_path.c.size(); j++)
      {
        if (j > 0 && j < path_size)
        {
          if (fabs(frenet_path.c[j] - frenet_path.c[j-1]) > fop::Vehicle::max_curvature(settings_.tick_t))
          {
            // frenet_path.curvature_check = false;
            std::cout << "Exceeded max curvature = " << fop::Vehicle::max_curvature(settings_.tick_t) 
                      << ". Curr curvature rate = " << (frenet_path.c[j] - frenet_path.c[j-1]) << std::endl;
            break;
          }
        }
      }

      // if (frenet_path.curvature_check)
      if (curvature_passed)
      {
        passed_constraints_paths.push_back(frenet_path);
      }
      else
      {
        backup_paths.push_back(frenet_path);
        backup_unchecked_paths.push_back(frenet_path);
      }
    }
    else
    {
      unsafe_paths.push_back(frenet_path);
    }
  }
  std::cout << "Constraints Checked for all Paths, Starting Collision Checking..." << std::endl;

  /* ------------------- Check paths for collisions (Async) ------------------- */

  // std::vector<std::future<bool>> collision_checks;
  // std::vector<std::future<bool>> backup_collision_checks;

  // for (auto const& frenet_path : passed_constraints_paths)
  // {
  //   collision_checks.push_back(std::async(std::launch::async, &FrenetOptimalTrajectoryPlanner::checkPathCollision, this,
  //                                         frenet_path, obstacles, "no"));
  // }
  // std::cout << "Async Collision Checking Done!" << std::endl;

  std::vector<bool> collision_checks(passed_constraints_paths.size(), bool{true});
  for (int i = 0; i < collision_checks.size(); i++)
  {
    // if (collision_checks[i].get() == false)
    if (collision_checks[i] == false)
    {
      unsafe_paths.push_back(passed_constraints_paths[i]);
    }
    else
    {
      safe_paths.push_back(passed_constraints_paths[i]);
    }
  }

  std::cout << "Fake Collision Checking Done!" << std::endl;

  //! If there is no available path from passed_constraints_paths, check backup_paths.
  // if (safe_paths.empty())
  // {
  //   using_backup_paths = true;
  //   backup_unchecked_paths.clear();

  //   std::cout << "No paths passed curvature checks available. Checking backup paths.";
  //   for (auto const& frenet_path : backup_paths)
  //   {
  //     backup_collision_checks.push_back(std::async(
  //         std::launch::async, &FrenetOptimalTrajectoryPlanner::checkPathCollision, this, frenet_path, obstacles, "no"));
  //   }

  //   for (int i = 0; i < backup_collision_checks.size(); i++)
  //   {
  //     if (backup_collision_checks[i].get() == false)
  //     {
  //       backup_unsafe_paths.push_back(backup_paths[i]);
  //     }
  //     else
  //     {
  //       safe_paths.push_back(backup_paths[i]);
  //     }
  //   }
  // }

  //! cost function for safe paths
  // std::vector<std::future<bool>> soft_margin_collision_checks;

  // for (auto const& frenet_path : safe_paths)
  // {
  //   soft_margin_collision_checks.push_back(std::async(
  //       std::launch::async, &FrenetOptimalTrajectoryPlanner::checkPathCollision, this, frenet_path, obstacles, "soft"));
  // }

  // for (int i = 0; i < soft_margin_collision_checks.size(); i++)
  // {
  //   if (soft_margin_collision_checks[i].get() == false)
  //   {
  //     safe_paths[i].cf += settings_.k_obstacle * 100;  // hard code cost, obstacle top priority

  //     if (using_backup_paths)
  //     {
  //       backup_close_proximity_paths.push_back(safe_paths[i]);
  //     }
  //     else
  //     {
  //       close_proximity_paths.push_back(safe_paths[i]);
  //     }
  //   }
  //   else
  //   {
  //     if (using_backup_paths)
  //     {
  //       backup_safest_paths.push_back(safe_paths[i]);
  //     }
  //     else
  //     {
  //       safest_paths.push_back(safe_paths[i]);
  //     }
  //   }
  // }
  /* ------------------------------ Done checking ----------------------------- */

  if (safe_paths.size() > 0)
  {
    return safe_paths;
  }
  else  // No safe paths
  {
    std::vector<fop::FrenetPath> dummy_paths;
    return dummy_paths;
  }
}

/**
 * @brief Find the path with the lowest cost in each area (transiton area, left lane, right lane)
 * NOTE: This function only works properly when sampling from both lanes
 *
 * @param frenet_paths_list vector of paths to sample from
 * @return vector containing the 3 best paths
 */
std::vector<fop::FrenetPath>
FrenetOptimalTrajectoryPlanner::findBestPaths(const std::vector<fop::FrenetPath>& frenet_paths_list)
{
  std::vector<fop::FrenetPath> best_path_list;
  best_path_list.push_back(findBestPath(frenet_paths_list, 0));  // transition area
  best_path_list.push_back(findBestPath(frenet_paths_list, 1));  // left lane

  best_path_list.push_back(findBestPath(frenet_paths_list, 2));  // right lane

  return best_path_list;
}

fop::FrenetPath FrenetOptimalTrajectoryPlanner::findBestPath(
    const std::vector<fop::FrenetPath>& frenet_paths_list, int target_lane_id)
{
  // if best paths list isn't empty
  if (!frenet_paths_list.empty())
  {
    bool found_path_in_target_lane = false;

    double min_cost = 1000000000.0;  // start with a large number
    int best_path_id = 0;
    for (int i = 0; i < frenet_paths_list.size(); i++)
    {
      if (frenet_paths_list[i].lane_id == target_lane_id)
      {
        found_path_in_target_lane = true;

        if (min_cost >= frenet_paths_list[i].cf)
        {
          min_cost = frenet_paths_list[i].cf;
          best_path_id = i;
        }
      }
    }

    if (!found_path_in_target_lane)
    {
      ROS_WARN("NO PATH WITH LANE ID: %d", target_lane_id);
    }
    // std::cout << "Best Path ID: " << best_path_id << std::endl;

    return frenet_paths_list[best_path_id];
  }

  // if best paths list is empty
  else
  {
    // std::cout << "Best Path Is Empty" << std::endl;
    return fop::FrenetPath();
  }
}

}  // namespace fop