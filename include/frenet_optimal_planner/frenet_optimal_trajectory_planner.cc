/* frenet_optimal_trajectory_planner.cc

  Copyright (C) 2019 SS47816 & Advanced Robotics Center, National University of Singapore & Micron Technology

  Implementation of Optimal trajectory planning in Frenet Coordinate Algorithm
  Using the algorithm described in this paper, https://ieeexplore.ieee.org/document/5509799
*/

#include "frenet_optimal_trajectory_planner.h"

namespace fop
{

FrenetOptimalTrajectoryPlanner::TestResult::TestResult() : count(0)
{
  this->numbers = std::vector<size_t>(5, size_t(0));
  this->total_numbers = std::vector<size_t>(5, size_t(0));
  this->time = std::vector<double>(6, double(0));
  this->total_time = std::vector<double>(6, double(0));
  this->numbers.shrink_to_fit();
  this->total_numbers.shrink_to_fit();
  this->time.shrink_to_fit();
  this->total_time.shrink_to_fit();
}

void FrenetOptimalTrajectoryPlanner::TestResult::updateCount(const std::vector<size_t> numbers, const std::vector<std::chrono::_V2::system_clock::time_point> timestamps)
{
  if (numbers.size() != 5 || timestamps.size() != 6)
  {
    std::cout << "Recorded TestResult for this planning iteration is invalid" << std::endl;
    return;
  }
  
  this->count++;

  // Update the numbers for the current iteration
  this->numbers = numbers;

  // Add the current numbers to total numbers
  std::transform(this->total_numbers.begin(), this->total_numbers.end(), numbers.begin(), this->total_numbers.begin(), std::plus<size_t>());
  
  // Calculate the elapsed_time for the current iteration, in [ms]
  for (int i = 0; i < timestamps.size() - 1; i++)
  {
    const std::chrono::duration<double, std::milli> elapsed_time = timestamps[i+1] - timestamps[i];
    this->time[i] = elapsed_time.count();
  }
  const std::chrono::duration<double, std::milli> elapsed_time = timestamps.back() - timestamps.front();
  this->time.back() = elapsed_time.count();

  // Add the current elapsed_time to total time, in [ms]
  std::transform(this->total_time.begin(), this->total_time.end(), this->time.begin(), this->total_time.begin(), std::plus<double>());
}

void FrenetOptimalTrajectoryPlanner::TestResult::printSummary()
{
  // Print Summary for this iteration
  std::cout << " " << std::endl;
  std::cout << "Summary: This Planning Iteration (iteration no." << this->count << ")" << std::endl;
  std::cout << "Step 1 : Generated               " << this->numbers[0] << " Trajectories in " << this->time[0] << " ms" << std::endl;
  std::cout << "Step 2 : Converted               " << this->numbers[1] << " Trajectories in " << this->time[1] << " ms" << std::endl;
  std::cout << "Step 3 : Checked Constraints for " << this->numbers[2] << " Trajectories in " << this->time[2] << " ms" << std::endl;
  std::cout << "Step 4 : Computed Cost for       " << this->numbers[3] << " Trajectories in " << this->time[3] << " ms" << std::endl;
  std::cout << "Step 5 : Checked Collisions for  " << this->numbers[4] << " PolygonPairs in " << this->time[4] << " ms" << std::endl;
  std::cout << "Total  : Planning Took           " << this->time[5] << " ms (or " << 1000/this->time[5] << " Hz)" << std::endl;

  // Print Summary for average performance
  std::cout << " " << std::endl;
  std::cout << "Summary: Average Performance (" << this->count << " iterations so far)" << std::endl;
  std::cout << "Step 1 : Generated               " << this->total_numbers[0]/this->count << " Trajectories in " << this->total_time[0]/this->count << " ms" << std::endl;
  std::cout << "Step 2 : Converted               " << this->total_numbers[1]/this->count << " Trajectories in " << this->total_time[1]/this->count << " ms" << std::endl;
  std::cout << "Step 3 : Checked Constraints for " << this->total_numbers[2]/this->count << " Trajectories in " << this->total_time[2]/this->count << " ms" << std::endl;
  std::cout << "Step 4 : Computed Cost for       " << this->total_numbers[3]/this->count << " Trajectories in " << this->total_time[3]/this->count << " ms" << std::endl;
  std::cout << "Step 5 : Checked Collisions for  " << this->total_numbers[4]/this->count << " PolygonPairs in " << this->total_time[4]/this->count << " ms" << std::endl;
  std::cout << "Total  : Planning Took           " << this->total_time[5]/this->count << " ms (or " << 1000/this->time[5] << " Hz)" << std::endl;
}

FrenetOptimalTrajectoryPlanner::FrenetOptimalTrajectoryPlanner()
{
  this->settings_ = Setting();
  this->test_result_ = TestResult();
}

FrenetOptimalTrajectoryPlanner::FrenetOptimalTrajectoryPlanner(FrenetOptimalTrajectoryPlanner::Setting& settings)
{
  this->settings_ = settings;
  this->test_result_ = TestResult();
}

void FrenetOptimalTrajectoryPlanner::updateSettings(Setting& settings)
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

std::vector<fop::FrenetPath> 
FrenetOptimalTrajectoryPlanner::frenetOptimalPlanning(fop::Spline2D& cubic_spline, const fop::FrenetState& frenet_state, const int lane_id,
                                                      const double left_width, const double right_width, const double current_speed, 
                                                      const autoware_msgs::DetectedObjectArray& obstacles, const bool check_collision, const bool use_async)
{
  // Initialize a series of results to be recorded
  std::vector<size_t> numbers;
  std::vector<std::chrono::_V2::system_clock::time_point> timestamps;
  timestamps.emplace_back(std::chrono::high_resolution_clock::now());
  
  // Sample a list of FrenetPaths
  candidate_trajs_ = std::make_shared<std::vector<fop::FrenetPath>>(generateFrenetPaths(frenet_state, lane_id, left_width, right_width, current_speed));
  numbers.push_back(candidate_trajs_->size());
  timestamps.emplace_back(std::chrono::high_resolution_clock::now());

  // Convert to global paths
  const int num_conversion_checks = calculateGlobalPaths(*candidate_trajs_, cubic_spline);
  numbers.push_back(num_conversion_checks);
  timestamps.emplace_back(std::chrono::high_resolution_clock::now());

  // Check the constraints
  const int num_constraint_checks = checkConstraints(*candidate_trajs_);
  numbers.push_back(num_constraint_checks);
  timestamps.emplace_back(std::chrono::high_resolution_clock::now());

  // Compute costs
  const int num_cost_checks = computeCosts(*candidate_trajs_, current_speed);
  numbers.push_back(num_cost_checks);
  timestamps.emplace_back(std::chrono::high_resolution_clock::now());

  // Check collisions
  int num_collision_checks = 0;
  if (check_collision)
  {
    num_collision_checks = checkCollisions(*candidate_trajs_, obstacles, use_async);
  }
  else
  {
    std::cout << "Collision Checking Skipped" << std::endl;
  }
  numbers.push_back(num_collision_checks);
  const auto start_time = std::chrono::high_resolution_clock::now();
  timestamps.emplace_back(std::chrono::high_resolution_clock::now());

  // Find the path with minimum costs
  std::vector<fop::FrenetPath> best_path_list = findBestPaths(*candidate_trajs_);

  test_result_.updateCount(std::move(numbers), std::move(timestamps));
  test_result_.printSummary();

  return best_path_list;
}

// std::vector<FrenetState> FrenetOptimalTrajectoryPlanner::sampleEndStates(const fop::FrenetState& frenet_state, const int lane_id,
//                                                                          const std::pair<double, double> lat_bounds,
//                                                                          const double desired_speed, const double current_speed)
// {
//   std::vector<FrenetState> end_states;
//   std::vector<double> goal_ds;

//   // generate different goals with a lateral offset
//   const double start_d = 0.0 + settings_.center_offset - 
//   for (double d = 0.0 + center_offset; d <= lat_bounds.first; d += settings_.delta_width)  // left being positive
//   {
//     end_states
//     goal_ds.push_back(d);
//   }
//   for (double d = 0.0 + center_offset - settings_.delta_width; d >= lat_bounds.second; d -= settings_.delta_width)  // right being negative
//   {
//     goal_ds.push_back(d);
//   }

//   for (auto goal_d : goal_ds)
//   {
//     // generate d_t polynomials
//     int t_count = 0;
//     for (double T = settings_.min_t; T <= settings_.max_t; T += settings_.delta_t)
//     {
//       t_count++;
      
//       fop::FrenetPath frenet_traj = fop::FrenetPath();
//       frenet_traj.lane_id = lane_id;

//       // start lateral state [d, d_d, d_dd]
//       std::vector<double> start_d;
//       start_d.emplace_back(frenet_state.d);
//       start_d.emplace_back(frenet_state.d_d);
//       start_d.emplace_back(frenet_state.d_dd);

//       // end lateral state [d, d_d, d_dd]
//       std::vector<double> end_d;
//       end_d.emplace_back(goal_d);
//       end_d.emplace_back(0.0);
//       end_d.emplace_back(0.0);

//       // generate lateral quintic polynomial
//       fop::QuinticPolynomial lateral_quintic_poly = fop::QuinticPolynomial(start_d, end_d, T);

//       // store the this lateral trajectory into frenet_traj
//       for (double t = 0.0; t <= T; t += settings_.tick_t)
//       {
//         frenet_traj.t.emplace_back(t);
//         frenet_traj.d.emplace_back(lateral_quintic_poly.calculatePoint(t));
//         frenet_traj.d_d.emplace_back(lateral_quintic_poly.calculateFirstDerivative(t));
//         frenet_traj.d_dd.emplace_back(lateral_quintic_poly.calculateSecondDerivative(t));
//         frenet_traj.d_ddd.emplace_back(lateral_quintic_poly.calculateThirdDerivative(t));
//       }

//       // generate longitudinal quartic polynomial
//       const double min_sample_speed = settings_.target_speed - (settings_.num_speed_sample - 1)*settings_.delta_speed;
//       const double max_sample_speed = settings_.target_speed;
//       for (double sample_speed = min_sample_speed; sample_speed <= max_sample_speed; sample_speed += settings_.delta_speed)
//       {
//         if (sample_speed <= 0)  // ensure target speed is positive
//         {
//           continue;
//         }

//         // copy the longitudinal path over
//         fop::FrenetPath target_frenet_traj = frenet_traj;

//         // start longitudinal state [s, s_d, s_dd]
//         std::vector<double> start_s;
//         start_s.emplace_back(frenet_state.s);
//         start_s.emplace_back(frenet_state.s_d);
//         start_s.emplace_back(0.0);

//         // end longitudinal state [s_d, s_dd]
//         std::vector<double> end_s;
//         end_s.emplace_back(sample_speed);
//         end_s.emplace_back(0.0);

//         // generate longitudinal quartic polynomial
//         fop::QuarticPolynomial longitudinal_quartic_poly = fop::QuarticPolynomial(start_s, end_s, T);

//         // store the this longitudinal trajectory into target_frenet_traj
//         for (double t = 0.0; t <= T; t += settings_.tick_t)
//         {
//           target_frenet_traj.s.emplace_back(longitudinal_quartic_poly.calculatePoint(t));
//           target_frenet_traj.s_d.emplace_back(longitudinal_quartic_poly.calculateFirstDerivative(t));
//           target_frenet_traj.s_dd.emplace_back(longitudinal_quartic_poly.calculateSecondDerivative(t));
//           target_frenet_traj.s_ddd.emplace_back(longitudinal_quartic_poly.calculateThirdDerivative(t));
//         }
        
//         frenet_trajs.emplace_back(target_frenet_traj);
//       }
//     }
//   }

//   return frenet_trajs;
// }

std::vector<fop::FrenetPath> FrenetOptimalTrajectoryPlanner::generateFrenetPaths(const fop::FrenetState& frenet_state, const int lane_id,
                                                                                 const double left_bound, const double right_bound, const double current_speed)
{
  // list of frenet paths generated
  std::vector<fop::FrenetPath> frenet_trajs;
  std::vector<double> goal_ds;

  // generate different goals with a lateral offset
  const double delta_width = (left_bound - settings_.center_offset)/((settings_.num_width - 1)/2);
  for (double d = right_bound; d <= left_bound; d += delta_width)  // left being positive
  {
    goal_ds.push_back(d);
  }

  for (auto goal_d : goal_ds)
  {
    // generate d_t polynomials
    int t_count = 0;
    const double delta_t = (settings_.max_t - settings_.min_t)/(settings_.num_t - 1);
    for (double T = settings_.min_t; T <= settings_.max_t; T += delta_t)
    {
      t_count++;
      
      fop::FrenetPath frenet_traj = fop::FrenetPath();
      frenet_traj.lane_id = lane_id;

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
      const double delta_speed = (settings_.highest_speed - settings_.lowest_speed)/(settings_.num_speed);
      for (double sample_speed = settings_.lowest_speed; sample_speed <= settings_.highest_speed; sample_speed += delta_speed)
      {
        if (sample_speed <= 0)  // ensure target speed is positive
        {
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

int FrenetOptimalTrajectoryPlanner::calculateGlobalPaths(std::vector<fop::FrenetPath>& frenet_traj_list, fop::Spline2D& cubic_spline)
{
  int num_checks = 0;
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

    num_checks++;
  }

  return num_checks;
}

/**
 * @brief Checks whether frenet paths are safe to execute based on constraints and whether there is a collision along a
 * path
 *
 * @param frenet_traj_list the vector of paths to check
 * @param obstacles obstacles to check against for collision
 * @return vector containing the safe paths. If there are no safe paths, a dummy vector is returned.
 */

int FrenetOptimalTrajectoryPlanner::checkConstraints(std::vector<fop::FrenetPath>& frenet_traj_list)
{
  int num_passed = 0;
  int num_checks = 0;
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
        // std::cout << "Exceeded max curvature = " << settings_.max_curvature
        //           << ". Curr curvature = " << (frenet_traj.c[j]) << std::endl;
        break;
      }
    }

    frenet_traj.constraint_passed = passed;
    num_passed += passed? 1 : 0;
    num_checks++;
  }

  std::cout << "Constraints checked for " << frenet_traj_list.size() << " trajectories, with " << num_passed << " passed" << std::endl;
  return num_checks;
}

int FrenetOptimalTrajectoryPlanner::computeCosts(std::vector<fop::FrenetPath>& frenet_trajs, const double curr_speed)
{
  int num_checks = 0;
  for (auto& traj : frenet_trajs)
  {
    if (!traj.constraint_passed)
      continue;

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
    const double speed_diff = pow(settings_.highest_speed - traj.s_d.back(), 2) + 0.5*pow(curr_speed - traj.s_d.back(), 2);

    // encourage longer planning time
    const double planning_time_cost = settings_.k_time * (1 - traj.t.size()*settings_.tick_t / settings_.max_t);

    traj.cd = settings_.k_jerk * jerk_d + planning_time_cost + settings_.k_diff * pow(traj.d.back() - settings_.center_offset, 2);
    traj.cs = settings_.k_jerk * jerk_s + planning_time_cost + settings_.k_diff * speed_diff;
    traj.cf = settings_.k_lateral * traj.cd + settings_.k_longitudinal * traj.cs;
    
    num_checks++;
  }

  return num_checks;
}


int FrenetOptimalTrajectoryPlanner::checkCollisions(std::vector<fop::FrenetPath>& frenet_traj_list, const autoware_msgs::DetectedObjectArray& obstacles, const bool use_async)
{
  const auto start_time = std::chrono::high_resolution_clock::now();
  int num_passed = 0;
  int num_checks = 0;
  if (use_async)
  {
    std::vector<std::future<std::pair<bool, int>>> collision_checks;
    for (auto& frenet_traj : frenet_traj_list)
    {
      collision_checks.emplace_back(
        std::async(std::launch::async, &FrenetOptimalTrajectoryPlanner::checkTrajCollision, this, 
        frenet_traj, obstacles, 0.0));
    }
    for (int i = 0; i < collision_checks.size(); i++)
    {
      const auto result = collision_checks[i].get();
      frenet_traj_list[i].collision_passed = result.first;
      num_passed += frenet_traj_list[i].collision_passed ? 1 : 0;
      num_checks += result.second;
    }

    const auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed_time = end_time - start_time;
    std::cout << "Async Collision Checked " << frenet_traj_list.size() << " trajectories in " 
              << elapsed_time.count() << " ms, with " << num_passed << " passed" << std::endl;
  }
  else
  {
    for (auto& frenet_traj : frenet_traj_list)
    {
      const auto result = checkTrajCollision(frenet_traj, obstacles, 0.0);
      frenet_traj.collision_passed = result.first;
      num_passed += frenet_traj.collision_passed ? 1 : 0;
      num_checks += result.second;
    }

    const auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed_time = end_time - start_time;
    std::cout << "Sync Collision Checked " << frenet_traj_list.size() << " trajectories in " 
              << elapsed_time.count() << " ms, with " << num_passed << " passed" << std::endl;
  }

  return num_checks;
}

/**
 * @brief Check for collisions at each point along a frenet path
 *
 * @param frenet_traj the path to check
 * @param obstacles obstacles to check against for collision
 * @param margin collision margin in [m]
 * @return false if there is a collision along the path. Otherwise, true
 */
std::pair<bool, int> FrenetOptimalTrajectoryPlanner::checkTrajCollision(const fop::FrenetPath& frenet_traj,
                                                        const autoware_msgs::DetectedObjectArray& obstacles,
                                                        const double margin)
{
  int num_checks = 0;
  geometry_msgs::Polygon vehicle_rect, obstacle_rect;
  for (auto const& object : obstacles.objects)
  {
    // Predict Object's Trajectory
    const int num_steps = frenet_traj.x.size();
    fop::Path object_traj = predictTrajectory(object, settings_.tick_t, num_steps);
    
    // Check for collisions between ego and obstacle trajectories
    for (int i = 0; i < num_steps; i++)
    {
      num_checks++;
      double vehicle_center_x = frenet_traj.x[i] + fop::Vehicle::Lr() * cos(frenet_traj.yaw[i]);
      double vehicle_center_y = frenet_traj.y[i] + fop::Vehicle::Lr() * sin(frenet_traj.yaw[i]);

      vehicle_rect = sat_collision_checker_.construct_rectangle(vehicle_center_x, vehicle_center_y, frenet_traj.yaw[i], 
                                                                        settings_.vehicle_length, settings_.vehicle_width, margin);
      obstacle_rect = sat_collision_checker_.construct_rectangle(object_traj.x[i], object_traj.y[i], object_traj.yaw[i], 
                                                                         object.dimensions.x, object.dimensions.y, 0.0);

      if (sat_collision_checker_.check_collision(vehicle_rect, obstacle_rect))
      {
        return std::pair<bool, int>{false, num_checks};
      }
    }
  }

  return std::pair<bool, int>{true, num_checks};
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
  best_path_list.emplace_back(findBestPath(frenet_traj_list, LaneID::RIGHT_LANE));
  best_path_list.emplace_back(findBestPath(frenet_traj_list, LaneID::CURR_LANE));
  best_path_list.emplace_back(findBestPath(frenet_traj_list, LaneID::LEFT_LANE));

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
      std::cout << "No trajectory generated with lane id: " << target_lane_id << std::endl;
    }

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