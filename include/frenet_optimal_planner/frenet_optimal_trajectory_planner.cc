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

std::pair<Path, Spline2D> FrenetOptimalTrajectoryPlanner::generateReferenceCurve(const Lane& lane)
{
  Path ref_path = Path();
  auto cubic_spline = Spline2D(lane);

  std::vector<double> s;
  for (double i = 0; i < cubic_spline.s_.back(); i += 0.1)
  {
    s.emplace_back(i);
  }

  for (int i = 0; i < s.size(); i++)
  {
    VehicleState state = cubic_spline.calculatePosition(s[i]);
    ref_path.x.emplace_back(state.x);
    ref_path.y.emplace_back(state.y);
    ref_path.yaw.emplace_back(cubic_spline.calculateYaw(s[i]));
  }

  return std::pair<Path, Spline2D>{ref_path, cubic_spline};
}

std::vector<FrenetPath> 
FrenetOptimalTrajectoryPlanner::frenetOptimalPlanning(Spline2D& cubic_spline, const FrenetState& start_state, const int lane_id,
                                                      const double left_width, const double right_width, const double current_speed, 
                                                      const autoware_msgs::DetectedObjectArray& obstacles, const bool check_collision, const bool use_async)
{
  // Initialize a series of results to be recorded
  // std::vector<size_t> numbers;
  // std::vector<std::chrono::_V2::system_clock::time_point> timestamps;
  // timestamps.emplace_back(std::chrono::high_resolution_clock::now());

  /* --------------------------------- Construction Zone -------------------------------- */
  const auto start_time = std::chrono::high_resolution_clock::now();
  const auto obstacle_trajs = predictTrajectories(obstacles);
  const std::chrono::duration<double, std::milli> prediction_time = std::chrono::high_resolution_clock::now() - start_time;
  
  // Sample all the end states in 3 dimensions, [d, v, t]
  std::vector<std::vector<std::vector<FrenetState>>> end_states = sampleEndStates(start_state, lane_id, left_width, right_width, current_speed);
  const std::chrono::duration<double, std::milli> sampling_time = std::chrono::high_resolution_clock::now() - start_time - prediction_time;

  // Record test results
  const int num_states = end_states.size()*end_states[0].size()*end_states[0][0].size();
  int num_trajs = 0;
  int num_collision_checks = 0;
  std::cout << "FOP: Sampled " << num_states << " End States" << num_trajs << std::endl;

  this->candidate_trajs_ = std::make_shared<std::vector<FrenetPath>>();
  // double min_cost = 10000.0;
  
  bool best_traj_found = false;
  bool unchecked_states_remaining = true;
  while (!best_traj_found && unchecked_states_remaining)
  {
    num_trajs++;
    std::cout << "FOP: Search iteration: " << num_trajs << std::endl;

    const auto init_guess = findNextBest(end_states);
    unchecked_states_remaining = init_guess.second; // false for no more unchecked end states

    const auto idx = init_guess.first;
    // Generate the Trajectory accordingly
    FrenetPath candidate_traj = generateFrenetPath(start_state, end_states[idx[0]][idx[1]][idx[2]]);
    // Convert to the global frame
    convertToGlobalFrame(candidate_traj, cubic_spline);
    // Compute real costs
    computeTrajCost(candidate_traj);
    // Check for constraints
    bool is_safe = checkConstraints(candidate_traj);
    std::cout << "FOP: Constraints Checked, is_safe: " << is_safe << std::endl;
    if (!is_safe)
    {
      this->candidate_trajs_->push_back(candidate_traj);
      continue;
    }
    else
    {
      // Check for collisions
      if (check_collision)
      {
        const auto result = checkCollisions(candidate_traj, obstacle_trajs, obstacles, false);
        is_safe = result.first;
        num_collision_checks += result.second;
      }
      else
      {
        std::cout << "Collision Checking Skipped" << std::endl;
      }
      candidate_trajs_->push_back(candidate_traj);
    }
    std::cout << "FOP: Collision Checked, is_safe: " << is_safe << std::endl;
    
    if (is_safe)
    {
      best_traj_found = true;
      std::cout << "FOP: Best Traj Found" << std::endl;
    }
  }

  const std::chrono::duration<double, std::milli> total_time = std::chrono::high_resolution_clock::now() - start_time;

  // Print Summary for this iteration
  std::cout << " " << std::endl;
  std::cout << "Summary: This Planning Iteration" << std::endl;
  std::cout << "Step 0 : Predicted               " << obstacle_trajs.size() << " Trajectories in " << prediction_time.count() << " ms" << std::endl;
  std::cout << "Step 1 : Generated               " << num_states << "  End States  in " << sampling_time.count() << " ms" << std::endl;
  std::cout << "Step 2 : Converted               " << num_trajs << " Trajectories in " << "unknown" << " ms" << std::endl;
  std::cout << "Step 3 : Checked Constraints for " << num_trajs << " Trajectories in " << "unknown" << " ms" << std::endl;
  std::cout << "Step 4 : Computed Cost for       " << num_trajs << " Trajectories in " << "unknown" << " ms" << std::endl;
  std::cout << "Step 5 : Checked Collisions for  " << num_collision_checks << " PolygonPairs in " << "unknown" << " ms" << std::endl;
  std::cout << "Total  : Planning Took           " << total_time.count() << " ms (or " << 1000/total_time.count() << " Hz)" << std::endl;
  
  if (best_traj_found)
  {
    return std::vector<FrenetPath>{1, candidate_trajs_->back()};
  }
  else
  {
    return std::vector<FrenetPath>{};
  }

  /* --------------------------------- Construction Zone -------------------------------- */


  // generateFrenetPaths(start_state, end_state);
  // candidate_trajs_ = std::make_shared<std::vector<FrenetPath>>();
  // numbers.push_back(candidate_trajs_->size());
  // timestamps.emplace_back(std::chrono::high_resolution_clock::now());

  // Convert to global paths
  // const int num_conversion_checks = calculateGlobalPaths(*candidate_trajs_, cubic_spline);
  // numbers.push_back(num_conversion_checks);
  // timestamps.emplace_back(std::chrono::high_resolution_clock::now());

  // Check the constraints
  // const int num_constraint_checks = checkConstraints(*candidate_trajs_);
  // numbers.push_back(num_constraint_checks);
  // timestamps.emplace_back(std::chrono::high_resolution_clock::now());

  // Compute costs
  // const int num_cost_checks = computeCosts(*candidate_trajs_, current_speed);
  // numbers.push_back(num_cost_checks);
  // timestamps.emplace_back(std::chrono::high_resolution_clock::now());

  // Check collisions
  // int num_collision_checks = 0;
  // if (check_collision)
  // {
  //   num_collision_checks = checkCollisions(*candidate_trajs_, obstacles, use_async);
  // }
  // else
  // {
  //   std::cout << "Collision Checking Skipped" << std::endl;
  // }
  // numbers.push_back(num_collision_checks);
  // const auto start_time = std::chrono::high_resolution_clock::now();
  // timestamps.emplace_back(std::chrono::high_resolution_clock::now());

  // Find the path with minimum costs
  // std::vector<FrenetPath> best_path_list = findBestPaths(*candidate_trajs_);

  // test_result_.updateCount(std::move(numbers), std::move(timestamps));
  // test_result_.printSummary();

  // return best_path_list;
}

std::vector<std::vector<std::vector<FrenetState>>> FrenetOptimalTrajectoryPlanner::sampleEndStates(const FrenetState& start_state, const int lane_id, const double left_bound, const double right_bound, const double current_speed)
{
  // list of frenet end states sampled
  std::vector<std::vector<std::vector<FrenetState>>> end_states_3d;
  
  // Sampling on the lateral direction
  const double delta_width = (left_bound - settings_.center_offset)/((settings_.num_width - 1)/2);
  for (int i = 0; i < settings_.num_width; i++)  // left being positive
  {
    std::vector<std::vector<FrenetState>> end_states_2d;
    const double d = right_bound + i*delta_width;
    const double lat_norm = std::max(std::pow(left_bound - settings_.center_offset, 2), std::pow(right_bound - settings_.center_offset, 2));
    const double lat_cost = std::pow(d - settings_.center_offset, 2)/lat_norm;

    // Sampling on the longitudial direction
    const double delta_v = (settings_.highest_speed - settings_.lowest_speed)/(settings_.num_speed - 1);
    for (int j = 0; j < settings_.num_speed; j++)
    {
      std::vector<FrenetState> end_states_1d;
      const double v = settings_.lowest_speed + j*delta_v;
      const double speed_cost = pow(settings_.highest_speed - v, 2) + 0.5*pow(current_speed - v, 2);

      // Sampling on the time dimension
      const double delta_t = (settings_.max_t - settings_.min_t)/(settings_.num_t - 1);
      for (int k = 0; k < settings_.num_t; k++)
      {
        FrenetState end_state;
        end_state.lane_id = lane_id;
        end_state.is_used = false;
        // end time
        end_state.T = settings_.min_t + k*delta_t;
        // end longitudial state [s, s_d, s_dd]
        end_state.s = 0.0;  // TBD later by polynomial
        end_state.s_d = v;
        end_state.s_dd = 0.0;
        // end lateral state [d, d_d, d_dd]
        end_state.d = d;
        end_state.d_d = 0.0;
        end_state.d_dd = 0.0;
        
        // Pre-computed Cost (or priority)
        const double time_cost = (1 - end_state.T/settings_.max_t);
        // fixed cost terms
        end_state.fix_cost = settings_.k_lat * settings_.k_diff*lat_cost 
                           + settings_.k_lon * (settings_.k_time*time_cost + settings_.k_diff*speed_cost);
        // estimated huristic cost terms
        end_state.est_cost = settings_.k_lat * settings_.k_diff*pow(start_state.d - end_state.d, 2);

        end_states_1d.emplace_back(end_state);
      }

      end_states_2d.emplace_back(end_states_1d);
    }

    end_states_3d.emplace_back(end_states_2d);
  }

  return std::move(end_states_3d);
}

std::pair<std::vector<int>, bool> FrenetOptimalTrajectoryPlanner::findNextBest(std::vector<std::vector<std::vector<FrenetState>>>& end_states)
{
  double min_cost = 10000.0;
  std::vector<int> indices = std::vector<int>{3, int(0)};
  bool found = false;
  for (int i = 0; i < end_states.size(); i++)
  {
    for (int j = 0; j < end_states[0].size(); j++)
    {
      for (int k = 0; k < end_states[0][0].size(); k++)
      {
        if (end_states[i][j][k].is_used)
        {  
          continue;
        }
        else if (end_states[i][j][k].final_cost < min_cost)
        {
          found = true;
          indices[0] = i;
          indices[1] = j;
          indices[2] = k;
          min_cost = end_states[i][j][k].final_cost;
        }
      }
    }
  }
  // mark this end state as used
  end_states[indices[0]][indices[1]][indices[2]].is_used = true;
  return std::pair<std::vector<int>, bool>{indices, found};
}

FrenetPath FrenetOptimalTrajectoryPlanner::generateFrenetPath(const FrenetState& start_state, const FrenetState& end_state)
{
  // frenet path to be generated
  FrenetPath frenet_traj = FrenetPath();
  frenet_traj.lane_id = end_state.lane_id;

  // generate lateral quintic polynomial
  QuinticPolynomial lateral_quintic_poly = QuinticPolynomial(start_state, end_state);

  // store the this lateral trajectory into frenet_traj
  for (double t = 0.0; t <= end_state.T; t += settings_.tick_t)
  {
    frenet_traj.t.emplace_back(t);
    frenet_traj.d.emplace_back(lateral_quintic_poly.calculatePoint(t));
    frenet_traj.d_d.emplace_back(lateral_quintic_poly.calculateFirstDerivative(t));
    frenet_traj.d_dd.emplace_back(lateral_quintic_poly.calculateSecondDerivative(t));
    frenet_traj.d_ddd.emplace_back(lateral_quintic_poly.calculateThirdDerivative(t));
  }

  // generate longitudinal quartic polynomial
  QuarticPolynomial longitudinal_quartic_poly = QuarticPolynomial(start_state, end_state);

  // store the this longitudinal trajectory into frenet_traj
  for (double t = 0.0; t <= end_state.T; t += settings_.tick_t)
  {
    frenet_traj.s.emplace_back(longitudinal_quartic_poly.calculatePoint(t));
    frenet_traj.s_d.emplace_back(longitudinal_quartic_poly.calculateFirstDerivative(t));
    frenet_traj.s_dd.emplace_back(longitudinal_quartic_poly.calculateSecondDerivative(t));
    frenet_traj.s_ddd.emplace_back(longitudinal_quartic_poly.calculateThirdDerivative(t));
  }

  // Pass the pre-computed costs to trajectory
  frenet_traj.fix_cost = end_state.fix_cost;
  frenet_traj.est_cost = end_state.est_cost;
  
  return frenet_traj;
}

void FrenetOptimalTrajectoryPlanner::convertToGlobalFrame(FrenetPath& frenet_traj, Spline2D& cubic_spline)
{
  // calculate global positions
  for (int j = 0; j < frenet_traj.s.size(); j++)
  {
    VehicleState state = cubic_spline.calculatePosition(frenet_traj.s[j]);
    double i_yaw = cubic_spline.calculateYaw(frenet_traj.s[j]);
    const double di = frenet_traj.d[j];
    const double frenet_x = state.x + di * cos(i_yaw + M_PI / 2.0);
    const double frenet_y = state.y + di * sin(i_yaw + M_PI / 2.0);
    if (!isLegal(frenet_x) || !isLegal(frenet_y))
    {
      break;
    }
    else
    {
      frenet_traj.x.emplace_back(frenet_x);
      frenet_traj.y.emplace_back(frenet_y);
    }
  }
  // calculate yaw and ds
  for (int j = 0; j < frenet_traj.x.size() - 1; j++)
  {
    const double dx = frenet_traj.x[j+1] - frenet_traj.x[j];
    const double dy = frenet_traj.y[j+1] - frenet_traj.y[j];
    frenet_traj.yaw.emplace_back(atan2(dy, dx));
    frenet_traj.ds.emplace_back(sqrt(dx * dx + dy * dy));
  }

  frenet_traj.yaw.emplace_back(frenet_traj.yaw.back());
  frenet_traj.ds.emplace_back(frenet_traj.ds.back());

  // calculate curvature
  for (int j = 0; j < frenet_traj.yaw.size() - 1; j++)
  {
    double yaw_diff = unifyAngleRange(frenet_traj.yaw[j+1] - frenet_traj.yaw[j]);
    frenet_traj.c.emplace_back(yaw_diff / frenet_traj.ds[j]);
  }
}

void FrenetOptimalTrajectoryPlanner::computeTrajCost(FrenetPath& traj)
{
  // calculate the costs
  double jerk_s = 0.0;
  double jerk_d = 0.0;
  // calculate total squared jerks
  for (int i = 0; i < traj.t.size(); i++)
  {
    jerk_s += std::pow(traj.s_ddd[i], 2);
    jerk_d += std::pow(traj.d_ddd[i], 2);
  }

  traj.dyn_cost = settings_.k_jerk * (settings_.k_lon * jerk_s + settings_.k_lat * jerk_d);
  traj.final_cost = traj.fix_cost + traj.dyn_cost;
}

/**
 * @brief Checks whether frenet paths are safe to execute based on constraints 
 * @param traj the trajectory to be checked
 * @return true if trajectory satisfies constraints
 */
bool FrenetOptimalTrajectoryPlanner::checkConstraints(FrenetPath& traj)
{
  bool passed = true;
  for (int i = 0; i < traj.c.size(); i++)
  {
    if (!std::isnormal(traj.x[i]) || !std::isnormal(traj.y[i]))
    {
      passed = false;
      std::cout << "Condition 0: Contains ilegal values" << std::endl;
      break;
    }
    else if (traj.s_d[i] > settings_.max_speed)
    {
      passed = false;
      std::cout << "Condition 1: Exceeded Max Speed" << std::endl;
      break;
    }
    else if (traj.s_dd[i] > settings_.max_accel || traj.s_dd[i] < settings_.max_decel)
    {
      passed = false;
      std::cout << "Condition 2: Exceeded Max Acceleration" << std::endl;
      break;
    }
    else if (std::abs(traj.c[i]) > settings_.max_curvature)
    {
      passed = false;
      std::cout << "Exceeded max curvature = " << settings_.max_curvature
                << ". Curr curvature = " << (traj.c[i]) << std::endl;
      break;
    }
  }

  traj.constraint_passed = passed;
  return passed;
}

std::vector<Path> FrenetOptimalTrajectoryPlanner::predictTrajectories(const autoware_msgs::DetectedObjectArray& obstacles)
{
  std::vector<Path> obstacle_trajs;

  for (const auto& obstacle : obstacles.objects)
  {
    Path obstacle_traj;

    obstacle_traj.x.push_back(obstacle.pose.position.x);
    obstacle_traj.y.push_back(obstacle.pose.position.y);
    tf2::Quaternion q_tf2(obstacle.pose.orientation.x, obstacle.pose.orientation.y,
                          obstacle.pose.orientation.z, obstacle.pose.orientation.w);
    tf2::Matrix3x3 m(q_tf2.normalize());
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    obstacle_traj.yaw.push_back(yaw);
    const double v = magnitude(obstacle.velocity.linear.x, obstacle.velocity.linear.y, obstacle.velocity.linear.z);
    obstacle_traj.v.push_back(v);
    
    const int steps = settings_.max_t/settings_.tick_t;
    for (size_t i = 0; i < steps; i++)
    {
      obstacle_traj.x.push_back(obstacle_traj.x.back() + v*settings_.tick_t*std::cos(yaw));
      obstacle_traj.x.push_back(obstacle_traj.y.back() + v*settings_.tick_t*std::sin(yaw));
      obstacle_traj.yaw.push_back(yaw);
      obstacle_traj.v.push_back(v);
    }

    obstacle_trajs.emplace_back(obstacle_traj);
  }

  return obstacle_trajs;
}

std::pair<bool, int> FrenetOptimalTrajectoryPlanner::checkCollisions(FrenetPath& ego_traj, const std::vector<Path>& obstacle_trajs, 
                                                                     const autoware_msgs::DetectedObjectArray& obstacles, const bool use_async)
{
  int num_checks = 0;
  if (use_async)
  {
    std::future<std::pair<bool, int>> collision_check = std::async(std::launch::async, &FrenetOptimalTrajectoryPlanner::checkTrajCollision, this, 
                                                                   ego_traj, obstacle_trajs, obstacles, settings_.safety_margin_lon, settings_.safety_margin_lat);
    const auto result = collision_check.get();
    ego_traj.collision_passed = result.first;
    num_checks += result.second;
    std::cout << "Async Collision Checking" << std::endl;
  }
  else
  {
    const auto result = checkTrajCollision(ego_traj, obstacle_trajs, obstacles, settings_.safety_margin_lon, settings_.safety_margin_lat);
    ego_traj.collision_passed = result.first;
    num_checks += result.second;
    std::cout << "Sync Collision Checking" << std::endl;
  }

  return std::pair<bool, int>{ego_traj.collision_passed, num_checks};
}

/**
 * @brief Check for collisions at each point along a frenet path
 *
 * @param ego_traj the path to check
 * @param obstacles obstacles to check against for collision
 * @param margin collision margin in [m]
 * @return false if there is a collision along the path. Otherwise, true
 */
std::pair<bool, int> FrenetOptimalTrajectoryPlanner::checkTrajCollision(const FrenetPath& ego_traj, const std::vector<Path>& obstacle_trajs,
                                                                        const autoware_msgs::DetectedObjectArray& obstacles,
                                                                        const double margin_lon, const double margin_lat)
{
  int num_checks = 0;
  geometry_msgs::Polygon ego_rect, obstacle_rect;
  for (int i = 0; i < obstacles.objects.size(); i++)
  {
    const int num_steps = ego_traj.x.size();
    // Check for collisions between ego and obstacle trajectories
    for (int j = 0; j < num_steps; j++)
    {
      num_checks++;
      const double vehicle_center_x = ego_traj.x[j] + Vehicle::Lr() * cos(ego_traj.yaw[j]);
      const double vehicle_center_y = ego_traj.y[j] + Vehicle::Lr() * sin(ego_traj.yaw[j]);

      ego_rect = sat_collision_checker_.construct_rectangle(vehicle_center_x, vehicle_center_y, ego_traj.yaw[j], 
                                                            settings_.vehicle_length, settings_.vehicle_width, 0.0, 0.0);
      obstacle_rect = sat_collision_checker_.construct_rectangle(obstacle_trajs[i].x[j], obstacle_trajs[i].y[j], obstacle_trajs[i].yaw[j], 
                                                                 obstacles.objects[i].dimensions.x, obstacles.objects[i].dimensions.y, margin_lon, margin_lat);

      if (sat_collision_checker_.check_collision(ego_rect, obstacle_rect))
      {
        return std::pair<bool, int>{false, num_checks};
      }
    }
  }

  return std::pair<bool, int>{true, num_checks};
}

/**
 * @brief Find the path with the lowest cost in each area (transiton area, left lane, right lane)
 * NOTE: This function only works properly when sampling from both lanes
 *
 * @param frenet_traj_list vector of paths to sample from
 * @return vector containing the 3 best paths
 */
std::vector<FrenetPath>
FrenetOptimalTrajectoryPlanner::findBestPaths(const std::vector<FrenetPath>& frenet_traj_list)
{
  std::vector<FrenetPath> best_path_list;
  best_path_list.emplace_back(findBestPath(frenet_traj_list, LaneID::RIGHT_LANE));
  best_path_list.emplace_back(findBestPath(frenet_traj_list, LaneID::CURR_LANE));
  best_path_list.emplace_back(findBestPath(frenet_traj_list, LaneID::LEFT_LANE));

  return best_path_list;
}

FrenetPath FrenetOptimalTrajectoryPlanner::findBestPath(const std::vector<FrenetPath>& frenet_traj_list, int target_lane_id)
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

          if (min_cost >= frenet_traj_list[i].final_cost)
          {
            min_cost = frenet_traj_list[i].final_cost;
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
    return FrenetPath();
  }
}

}  // namespace fop