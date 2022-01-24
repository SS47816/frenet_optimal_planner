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
  std::vector<size_t> numbers;
  std::vector<std::chrono::_V2::system_clock::time_point> timestamps;
  timestamps.emplace_back(std::chrono::high_resolution_clock::now());

  /* --------------------------------- Construction Zone -------------------------------- */
  
  // Sample a list of FrenetPaths
  std::vector<FrenetState> end_states = sampleEndStates(lane_id, left_width, right_width, current_speed);

  for (const auto& end_state : end_states)
  {
    // Generate the Trajectory accordingly
    FrenetPath candidate_traj = generateFrenetPath(start_state, end_state);
    // Convert to the global frame
    convertToGlobalFrame(candidate_traj, cubic_spline);
    // Compute real costs
    computeTrajCost(candidate_traj);
    checkConstraints(candidate_traj);
    
  }

  /* --------------------------------- Construction Zone -------------------------------- */


  // generateFrenetPaths(start_state, end_state);
  candidate_trajs_ = std::make_shared<std::vector<FrenetPath>>();
  numbers.push_back(candidate_trajs_->size());
  timestamps.emplace_back(std::chrono::high_resolution_clock::now());

  // Convert to global paths
  // const int num_conversion_checks = calculateGlobalPaths(*candidate_trajs_, cubic_spline);
  // numbers.push_back(num_conversion_checks);
  timestamps.emplace_back(std::chrono::high_resolution_clock::now());

  // Check the constraints
  // const int num_constraint_checks = checkConstraints(*candidate_trajs_);
  // numbers.push_back(num_constraint_checks);
  timestamps.emplace_back(std::chrono::high_resolution_clock::now());

  // Compute costs
  // const int num_cost_checks = computeCosts(*candidate_trajs_, current_speed);
  // numbers.push_back(num_cost_checks);
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
  std::vector<FrenetPath> best_path_list = findBestPaths(*candidate_trajs_);

  test_result_.updateCount(std::move(numbers), std::move(timestamps));
  test_result_.printSummary();

  return best_path_list;
}

std::vector<FrenetState> FrenetOptimalTrajectoryPlanner::sampleEndStates(const int lane_id, const double left_bound, const double right_bound, const double current_speed)
{
  // list of frenet end states sampled
  std::vector<FrenetState> end_states;

  // Sampling on the lateral direction
  const double delta_width = (left_bound - settings_.center_offset)/((settings_.num_width - 1)/2);
  for (double d = right_bound; d <= left_bound; d += delta_width)  // left being positive
  {
    const double lat_norm = std::max(std::pow(left_bound - settings_.center_offset, 2), std::pow(right_bound - settings_.center_offset, 2));
    const double lat_cost = std::pow(d - settings_.center_offset, 2)/lat_norm;
    // Sampling on the time dimension
    const double delta_t = (settings_.max_t - settings_.min_t)/(settings_.num_t - 1);
    for (double T = settings_.min_t; T <= settings_.max_t; T += delta_t)
    {
      const double time_cost = (1 - T/settings_.max_t);
      // Sampling on the longitudial direction
      const double delta_speed = (settings_.highest_speed - settings_.lowest_speed)/(settings_.num_speed);
      for (double v = settings_.lowest_speed; v <= settings_.highest_speed; v += delta_speed)
      {
        const double speed_cost = pow(settings_.highest_speed - v, 2) + 0.5*pow(current_speed - v, 2);
        
        FrenetState end_state;
        end_state.lane_id = lane_id;
        // end longitudial state [s, s_d, s_dd]
        end_state.s = -1.0;                 // TBD later by polynomial
        end_state.s_d = v;
        end_state.s_dd = 0.0;
        // end lateral state [d, d_d, d_dd]
        end_state.d = d;
        end_state.d_d = 0.0;
        end_state.d_dd = 0.0;
        // end time
        end_state.T = T;
        
        // Pre-computed Cost (or priority)
        end_state.fix_cost = settings_.k_lat * settings_.k_diff*lat_cost 
                           + settings_.k_lon * (settings_.k_time*time_cost + settings_.k_diff*speed_cost);

        end_states.emplace_back(end_state);
      }
    }
  }

  return std::move(end_states);
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
 * @return void
 */
void FrenetOptimalTrajectoryPlanner::checkConstraints(FrenetPath& traj)
{
  bool passed = true;
  for (int j = 0; j < traj.c.size(); j++)
  {
    if (!isLegal(traj.x[j]) || !isLegal(traj.y[j]))
    {
      passed = false;
      // std::cout << "Condition 0: Contains ilegal values" << std::endl;
      break;
    }
    else if (traj.s_d[j] > settings_.max_speed)
    {
      passed = false;
      // std::cout << "Condition 1: Exceeded Max Speed" << std::endl;
      break;
    }
    else if (traj.s_dd[j] > settings_.max_accel || traj.s_dd[j] < settings_.max_decel)
    {
      passed = false;
      // std::cout << "Condition 2: Exceeded Max Acceleration" << std::endl;
      break;
    }
    else if (std::abs(traj.c[j]) > settings_.max_curvature)
    {
      passed = false;
      // std::cout << "Exceeded max curvature = " << settings_.max_curvature
      //           << ". Curr curvature = " << (traj.c[j]) << std::endl;
      break;
    }
  }

  traj.constraint_passed = passed;
}

int FrenetOptimalTrajectoryPlanner::checkCollisions(std::vector<FrenetPath>& frenet_traj_list, const autoware_msgs::DetectedObjectArray& obstacles, const bool use_async)
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
std::pair<bool, int> FrenetOptimalTrajectoryPlanner::checkTrajCollision(const FrenetPath& frenet_traj,
                                                        const autoware_msgs::DetectedObjectArray& obstacles,
                                                        const double margin)
{
  int num_checks = 0;
  geometry_msgs::Polygon vehicle_rect, obstacle_rect;
  for (auto const& object : obstacles.objects)
  {
    // Predict Object's Trajectory
    const int num_steps = frenet_traj.x.size();
    Path object_traj = predictTrajectory(object, settings_.tick_t, num_steps);
    
    // Check for collisions between ego and obstacle trajectories
    for (int i = 0; i < num_steps; i++)
    {
      num_checks++;
      double vehicle_center_x = frenet_traj.x[i] + Vehicle::Lr() * cos(frenet_traj.yaw[i]);
      double vehicle_center_y = frenet_traj.y[i] + Vehicle::Lr() * sin(frenet_traj.yaw[i]);

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

Path FrenetOptimalTrajectoryPlanner::predictTrajectory(const autoware_msgs::DetectedObject& obstacle, const double tick_t, const int steps)
{
  Path obstacle_trajectory;

  obstacle_trajectory.x.push_back(obstacle.pose.position.x);
  obstacle_trajectory.y.push_back(obstacle.pose.position.y);
  tf2::Quaternion q_tf2(obstacle.pose.orientation.x, obstacle.pose.orientation.y,
                        obstacle.pose.orientation.z, obstacle.pose.orientation.w);
  tf2::Matrix3x3 m(q_tf2.normalize());
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  obstacle_trajectory.yaw.push_back(yaw);
  const double v = magnitude(obstacle.velocity.linear.x, obstacle.velocity.linear.y, obstacle.velocity.linear.z);
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