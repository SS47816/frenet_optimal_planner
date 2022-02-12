/* frenet_optimal_trajectory_planner.h

  Copyright (C) 2019 SS47816 & Advanced Robotics Center, National University of Singapore & Micron Technology

  Implementation of Optimal trajectory planning in Frenet Coordinate Algorithm
  Using the algorithm described in this paper, https://ieeexplore.ieee.org/document/5509799
*/

#ifndef FRENET_OPTIMAL_TRAJECTORY_PLANNER_H_
#define FRENET_OPTIMAL_TRAJECTORY_PLANNER_H_

#include <cmath>
#include <vector>
#include <iostream>
#include <future>
#include <queue>

#include "frenet.h"
#include "math_utils.h"
#include "quintic_polynomial.h"
#include "quartic_polynomial.h"
#include "spline.h"
#include "vehicle_state.h"
#include "vehicle.h"

// #include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <autoware_msgs/DetectedObjectArray.h>

#include "sat_collision_checker.h"

// #define TRUE_SIZE_LENGTH 3
// #define TRUE_SIZE_MARGIN 0.3

namespace fop
{

class FrenetOptimalTrajectoryPlanner
{
 public:
  struct Setting
  {
   public:
    Setting() {};
    virtual ~Setting() {};

    // General Settings
    double tick_t;              // time tick [s]

    // Sampling Parameters
    double center_offset;       // offset from the center of the lane [m]
    int num_width;              // number of road width samples
    double max_t;               // max prediction time [s]
    double min_t;               // min prediction time [s]
    int num_t;                  // number of time samples
    double highest_speed;       // highest target speed [m/s]
    double lowest_speed;        // lowest target speed [m/s]
    int num_speed;              // number of speed samples

    // Hard Constraints
    double max_speed;           // maximum speed [m/s]
    double max_accel;           // maximum acceleration [m/ss]
    double max_decel;           // maximum deceleration [m/ss]
    double max_curvature;       // maximum curvature [rad/m]
    // double max_jerk_s;          // maximum longitudinal jerk [m/s^3]
    // double max_jerk_d;          // maximum lateral jerk [m/s^3]
    // double steering_angle_rate; // [rad/s]

    // Cost Weights
    double k_jerk;              // jerk cost weight
    double k_time;              // time cost weight
    double k_diff;              // speed and lateral offset cost weight
    double k_lateral;           // lateral overall cost weight
    double k_longitudinal;      // longitudinal overall cost weight
    double k_obstacle;          // obstacle cost weight

    // Collision Parameters
    double soft_safety_margin;  // soft safety margin [m]
    double vehicle_width;       // vehicle width [m]
    double vehicle_length;      // vehicle length [m]
  };

  class TestResult
  {
   public:
    size_t count;
    std::vector<size_t> numbers;
    std::vector<size_t> total_numbers;
    std::vector<double> time;
    std::vector<double> total_time;

    TestResult();
    void updateCount(const std::vector<size_t> numbers, const std::vector<std::chrono::_V2::system_clock::time_point> timestamps);
    void printSummary();
  };

  /* --------------------------------- Methods -------------------------------- */

  // Constructors
  FrenetOptimalTrajectoryPlanner();
  FrenetOptimalTrajectoryPlanner(Setting& settings);

  // Destructor
  virtual ~FrenetOptimalTrajectoryPlanner(){};

  void updateSettings(Setting& settings);

  /* Public Functions */
  // Generate reference curve as the frenet s coordinate
  std::pair<Path, Spline2D> generateReferenceCurve(const fop::Lane& lane);

  // Plan for the optimal trajectory
  std::vector<fop::FrenetPath> frenetOptimalPlanning(fop::Spline2D& cubic_spline, const fop::FrenetState& frenet_state, const int lane_id,
                                                     const double left_width, const double right_width, const double current_speed, 
                                                     const autoware_msgs::DetectedObjectArray& obstacles, const bool check_collision, const bool use_async);
  
  std::shared_ptr<std::vector<fop::FrenetPath>> candidate_trajs_;
  std::priority_queue<FrenetPath, std::vector<FrenetPath>, std::greater<std::vector<FrenetPath>::value_type>> trajs_queue_;

private:
  Setting settings_;
  TestResult test_result_;
  SATCollisionChecker sat_collision_checker_;

  // Sample candidate trajectories
  std::vector<fop::FrenetPath> generateFrenetPaths(const fop::FrenetState& frenet_state, const int lane_id,
                                                   const double left_bound, const double right_bound, const double current_speed);

  // Convert paths from frenet frame to gobal map frame
  int calculateGlobalPaths(std::vector<fop::FrenetPath>& frenet_traj_list, fop::Spline2D& cubic_spline);
  // Compute costs for candidate trajectories
  int computeCosts(std::vector<fop::FrenetPath>& frenet_trajs, const double curr_speed);

  // Check for vehicle kinematic constraints
  bool checkConstraints(FrenetPath& traj);
  // Check for collisions and calculate obstacle cost
  std::vector<Path> predictTrajectories(const autoware_msgs::DetectedObjectArray& obstacles);
  bool checkCollisions(FrenetPath& ego_traj, const std::vector<Path>& obstacle_trajs, 
                       const autoware_msgs::DetectedObjectArray& obstacles, const bool use_async, size_t& num_checks);
  std::pair<bool, int> checkTrajCollision(const FrenetPath& ego_traj, const std::vector<Path>& obstacle_trajs, 
                                          const autoware_msgs::DetectedObjectArray& obstacles, const double margin_lon, const double margin_lat);
};

}  // namespace fop

#endif  // FRENET_OPTIMAL_TRAJECTORY_PLANNER_H_