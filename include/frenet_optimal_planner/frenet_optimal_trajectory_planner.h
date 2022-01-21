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
  class Setting
  {
  public:
    Setting(){};
    virtual ~Setting(){};

    // parameters
    double max_speed;           // maximum speed [m/s]
    double max_accel;           // maximum acceleration [m/ss]
    double max_decel;           // maximum deceleration [m/ss]
    double max_curvature;       // maximum curvature [rad/m]

    double steering_angle_rate; // [rad/s]

    double centre_offset;       // offset from the center of the lane [m]
    double delta_width;         // road width sampling length [m]

    double max_t;               // max prediction time [m]
    double min_t;               // min prediction time [m]
    double delta_t;             // sampling time increment [s]
    double tick_t;              // time tick [s]

    double target_speed;        // target speed [m/s]
    double delta_speed;         // target speed sampling length [m/s]
    double num_speed_sample;    // sampling number of target speed

    // double hard_safety_margin;  
    double soft_safety_margin;  // soft safety margin [m]
    double vehicle_width;       // vehicle width [m]
    double vehicle_length;      // vehicle length [m]

    // Cost Weights
    double k_jerk;              // jerk cost weight
    double k_time;              // time cost weight
    double k_diff;              // speed and lateral offset cost weight
    double k_lateral;           // lateral overall cost weight
    double k_longitudinal;      // longitudinal overall cost weight
    double k_obstacle;          // obstacle cost weight
  };

  /* --------------------------------- Methods -------------------------------- */

  // Constructors
  FrenetOptimalTrajectoryPlanner(){};
  FrenetOptimalTrajectoryPlanner(Setting settings);

  // Destructor
  virtual ~FrenetOptimalTrajectoryPlanner(){};

  void updateSettings(Setting settings)
  {
    this->settings_ = settings;
  }

  /* Public Functions */
  // Generate reference curve as the frenet s coordinate
  std::pair<Path, Spline2D> generateReferenceCurve(const fop::Lane& lane);

  // Plan for the optimal trajectory
  std::vector<fop::FrenetPath> frenetOptimalPlanning(fop::Spline2D& cubic_spline, const fop::FrenetState& frenet_state, const int lane_id,
                                                     const double left_width, const double right_width, const double current_speed, 
                                                     const autoware_msgs::DetectedObjectArray& obstacles, const bool check_collision, const bool use_async);

  /* ------------------------ variables (visualization) ----------------------- */
  std::vector<fop::FrenetPath> safest_paths;
  std::vector<fop::FrenetPath> unsafe_paths;

  // std::vector<fop::FrenetPath> close_proximity_paths;
  // std::vector<fop::FrenetPath> backup_unchecked_paths;
  // std::vector<fop::FrenetPath> backup_safest_paths;
  // std::vector<fop::FrenetPath> backup_close_proximity_paths;
  // std::vector<fop::FrenetPath> backup_unsafe_paths;

private:
  Setting settings_;
  SATCollisionChecker sat_collision_checker_instance;
  
  /* Private Functions */
  // Sample candidate trajectories
  std::vector<fop::FrenetPath> generateFrenetPaths(const fop::FrenetState& frenet_state, const int lane_id,
                                                   const double center_offset, const double left_bound, const double right_bound, 
                                                   const double desired_speed, const double current_speed);

  // Convert paths from frenet frame to gobal map frame
  std::vector<fop::FrenetPath> calculateGlobalPaths(std::vector<fop::FrenetPath>& frenet_traj_list, fop::Spline2D& cubic_spline);

  // Compute costs for candidate trajectories
  void computeCosts(std::vector<fop::FrenetPath>& frenet_trajs, const double curr_speed);

  // Check for vehicle kinematic constraints
  void checkConstraints(std::vector<fop::FrenetPath>& frenet_traj_list);

  // Check for collisions and calculate obstacle cost
  void checkCollisions(std::vector<fop::FrenetPath>& frenet_traj_list, const autoware_msgs::DetectedObjectArray& obstacles, const bool use_async);
  bool checkTrajCollision(const fop::FrenetPath& frenet_traj, const autoware_msgs::DetectedObjectArray& obstacles, const double margin);
  fop::Path predictTrajectory(const autoware_msgs::DetectedObject& obstacle, const double tick_t, const int steps);

  // Select the best paths for each lane option
  std::vector<fop::FrenetPath> findBestPaths(const std::vector<fop::FrenetPath>& frenet_traj_list);

  // Select the path with the minimum cost
  fop::FrenetPath findBestPath(const std::vector<fop::FrenetPath>& frenet_traj_list, int target_lane_id);
};

}  // namespace fop

#endif  // FRENET_OPTIMAL_TRAJECTORY_PLANNER_H_