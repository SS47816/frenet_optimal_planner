/** frenet_optimal_planner_node.h
 * 
 * Copyright (C) 2022 Shuo SUN & Advanced Robotics Center, National University of Singapore
 * 
 * Apache License 2.0 https://www.apache.org/licenses/LICENSE-2.0
 * 
 * Local Planner ROS Node
 * Using the algorithm described in this paper, https://ieeexplore.ieee.org/document/5509799
 */

#include <cmath>
#include <vector>
#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>

#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <autoware_msgs/VehicleCmd.h>
#include <autoware_msgs/DetectedObjectArray.h>

#include <dynamic_reconfigure/server.h>
#include <frenet_optimal_planner/frenet_optimal_plannerConfig.h>

#include "frenet_optimal_planner/pid.hpp"
#include "frenet_optimal_planner/visualization.hpp"
#include "frenet_optimal_planner/frenet_optimal_trajectory_planner.h"

namespace fop
{

class FrenetOptimalPlannerNode
{
 public:
  // Constructor
  FrenetOptimalPlannerNode();
  // Destructor
  virtual ~FrenetOptimalPlannerNode(){};

  // Planning algorithm instance
  FrenetOptimalTrajectoryPlanner frenet_planner_;

 private:
  // Regnerate path flag
  bool regenerate_flag_;

  // Lane related variables
  int current_lane_id_;
  int target_lane_id_;
  double map_height_;

  // Control outputs
  double acceleration_;
  double steering_angle_;

  // Vehicle's current state
  fop::VehicleState current_state_;    // State of the vehicle baselink
  fop::VehicleState frontaxle_state_;  // State of the vehicle frontaxle

  // Trajectory Sampling States
  fop::FrenetState start_state_;       // Starting States for sampling
  
  // Maps and Trajs
  fop::Lane lane_;                      // Maps (All the waypoints)
  fop::Lane local_lane_;                // Selected Waypoints
  fop::Path ref_spline_;                // Reference Spline
  fop::Path curr_trajectory_;           // Output Trajectory
  std::vector<double> roi_boundaries_;  //[0] = left boundary length in metre, [1] = right boundary length in metre.

  // Controllers
  control::PID pid_;

  // subscriber and publishers
  ros::Subscriber odom_sub;
  ros::Subscriber lane_info_sub;
  ros::Subscriber obstacles_sub;

  ros::Publisher curr_traj_pub;
  ros::Publisher next_traj_pub;
  ros::Publisher ref_path_pub;
  ros::Publisher vehicle_cmd_pub;
  ros::Publisher candidate_paths_pub;
  ros::Publisher obstacles_pub;

  // ROS
  ros::NodeHandle nh;
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;
  dynamic_reconfigure::Server<frenet_optimal_planner::frenet_optimal_plannerConfig> server;
  dynamic_reconfigure::Server<frenet_optimal_planner::frenet_optimal_plannerConfig>::CallbackType f;

  // ###################################### Private Functions ######################################

  // Functions for subscribing
  void laneInfoCallback(const nav_msgs::Path::ConstPtr& global_path);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
  void obstaclesCallback(const autoware_msgs::DetectedObjectArray::ConstPtr& input_obstacles);

  // Functions fo publishing results
  void publishEmptyTrajsAndStop();
  void publishRefSpline(const fop::Path& path);
  void publishCurrTraj(const fop::Path& path);
  void publishNextTraj(const fop::FrenetPath& next_traj);
  void publishVehicleCmd(const double accel, const double angle);
  void publishCandidateTrajs(const std::vector<fop::FrenetPath>& candidate_trajs);

  // Planner Helper Functions
  bool feedWaypoints();
  void updateStartState();
  void transformObjects(autoware_msgs::DetectedObjectArray& output_objects, const autoware_msgs::DetectedObjectArray& input_objects);

  std::vector<double> getSamplingWidthFromTargetLane(const int lane_id, const double vehicle_width, const double current_lane_width,
                                                     const double left_lane_width, const double right_lane_width);
  fop::FrenetPath selectLane(const std::vector<fop::FrenetPath>& best_path_list, const int current_lane);
  void concatPath(const fop::FrenetPath& next_traj, const int traj_max_size, const int traj_min_size, const double wp_max_seperation, const double wp_min_seperation);

  // Stanley Steeing Functions
  void updateVehicleFrontAxleState();
  bool calculateControlOutput(const int next_wp_id, const fop::VehicleState& frontaxle_state);
};

} // namespace fop