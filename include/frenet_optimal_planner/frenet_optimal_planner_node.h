
/* frenet_optimal_planner_node.h

  Copyright (C) 2019 SS47816 & Advanced Robotics Center, National University of Singapore

  Local Planner ROS Node
  Using the algorithm described in this paper, https://ieeexplore.ieee.org/document/5509799
*/

/**
 * |              |              |
 * |LB +  0  - RB |              |
 * |<--*     *----|------------->|
 * |   +-----+    |              |
 * |   |     |    |              |
 * |   |  *  |    |              |
 * |   |     |    |              |
 * |   +-----+    |              |
 * |   Buggy W    |              |
 *     <----->    |              |
 * |              |              |
 * | L Lane Width | R Lane Width |
 * |<------------>|<------------>|
 * |              |              |
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
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <autoware_msgs/VehicleCmd.h>
#include <autoware_msgs/DetectedObjectArray.h>

#include <dynamic_reconfigure/server.h>
#include <frenet_optimal_planner/frenet_optimal_planner_Config.h>

#include "frenet_optimal_planner/pid.hpp"
#include "frenet_optimal_planner/visualization.cpp"
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
  int current_lane_;
  int target_lane_;

  // Control outputs
  double acceleration_;
  double steering_angle_;
  double current_steering_angle_;
  double ref_speed_ = kph2mps(30);

  // Vehicle's current state
  fop::VehicleState current_state_;    // State of the vehicle baselink
  fop::VehicleState frontaxle_state_;  // State of the vehicle frontaxle
  fop::FrenetState start_state_;       // Starting States for sampling
  
  // Maps and Paths
  fop::Lane lane_;                      // Maps (All the waypoints)
  fop::Lane local_lane_;                // Selected Waypoints
  fop::Path ref_spline_;                // Reference Spline
  fop::Path output_path_;               // Output Path
  std::vector<double> roi_boundaries_;  //[0] = left boundary length in metre, [1] = right boundary length in metre.
  
  // Controllers
  control::PID pid_;

  // subscriber and publishers
  ros::Subscriber odom_sub;
  ros::Subscriber lane_info_sub;
  ros::Subscriber obstacles_sub;

  ros::Publisher output_path_pub;
  ros::Publisher next_path_pub;
  ros::Publisher ref_path_pub;
  ros::Publisher vehicle_cmd_pub;
  ros::Publisher candidate_paths_pub;

  // ROS
  ros::NodeHandle nh;
  ros::Timer timer;
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;
  dynamic_reconfigure::Server<frenet_optimal_planner::frenet_optimal_planner_Config> server;
  dynamic_reconfigure::Server<frenet_optimal_planner::frenet_optimal_planner_Config>::CallbackType f;

  autoware_msgs::DetectedObjectArray obstacles;
  SATCollisionChecker sat_collision_checker_instance;

  // ###################################### Private Functions ######################################

  // Main Function in ROS running primary logics
  void mainTimerCallback(const ros::TimerEvent& timer_event);

  // Functions for subscribing
  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
  void obstaclesCallback(const autoware_msgs::DetectedObjectArray::Ptr& input_obstacles);
  void laneInfoCallback(const nav_msgs::Path::ConstPtr& global_path);

  // Functions fo publishing results
  void publishEmptyPathsAndStop();
  void publishRefSpline(const fop::Path& path);
  void publishOutputPath(const fop::Path& path);
  void publishNextPath(const fop::FrenetPath& frenet_path);
  void publishVehicleCmd(const double accel, const double angle);
  void publishCandidatePaths();

  // Odom Helper Function
  void updateVehicleFrontAxleState();

  // Planner Helper Functions
  bool feedWaypoints();
  void updateStartState();

  std::vector<double> getSamplingWidthFromTargetLane(const int lane_id, const double vehicle_width, const double current_lane_width,
                                                     const double left_lane_width, const double right_lane_width);

  fop::FrenetPath selectLane(const std::vector<fop::FrenetPath>& best_path_list, const int current_lane);

  void concatPath(const fop::FrenetPath& frenet_path, const int path_size, const double wp_max_seperation, const double wp_min_seperation);

  // Stanley Steeing Functions
  bool calculateControlOutput(const int next_wp_id, const fop::VehicleState& frontaxle_state);

  autoware_msgs::DetectedObject transformObjectFrame(const autoware_msgs::DetectedObject& object_input, const geometry_msgs::TransformStamped& transform_stamped);
};

} // namespace fop