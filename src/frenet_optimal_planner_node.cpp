
/* frenet_optimal_planner_node.cpp

    Copyright (C) 2019 SS47816 & Advanced Robotics Center, National University of Singapore & Micron Technology

    Local Planner ROS Node
    Using the algorithm described in this paper, https://ieeexplore.ieee.org/document/5509799
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
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <autoware_msgs/DetectedObjectArray.h>
#include <frenet_optimal_planner/LaneInfo.h>
#include <frenet_optimal_planner/SpecialWaypointArray.h>

#include <dynamic_reconfigure/server.h>
#include <frenet_optimal_planner/frenet_optimal_planner_Config.h>

#include "frenet_optimal_planner/frenet.h"
#include "frenet_optimal_planner/vehicle.h"
#include "frenet_optimal_planner/visualization.cpp"
#include "frenet_optimal_planner/lane.h"
#include "frenet_optimal_planner/frenet_optimal_trajectory_planner.h"

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


namespace fop
{
FrenetOptimalTrajectoryPlanner frenet_planner_instance;

// FrenetOptimalTrajectoryPlanner settings
FrenetOptimalTrajectoryPlanner::Setting SETTINGS = FrenetOptimalTrajectoryPlanner::Setting();

// Vehicle Parameters
const double L = fop::Vehicle::L();  // Wheelbase length, back wheel to front wheel
const double MAX_STEERING_ANGLE = fop::Vehicle::max_steering_angle();  // Maximum steering angle
// Constants values used as thresholds (Not for tuning)
const double WP_MAX_SEP = 3.0;                                    // Maximum allowable waypoint separation
const double WP_MIN_SEP = 0.01;                                   // Minimum allowable waypoint separation
const double HEADING_DIFF_THRESH = M_PI / 2;                      // Maximum allowed heading diff between vehicle and path
const double DISTANCE_THRESH = 25.0;                              // Maximum allowed distance between vehicle and path
const double MIN_PLANNING_SPEED = 1.0;                            // Minimum allowed vehicle speed for planning
const int NUM_LOOK_AHEAD_WP = 1;                                  // Number of waypoints to look ahead for Stanley
const double ERROR_VALUE = std::numeric_limits<double>::lowest(); // Error value for return

/* List of dynamic parameters */
// Hyperparameters for output path
double OUTPUT_PATH_MAX_SIZE;  // Maximum size of the output path
double OUTPUT_PATH_MIN_SIZE;  // Minimum size of the output path
double REF_SPLINE_LENGTH;
// Stanley gains
double STANLEY_OVERALL_GAIN;  // Stanley overall gain
double TRACK_ERROR_GAIN;      // Cross track error gain
// Turn signal thresholds
double TURN_YAW_THRESH;       // Yaw difference threshold
// Safety margins for collision check
double LANE_WIDTH;
double LEFT_LANE_WIDTH;       // Maximum left road width [m]
double RIGHT_LANE_WIDTH;      // Maximum right road width [m]
// double NARROW_PATH_OFFSET = -0.3;

double REGENERATE_BRAKE_THRESHOLD = 0.3;

// Dynamic parameter server callback function
void dynamicParamCallback(frenet_optimal_planner::frenet_optimal_planner_Config& config, uint32_t level)
{
  // Hyperparameters for output path
  OUTPUT_PATH_MAX_SIZE = config.output_path_max_size;
  OUTPUT_PATH_MIN_SIZE = config.output_path_min_size;
  REF_SPLINE_LENGTH = config.ref_spline_length;
  // Safety constraints
  SETTINGS.vehicle_width = fop::Vehicle::width();
  SETTINGS.vehicle_length = fop::Vehicle::length();
  SETTINGS.soft_safety_margin = config.soft_safety_margin;
  // Stanley gains
  STANLEY_OVERALL_GAIN = config.stanley_overall_gain;
  TRACK_ERROR_GAIN = config.track_error_gain;
  // Sampling parameters (lateral)
  LANE_WIDTH = config.curr_lane_width;
  LEFT_LANE_WIDTH = config.left_lane_width;
  RIGHT_LANE_WIDTH = config.right_lane_width;
  SETTINGS.centre_offset = config.center_offset;
  SETTINGS.delta_width = config.delta_width;
  // Sampling parameters (longitudinal)
  SETTINGS.max_t = config.max_t;
  SETTINGS.min_t = config.min_t;
  SETTINGS.delta_t = config.delta_t;
  SETTINGS.tick_t = config.tick_t;
  SETTINGS.target_speed = config.target_speed;
  SETTINGS.delta_speed = config.delta_speed;
  SETTINGS.num_speed_sample = config.num_speed_sample;
  // Constraints
  SETTINGS.max_speed = fop::Vehicle::max_speed();
  SETTINGS.max_accel = fop::Vehicle::max_acceleration();
  SETTINGS.max_decel = fop::Vehicle::max_deceleration();
  SETTINGS.max_curvature = fop::Vehicle::max_curvature();
  SETTINGS.steering_angle_rate = fop::Vehicle::steering_angle_rate();
  // Cost Weights
  SETTINGS.k_jerk = config.k_jerk;
  SETTINGS.k_diff = config.k_time;
  SETTINGS.k_diff = config.k_diff;
  SETTINGS.k_lateral = config.k_lateral;
  SETTINGS.k_longitudinal = config.k_longitudinal;
  SETTINGS.k_obstacle = config.k_obstacle;
  // Turn signal thresholds
  TURN_YAW_THRESH = config.turn_yaw_thresh;

  REGENERATE_BRAKE_THRESHOLD = config.regenerate_brake_threshold;

  frenet_planner_instance.updateSettings(SETTINGS);
}

class FrenetOptimalPlannerNode
{
public:
  // Constructor
  FrenetOptimalPlannerNode();

  // Destructor
  virtual ~FrenetOptimalPlannerNode(){};

private:
  // $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ Private Variables $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

  ros::NodeHandle nh;

  // Vehicle's current state
  fop::VehicleState current_state_;    // State of the vehicle baselink
  fop::VehicleState frontaxle_state_;  // State of the vehicle frontaxle
  fop::FrenetState start_state_;       // Starting States for sampling

  autoware_msgs::DetectedObjectArray obstacles;
  SATCollisionChecker sat_collision_checker_instance;

  // Maps and Paths
  fop::Lane lane_;          // Maps (All the waypoints)
  fop::Lane local_lane_;    // Selected Waypoints
  fop::Path ref_spline_;    // Reference Spline
  fop::Path output_path_;   // Output Path

  // Regnerate path flag
  bool regenerate_flag_;

  // Lane related variables
  int current_lane_;
  int target_lane_;
  std::vector<double> roi_boundaries_;  //[0] = left boundary length in metre, [1] = right boundary length in metre. roi
                                        //= region of interest
  int turn_signal_;                     // turn indicator signal, 1 = turn left, -1 = turn right, 0 = not turning

  // output steering angle
  double steering_angle_;

  double current_steering_angle_;

  double behaviour_min_speed_ = 7.0;

  // subscriber and publishers
  ros::Subscriber odom_sub;
  ros::Subscriber lane_info_sub;
  ros::Subscriber behaviour_sub;
  ros::Subscriber cmd_sub;
  ros::Subscriber obstacles_sub;
  ros::Subscriber behaviour_min_speed_sub;
  ros::Subscriber current_steering_angle_sub;
  // ros::Subscriber special_waypoint_sub;

  ros::Publisher output_path_pub;
  ros::Publisher next_path_pub;
  ros::Publisher ref_path_pub;
  ros::Publisher steering_angle_pub;
  ros::Publisher turn_signal_pub;
  ros::Publisher planner_target_speed_pub;
  ros::Publisher candidate_paths_pub;

  // timer
  ros::Timer timer;

  // Variables and Functions for subscribe to odom topic
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;

  dynamic_reconfigure::Server<frenet_optimal_planner::frenet_optimal_planner_Config> server;
  dynamic_reconfigure::Server<frenet_optimal_planner::frenet_optimal_planner_Config>::CallbackType f;

  // ###################################### Private Functions ######################################

  // Main Function in ROS running primary logics
  void mainTimerCallback(const ros::TimerEvent& timer_event);

  // Functions for subscribing
  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
  void objectCallback(const autoware_msgs::DetectedObjectArray::Ptr& input_obstacles);
  // void laneInfoCallback(const frenet_optimal_planner::LaneInfo::ConstPtr& lane_info);
  void laneInfoCallback(const nav_msgs::Path::ConstPtr& global_path);
  void cmdCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);
  void collisionSpeedCallback(const std_msgs::Float64::ConstPtr& min_speed_msg);
  void currSteeringAngleCallback(const std_msgs::Float64::ConstPtr& curr_steering_angle_msg);
  // void specialWaypointCallback(const frenet_optimal_planner::SpecialWaypointArray special_waypoint_msg);

  // Functions fo publishing results
  void publishRefSpline(const fop::Path& path);
  void publishOutputPath(const fop::Path& path);
  void publishNextPath(const fop::FrenetPath& frenet_path);
  void publishEmptyPaths();
  void publishSteeringAngle(const double angle);
  void publishTurnSignal(const fop::FrenetPath& best_path, const bool change_lane, const double yaw_thresh);
  void publishPlannerSpeed(const double speed);
  void publishCandidatePaths();

  // Odom Helper Function
  void updateVehicleFrontAxleState();

  // Planner Helper Functions
  bool feedWaypoints();

  void updateStartState();

  std::vector<double> getSamplingWidthFromTargetLane(const int lane_id, const double vehicle_width,
                                                     const double left_lane_width, const double right_lane_width);

  fop::FrenetPath selectLane(const std::vector<fop::FrenetPath>& best_path_list, const int current_lane);

  void concatPath(const fop::FrenetPath& frenet_path, const int path_size, const double wp_max_seperation, const double wp_min_seperation);

  // Stanley Steeing Functions
  double calculateSteeringAngle(const int next_wp_id, const fop::VehicleState& frontaxle_state);

  autoware_msgs::DetectedObject transformObjectFrame(const autoware_msgs::DetectedObject& object_input,
                                                     const geometry_msgs::TransformStamped& transform_stamped);
};

// Constructor
FrenetOptimalPlannerNode::FrenetOptimalPlannerNode() : tf_listener(tf_buffer)
{
  // Initializing states
  regenerate_flag_ = false;
  turn_signal_ = 0;
  target_lane_ = 0;

  ros::NodeHandle private_nh("~");

  // topics
  std::string odom_topic_;
  std::string obstacle_topic_;
  std::string obstacle_topic_2_;
  std::string lane_info_topic_;
  std::string cmd_topic_;
  std::string current_steering_angle_topic_;

  std::string output_path_topic_;
  std::string next_path_topic_;
  std::string ref_path_topic_;
  std::string steering_angle_topic_;
  std::string turn_signal_topic_;
  // std::string special_waypoint_topic_;

  std::string objects_topic;

  std::string beahviour_min_speed_topic_;
  std::string planner_target_speed_topic_;

  // Hyperparameters
  double planning_frequency_;

  // Dynamic Parameter Server & Function
  f = boost::bind(&dynamicParamCallback, _1, _2);
  server.setCallback(f);

  // Parameters from launch file: topic names
  ROS_ASSERT(private_nh.getParam("odom_topic", odom_topic_));
  ROS_ASSERT(private_nh.getParam("lane_info_topic", lane_info_topic_));
  ROS_ASSERT(private_nh.getParam("cmd_topic", cmd_topic_));
  ROS_ASSERT(private_nh.getParam("behaviour_min_speed_topic", beahviour_min_speed_topic_));
  ROS_ASSERT(private_nh.getParam("current_steering_angle_topic", current_steering_angle_topic_));
  ROS_ASSERT(private_nh.getParam("output_path_topic", output_path_topic_));
  ROS_ASSERT(private_nh.getParam("next_path_topic", next_path_topic_));
  ROS_ASSERT(private_nh.getParam("ref_path_topic", ref_path_topic_));
  ROS_ASSERT(private_nh.getParam("steering_angle_topic", steering_angle_topic_));
  ROS_ASSERT(private_nh.getParam("turn_signal_topic", turn_signal_topic_));
  ROS_ASSERT(private_nh.getParam("planner_target_speed_topic", planner_target_speed_topic_));
  // ROS_ASSERT(private_nh.getParam("special_waypoint_topic", special_waypoint_topic_));

  // Hyperparameters
  ROS_ASSERT(private_nh.getParam("planning_frequency", planning_frequency_));
  ROS_ASSERT(private_nh.getParam("objects_topic", objects_topic));

  candidate_paths_pub = nh.advertise<visualization_msgs::MarkerArray>("candidate_paths", 1);

  // Instantiate FrenetOptimalTrajectoryPlanner
  frenet_planner_instance = FrenetOptimalTrajectoryPlanner(SETTINGS);

  // Subscribe & Advertise
  odom_sub = nh.subscribe(odom_topic_, 1, &FrenetOptimalPlannerNode::odomCallback, this);
  lane_info_sub = nh.subscribe(lane_info_topic_, 1, &FrenetOptimalPlannerNode::laneInfoCallback, this);
  cmd_sub = nh.subscribe(cmd_topic_, 1, &FrenetOptimalPlannerNode::cmdCallback, this);
  obstacles_sub = nh.subscribe(objects_topic, 1, &FrenetOptimalPlannerNode::objectCallback, this);
  behaviour_min_speed_sub = nh.subscribe(beahviour_min_speed_topic_, 1, &FrenetOptimalPlannerNode::collisionSpeedCallback, this);
  current_steering_angle_sub =
      nh.subscribe(current_steering_angle_topic_, 1, &FrenetOptimalPlannerNode::currSteeringAngleCallback, this);
  // special_waypoint_sub = nh.subscribe(special_waypoint_topic_, 1, &FrenetOptimalPlannerNode::specialWaypointCallback, this);
  ref_path_pub = nh.advertise<nav_msgs::Path>(ref_path_topic_, 1);
  output_path_pub = nh.advertise<nav_msgs::Path>(output_path_topic_, 1);
  next_path_pub = nh.advertise<nav_msgs::Path>(next_path_topic_, 1);
  steering_angle_pub = nh.advertise<std_msgs::Float64>(steering_angle_topic_, 1);
  turn_signal_pub = nh.advertise<std_msgs::Int16>(turn_signal_topic_, 1);
  planner_target_speed_pub = nh.advertise<std_msgs::Float64>(planner_target_speed_topic_, 1);

  // timer
  timer = nh.createTimer(ros::Duration(1.0 / planning_frequency_), &FrenetOptimalPlannerNode::mainTimerCallback, this);
};

// Local planner main logic
void FrenetOptimalPlannerNode::mainTimerCallback(const ros::TimerEvent& timer_event)
{
  // Check if all required data are in position
  if (obstacles.objects.size() == 0)
  {
    ROS_FATAL("Local Planner: No obstacles received, No Path Generated");
    publishEmptyPaths();
    return;
  }
  ROS_INFO("Local Planner: Planning Start");

  // Update Waypoints
  if (!feedWaypoints())
  {
    ROS_WARN("Local Planner: Waiting for Waypoints");
    publishEmptyPaths();
    return;
  }
  // Print Waypoints
  // for (size_t i = 0; i < local_lane_.x.size(); i++)
  // {
  // 	std::cout << "waypoint no." << i << ": " << local_lane_.x[i] << " " << local_lane_.y[i] << std::endl;
  // }

  // Update start state
  updateStartState();
  // Get the reference lane's centerline as a spline
  FrenetOptimalTrajectoryPlanner::ResultType result = frenet_planner_instance.generateReferenceCurve(local_lane_);
  // Store the results into reference spline
  ref_spline_.x = result.rx;
  ref_spline_.y = result.ry;
  ref_spline_.yaw = result.ryaw;

  if (ref_spline_.x.empty())
  {
    ROS_ERROR("Local Planner: Reference Curve Is Empty, No Path Generated");
    publishEmptyPaths();
    return;
  }
  publishRefSpline(ref_spline_);  // publish to RVIZ for visualisation
  ROS_INFO("Local Planner: Reference Curve Generated");

  target_lane_ = 0;  //! Sample both lanes

  // Define ROI width for path sampling
  roi_boundaries_ = getSamplingWidthFromTargetLane(target_lane_, SETTINGS.vehicle_width, LEFT_LANE_WIDTH, RIGHT_LANE_WIDTH);

  // Get the planning result (best path of each of the 3 regions, 0 = vehicle transition zone (buggy width), 1 =
  // remaining of left lane, 2 = remaining of right lane
  double final_offset = SETTINGS.centre_offset;
  ROS_DEBUG("final_offset value : %f", final_offset);

  std::vector<fop::FrenetPath> best_path_list = frenet_planner_instance.frenetOptimalPlanning(
      result.cubic_spline, start_state_, final_offset, roi_boundaries_[0], roi_boundaries_[1],
      obstacles, behaviour_min_speed_, current_state_.v, OUTPUT_PATH_MAX_SIZE);

  // Find the best path from the 3 candidates from frenetOptimalPlanning, but still using same cost functions. Behaviour
  // can use this 3 options too choose [NOT IMPLEMENTED 20191213]
  fop::FrenetPath best_path = selectLane(best_path_list, current_lane_);
  ROS_INFO("Local Planner: Best Paths Selected");

  // Concatenate the best path into output_path
  concatPath(best_path, OUTPUT_PATH_MAX_SIZE, WP_MAX_SEP, WP_MIN_SEP);

  publishCandidatePaths();

  // Publish the best paths
  publishNextPath(best_path);       // publish to RVIZ for visualisation
  publishOutputPath(output_path_);  // publish to RVIZ for visualisation

  if (best_path.x.empty()) 
  {
    publishPlannerSpeed(1);
  }
  else
  {
    // Check whether we need to stop from curvature check. Publish speed
    if (!best_path.curvature_check)
    {
      ROS_INFO("Best path fails curvature check");
      if (fabs(current_steering_angle_ - steering_angle_) < fop::deg2rad(2.5))
      {
        publishPlannerSpeed(best_path.speed);
        ROS_INFO("Local Planner: Reached desired steering angle. Buggy starts to move");
      }
      else
      {
        publishPlannerSpeed(0);
        ROS_INFO("Local Planner: Desired steering angle is too high. Buggy stops to turn");
      }
    }
    else
    {
      publishPlannerSpeed(best_path.speed);
    }
  }

  // Publish steering angle
  publishSteeringAngle(steering_angle_);
}

// Update vehicle current state from the tf transform
void FrenetOptimalPlannerNode::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  current_state_.v = magnitude(odom_msg->twist.twist.linear.x, odom_msg->twist.twist.linear.y, odom_msg->twist.twist.linear.z);

  geometry_msgs::TransformStamped transform_stamped;
  try
  {
    transform_stamped = tf_buffer.lookupTransform("map", odom_msg->header.frame_id, ros::Time(0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }

  geometry_msgs::Pose pose_in_map;
  tf2::doTransform(odom_msg->pose.pose, pose_in_map, transform_stamped);
  // Current XY of robot (map frame)
  current_state_.x = pose_in_map.position.x;
  current_state_.y = pose_in_map.position.y;

  tf2::Quaternion q_tf2(pose_in_map.orientation.x, pose_in_map.orientation.y,
                        pose_in_map.orientation.z, pose_in_map.orientation.w);
  q_tf2.normalize();
  tf2::Matrix3x3 m(q_tf2);
  double roll, pitch;
  m.getRPY(roll, pitch, current_state_.yaw);

  updateVehicleFrontAxleState();
}

void FrenetOptimalPlannerNode::objectCallback(const autoware_msgs::DetectedObjectArray::Ptr& input_obstacles)
{
  obstacles.objects.clear();

  geometry_msgs::TransformStamped transform_stamped;
  try
  {
    transform_stamped = tf_buffer.lookupTransform("map", input_obstacles->header.frame_id, ros::Time(0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }

  obstacles.header = input_obstacles->header;
  obstacles.header.frame_id = "map";
  for (auto const& object : input_obstacles->objects)
  {
    autoware_msgs::DetectedObject new_object = object;
    new_object.convex_hull.polygon = sat_collision_checker_instance.remove_top_layer(object.convex_hull.polygon);
    obstacles.objects.push_back(transformObjectFrame(new_object, transform_stamped));
    obstacles.objects.emplace_back(new_object);
  }
}

autoware_msgs::DetectedObject FrenetOptimalPlannerNode::transformObjectFrame(const autoware_msgs::DetectedObject& object_input,
                                                                             const geometry_msgs::TransformStamped& transform_stamped)
{
  autoware_msgs::DetectedObject transformed_object;
  geometry_msgs::Polygon transformed_polygon;

  for (auto const& point : object_input.convex_hull.polygon.points)
  {
    geometry_msgs::Vector3 coord_before_transform, coord_after_transform;
    coord_before_transform.x = point.x;
    coord_before_transform.y = point.y;
    coord_before_transform.z = 0;

    // rotate the object
    tf2::doTransform(coord_before_transform, coord_after_transform, transform_stamped);

    geometry_msgs::Point32 transformed_point;
    transformed_point.x = coord_after_transform.x;
    transformed_point.y = coord_after_transform.y;

    // translate the object
    transformed_point.x += transform_stamped.transform.translation.x;
    transformed_point.y += transform_stamped.transform.translation.y;
    transformed_polygon.points.push_back(transformed_point);
  }

  transformed_object.convex_hull.polygon = transformed_polygon;
  transformed_object.header = object_input.header;
  transformed_object.header.frame_id = "map";

  return transformed_object;
}

// Receive lane info from the lane publisher
// void FrenetOptimalPlannerNode::laneInfoCallback(const frenet_optimal_planner::LaneInfo::ConstPtr& lane_info)
// {
//   lane_ = fop::Lane(lane_info);
// }

void FrenetOptimalPlannerNode::laneInfoCallback(const nav_msgs::Path::ConstPtr& global_path)
{
  lane_ = fop::Lane(global_path, LANE_WIDTH/2, LANE_WIDTH/2, LANE_WIDTH/2 + LEFT_LANE_WIDTH, LANE_WIDTH/2 + RIGHT_LANE_WIDTH);
  ROS_INFO("Local Planner: Lane Info Received, with %d points, filtered to %d points", int(lane_.points.size()), int(lane_.points.size()));
}

// Listen to control output
void FrenetOptimalPlannerNode::cmdCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg)
{
  // regenerate path when braking
  if (cmd_msg->linear.z >= REGENERATE_BRAKE_THRESHOLD)
  {
    regenerate_flag_ = true;
  }
}

void FrenetOptimalPlannerNode::collisionSpeedCallback(const std_msgs::Float64::ConstPtr& min_speed_msg)
{
  behaviour_min_speed_ = min_speed_msg->data;
}

void FrenetOptimalPlannerNode::currSteeringAngleCallback(const std_msgs::Float64::ConstPtr& curr_steering_angle_msg)
{
  current_steering_angle_ = curr_steering_angle_msg->data;
}

// Publish the reference spline (for Rviz only)
void FrenetOptimalPlannerNode::publishRefSpline(const fop::Path& path)
{
  nav_msgs::Path ref_path_msg;
  ref_path_msg.header.frame_id = "map";

  for (size_t i = 0; i < path.yaw.size(); i++)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = path.x[i];
    pose.pose.position.y = path.y[i];
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(path.yaw[i]);
    ref_path_msg.poses.emplace_back(pose);
  }

  ref_path_pub.publish(ref_path_msg);
}

// Publish the current path (for Rviz and MPC)
void FrenetOptimalPlannerNode::publishOutputPath(const fop::Path& path)
{
  nav_msgs::Path output_path_msg;
  output_path_msg.header.frame_id = "map";

  for (size_t i = 0; i < path.yaw.size(); i++)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = path.x[i];
    pose.pose.position.y = path.y[i];
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(path.yaw[i]);
    output_path_msg.poses.emplace_back(pose);
  }

  output_path_pub.publish(output_path_msg);
}

// Publish the best next path (for Rviz only)
void FrenetOptimalPlannerNode::publishNextPath(const fop::FrenetPath& frenet_path)
{
  //! For testing and visualization
  ROS_INFO("path size: %d", (int)frenet_path.c.size());

  nav_msgs::Path output_path_msg;
  output_path_msg.header.frame_id = "map";

  for (size_t i = 0; i < frenet_path.c.size(); i++)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = frenet_path.x[i];
    pose.pose.position.y = frenet_path.y[i];
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(frenet_path.yaw[i]);
    output_path_msg.poses.emplace_back(pose);
  }

  next_path_pub.publish(output_path_msg);
}

void FrenetOptimalPlannerNode::publishPlannerSpeed(const double speed)
{
  double desired_speed;
  if (!output_path_.x.empty() && !output_path_.y.empty())
  {
    const int next_frontlink_wp_id = fop::nextWaypoint(frontaxle_state_, output_path_) + NUM_LOOK_AHEAD_WP;
    if (output_path_.x.size() < next_frontlink_wp_id + 2)
    {
      ROS_INFO("Local Planner: Path is too short. Planner speed = 0.");
      desired_speed = 0;
    }
    else
    {
      desired_speed = speed;
      ROS_INFO("Local Planner: Planner speed published");
      std::cout << "Planner speed = " << desired_speed << std::endl;
    }
  }
  else
  {
    ROS_INFO("Local Planner: No path available. Planner speed = 0.");
    desired_speed = 0;
  }
  std_msgs::Float64 speed_msg;
  speed_msg.data = desired_speed;
  planner_target_speed_pub.publish(speed_msg);
}

/**
 * @brief publish candidate paths for visualization in rviz
 * 
 */
void FrenetOptimalPlannerNode::publishCandidatePaths()
{
  visualization_msgs::MarkerArray candidate_paths_markers = LocalPlannerVisualization::visualizeCandidatePaths(
    frenet_planner_instance.safest_paths, frenet_planner_instance.close_proximity_paths,
    frenet_planner_instance.unsafe_paths, frenet_planner_instance.backup_unchecked_paths,
    frenet_planner_instance.backup_safest_paths, frenet_planner_instance.backup_close_proximity_paths,
    frenet_planner_instance.backup_unsafe_paths);

  candidate_paths_pub.publish(candidate_paths_markers);
}

// Publish empty paths (for Rviz only)
void FrenetOptimalPlannerNode::publishEmptyPaths()
{
  // Publish empty paths
  publishRefSpline(fop::Path());
  publishOutputPath(fop::Path());
  publishNextPath(fop::FrenetPath());
}

// Update the vehicle front axle state (used in odomcallback)
void FrenetOptimalPlannerNode::updateVehicleFrontAxleState()
{
  // Current XY of robot (map frame)
  frontaxle_state_.x = current_state_.x + (L * std::cos(current_state_.yaw));
  frontaxle_state_.y = current_state_.y + (L * std::sin(current_state_.yaw));
  frontaxle_state_.yaw = current_state_.yaw;
  frontaxle_state_.v = current_state_.v;
}

// Feed map waypoints into local map
bool FrenetOptimalPlannerNode::feedWaypoints()
{
  if (lane_.points.empty())
  {
    ROS_WARN("Local Planner: Waiting for Lane Points");
    return false;
  }

  int start_id = fop::lastWaypoint(current_state_, lane_);

  // if reached the end of the lane, stop
  if (start_id >= lane_.points.size() - 2)  // exclude last 2 waypoints for safety, and prevent code crashing
  {
    ROS_WARN("Local Planner: Vehicle is at waypoint no.%d, and %d waypoints in total", start_id, int(lane_.points.size()));
    return false;
  }

  const double dist = fop::distance(lane_.points[start_id].point.x, lane_.points[start_id].point.y, current_state_.x, current_state_.y);
  const double heading_diff = fop::unifyAngleRange(current_state_.yaw - lane_.points[start_id].point.yaw);

  if (dist > DISTANCE_THRESH)
  {
    ROS_WARN("Local Planner: Vehicle's Location Is Too Far From The Target Lane");
    return false;
  }
  else if (std::fabs(heading_diff) > HEADING_DIFF_THRESH)
  {
    ROS_WARN("Local Planner: Vehicle's Is Heading In A Different Direction");
    return false;
  }

  // clear the old waypoints
  local_lane_.clear(); 

  if (start_id > lane_.points.size() - 5)
  {
    start_id = lane_.points.size() - 5;
  }

  // feed the new waypoints
  // for (size_t i = 0; i < 5; i++)
  // {
  //   local_lane_.points.push_back(lane_.points[start_id + i]);
  //   // std::cout << "waypoint no:" << start_id + i << std::endl;
  // }

  // Check if the global waypoints need to be filtered
  if ((lane_.points.back().point.s - lane_.points[start_id].point.s) >= REF_SPLINE_LENGTH)
  {
    // Filter the waypoints 
    double s_cumulative = 0.0;
    local_lane_.points.push_back(lane_.points[start_id]);
    for (size_t i = start_id + 1; i < lane_.points.size(); i++)
    {
      if ((lane_.points[i].point.s - local_lane_.points.back().point.s) >= REF_SPLINE_LENGTH/5.0)
      {
        local_lane_.points.push_back(lane_.points[i]);
      }
    }
  }
  else
  {
    if (lane_.points.size() > 5)
    {
      const int first_id = start_id;                            // 0
      const int third_id = (lane_.points.size() - first_id)/2;  // 2
      const int last_id = lane_.points.size() - 1;              // 4
      const int second_id = (first_id + third_id)/2;            // 1
      const int fourth_id = (third_id + last_id)/2;             // 3

      local_lane_.points.push_back(lane_.points[first_id]);
      local_lane_.points.push_back(lane_.points[second_id]);
      local_lane_.points.push_back(lane_.points[third_id]);
      local_lane_.points.push_back(lane_.points[fourth_id]);
      local_lane_.points.push_back(lane_.points[last_id]);
    }
    else
    {
      // feed the new waypoints
      for (size_t i = 0; i < 5; i++)
      {
        local_lane_.points.push_back(lane_.points[start_id + i]);
      }
    }
  }

  return true;
}

// Update the vehicle start state in frenet
void FrenetOptimalPlannerNode::updateStartState()
{
  if (!local_lane_.points.empty())
  {
    // The new starting state
    // fop::FrenetState new_state;

    // if the current path size is too small, regenerate
    if (output_path_.x.size() < OUTPUT_PATH_MIN_SIZE)
    {
      regenerate_flag_ = true;
    }

    // if need to regenerate the entire path
    if (regenerate_flag_)
    {
      ROS_INFO("Local Planner: Regenerating The Entire Path...");
      // Update the starting state in frenet (using ref_spline_ can produce a finer result compared to local_lane_, but
      // at fringe cases, such as start of code, ref spline might not be available
      start_state_ = ref_spline_.yaw.empty() ? fop::getFrenet(current_state_, local_lane_) :
                                               fop::getFrenet(current_state_, ref_spline_);

      // Clear the last output path
      output_path_.clear();
      regenerate_flag_ = false;
    }
    // if not regenerating
    else
    {
      ROS_INFO("Local Planner: Continuing From The Previous Path...");

      // End of the previous path speed
      const double output_path_last_speed =
          hypot(output_path_.x.back() - output_path_.x.end()[-2], output_path_.y.back() - output_path_.y.end()[-2]) /
          SETTINGS.tick_t;
      // End of the previous path state
      fop::VehicleState last_state = fop::VehicleState(output_path_.x.back(), output_path_.y.back(),
                                                                       output_path_.yaw.back(), output_path_last_speed);

      start_state_ = ref_spline_.yaw.empty() ? fop::getFrenet(last_state, local_lane_) :
                                               fop::getFrenet(last_state, ref_spline_);
    }

    // Ensure the speed is above the minimum planning speed
    start_state_.s_d = std::max(start_state_.s_d, MIN_PLANNING_SPEED);

    // Update current lane
    current_lane_ = (start_state_.d >= -LEFT_LANE_WIDTH / 2) ? LEFT_LANE : RIGHT_LANE;
  }
}

// Calculate the sampling width for the planner
std::vector<double> FrenetOptimalPlannerNode::getSamplingWidthFromTargetLane(const int lane_id, const double vehicle_width,
                                                                             const double left_lane_width, const double right_lane_width)
{
  double left_bound, right_bound;

  switch (lane_id)
  {
    // both lanes
    case 0:
      left_bound = left_lane_width / 2 - vehicle_width / 2;
      right_bound = -left_lane_width / 2 - right_lane_width + vehicle_width / 2;
      ROS_INFO("Local Planner: Sampling On Both Lanes");
      break;

    // stay within left lane
    case 1:
      left_bound = left_lane_width / 2 - vehicle_width / 2;
      right_bound = -left_bound;
      ROS_INFO("Local Planner: Sampling On The Left Lane");
      break;

    // stay within right lane
    case 2:
      left_bound = -left_lane_width / 2 - (vehicle_width / 2);
      right_bound = -left_lane_width / 2 - right_lane_width + vehicle_width / 2;
      ROS_INFO("Local Planner: Sampling On The Right Lane");
      break;
  }

  return { left_bound, right_bound };
}

// Select the ideal lane to proceed
fop::FrenetPath FrenetOptimalPlannerNode::selectLane(const std::vector<fop::FrenetPath>& best_path_list,
                                                     const int current_lane)
{
  fop::FrenetPath best_path;
  bool change_lane_flag;
  int keep_lane_id = -1;
  int change_lane_id = -1;
  double keep_lane_cost = std::numeric_limits<double>::max();
  double change_lane_cost = std::numeric_limits<double>::max();

  for (size_t i = 0; i < best_path_list.size(); i++)
  {
    if (!best_path_list[i].x.empty())
    {
      // keep lane option
      if (best_path_list[i].lane_id == current_lane || best_path_list[i].lane_id == 0)
      {
        if (best_path_list[i].cf < keep_lane_cost)
        {
          keep_lane_id = i;
          keep_lane_cost = best_path_list[i].cf;
        }
      }
      // change lane option
      else
      {
        change_lane_id = i;
        change_lane_cost = best_path_list[i].cf;
      }
    }
  }

  // if both lanes available
  if (keep_lane_id != -1 && change_lane_id != -1)
  {
    if (keep_lane_cost <= change_lane_cost)
    {
      ROS_DEBUG("Local Planner: Keeping Lane");
      change_lane_flag = false;
      best_path = best_path_list[keep_lane_id];
    }
    else
    {
      ROS_DEBUG("Local Planner: Changing Lane");
      change_lane_flag = true;
      best_path = best_path_list[change_lane_id];
    }
  }
  // if only keep lane available
  else if (keep_lane_id != -1 && change_lane_id == -1)
  {
    ROS_DEBUG("Local Planner: Keeping Lane");
    change_lane_flag = false;
    best_path = best_path_list[keep_lane_id];
  }
  // if only change lane available
  else if (keep_lane_id == -1 && change_lane_id != -1)
  {
    ROS_DEBUG("Local Planner: Changing Lane");
    change_lane_flag = true;
    best_path = best_path_list[change_lane_id];
  }
  // if none available
  else
  {
    ROS_DEBUG("Local Planner: No Path Available");
    change_lane_flag = false;
    // dummy path
    best_path = fop::FrenetPath();
  }

  publishTurnSignal(best_path, change_lane_flag, TURN_YAW_THRESH);

  return best_path;
}

// Concatenate the best next path to the current path
void FrenetOptimalPlannerNode::concatPath(const fop::FrenetPath& frenet_path, const int path_size,
                                  const double wp_max_seperation, const double wp_min_seperation)
{
  // Concatenate the best path to the output path
  int diff = std::min(path_size - output_path_.x.size(), frenet_path.x.size());
  // std::cout << "Output Path Size: " << output_path_.x.size() << " Current Size: " << path_size << " Diff: " << diff
  //           << " Next Path Size: " << frenet_path.x.size() << std::endl;

  for (size_t i = 0; i < diff; i++)
  {
    double wp_seperation;

    // Check if the separation between adjacent waypoint are permitted
    if (!output_path_.x.empty())
    {
      wp_seperation =
          fop::distance(output_path_.x.back(), output_path_.y.back(), frenet_path.x[i], frenet_path.y[i]);
    }
    else
    {
      wp_seperation = fop::distance(frenet_path.x[i], frenet_path.y[i], frenet_path.x[i+1],
                                            frenet_path.y[i+1]);
    }

    // If the separation is too big/small, reject point onward
    if (wp_seperation >= wp_max_seperation || wp_seperation <= wp_min_seperation)
    {
      // ROS_WARN("Local Planner: waypoint out of bound, rejected");
      // regenerate_flag_ = true;
      break;
    }

    output_path_.x.push_back(frenet_path.x[i]);
    output_path_.y.push_back(frenet_path.y[i]);
    output_path_.yaw.push_back(frenet_path.yaw[i]);

    // std::cout << "Concatenate round " << i << ": Output Path Size: " << output_path_.x.size() << std::endl;
  }

  // Calculate steering angle
  if (!output_path_.x.empty() && !output_path_.y.empty())
  {
    const int next_frontlink_wp_id = fop::nextWaypoint(frontaxle_state_, output_path_);
    ROS_INFO("Local Planner: Stanley Start");
    steering_angle_ = calculateSteeringAngle(next_frontlink_wp_id, frontaxle_state_);

    const int next_wp_id = fop::nextWaypoint(current_state_, output_path_);

    for (size_t i = 0; i < next_wp_id; i++)
    {
      output_path_.x.erase(output_path_.x.begin());
      output_path_.y.erase(output_path_.y.begin());
      output_path_.yaw.erase(output_path_.yaw.begin());
    }
  }
  else
  {
    ROS_ERROR("Local Planner: Output Path is Empty, No Steering Angle");
  }
}

// Steering Help Function
double FrenetOptimalPlannerNode::calculateSteeringAngle(const int next_wp_id, const fop::VehicleState& frontaxle_state)
{
  const double wp_id = next_wp_id + NUM_LOOK_AHEAD_WP;
  // std::cout << "Output Path Size: " << output_path_.x.size() << " Next Waypoint ID: " << wp_id << std::endl;

  // If the current path is too short, return error value
  if (output_path_.x.size() < wp_id + 2)
  {
    ROS_ERROR("Local Planner: Output Path Too Short! No output steering angle");
    // std::cout << "Output Path Size: " << output_path_.x.size() << " Required Size: " << wp_id + 2 << std::endl;
    regenerate_flag_ = true;
    return ERROR_VALUE;
  }
  else
  {
    // First Term
    const double delta_yaw = fop::unifyAngleRange(output_path_.yaw[wp_id] - current_state_.yaw);

    // Second Term
    const double c = fop::distance(output_path_.x[wp_id], output_path_.y[wp_id],
                                           output_path_.x[wp_id+1], output_path_.y[wp_id+1]);
    // if two waypoints overlapped, return error value
    if (c <= WP_MIN_SEP)
    {
      regenerate_flag_ = true;
      return ERROR_VALUE;
    }
    const double a =
        fop::distance(frontaxle_state.x, frontaxle_state.y, output_path_.x[wp_id], output_path_.y[wp_id]);
    const double b = fop::distance(frontaxle_state.x, frontaxle_state.y, output_path_.x[wp_id+1],
                                           output_path_.y[wp_id+1]);
    // if the vehicle is too far from the waypoint, return error value
    if (a >= WP_MAX_SEP || b >= WP_MAX_SEP)
    {
      ROS_WARN("Local Planner: Vehicle is too far from the path, Regenerate");
      regenerate_flag_ = true;
      return ERROR_VALUE;
    }

    const double p = (a + b + c) / 2.0;
    const double triangle_area = sqrt(p * (p - a) * (p - b) * (p - c));
    const double x = triangle_area * 2.0 / c;
    const double u = std::max(1.0, current_state_.v);

    // Angle of std::vector vehicle -> waypoint
    const double vectors_angle_diff =
        atan2(frontaxle_state.y - output_path_.y[wp_id], frontaxle_state.x - output_path_.x[wp_id]) -
        output_path_.yaw[wp_id];
    const double vectors_angle_diff_unified = fop::unifyAngleRange(vectors_angle_diff);
    const int direction = vectors_angle_diff_unified < 0 ? 1 : -1;

    // Final Angle
    steering_angle_ = STANLEY_OVERALL_GAIN * (delta_yaw + direction * atan(TRACK_ERROR_GAIN * x / u));
    // Check if exceeding max steering angle
    steering_angle_ = fop::limitWithinRange(steering_angle_, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);
    std::cout << "Steering Angle: " << fop::rad2deg(steering_angle_) << " degrees" << std::endl;

    return steering_angle_;
  }
}

// Publish the resulted steering angle (Stanley)
void FrenetOptimalPlannerNode::publishSteeringAngle(const double angle)
{
  // If steering angle is an error value, publish nothing
  if (angle <= ERROR_VALUE)
  {
    ROS_ERROR("Local Planner: Output steering angle is NULL");
    return;
  }
  // If steering angle is valid, publish it
  else
  {
    std_msgs::Float64 steering_angle_msg;
    steering_angle_msg.data = angle;
    steering_angle_pub.publish(steering_angle_msg);
  }
}

// Publish the turn signal
void FrenetOptimalPlannerNode::publishTurnSignal(const fop::FrenetPath& best_path, const bool change_lane,
                                         const double yaw_thresh)
{
  if (change_lane)
  {
    if (current_lane_ == 1)
    {
      target_lane_ = 2;
    }
    else if (current_lane_ == 2)
    {
      target_lane_ = 1;
    }
  }
  else
  {
    target_lane_ = current_lane_;
  }

  if (best_path.yaw.empty())
  {
    ROS_ERROR("Local Planner: No Path Generated");
  }
  else
  {
    // Check for yaw difference
    const double delta_yaw = best_path.yaw.back() - current_state_.yaw;

    // std::cout << "path yaw: " << best_path.yaw.back() << " vehicle yaw: " << best_path.yaw.front() << std::endl;
    // std::cout << "delta_yaw:" << delta_yaw << std::endl;

    if (delta_yaw >= yaw_thresh)
    {
      turn_signal_ = 1;
      ROS_DEBUG("Local Planner: Turning Left");
    }
    else if (delta_yaw <= -yaw_thresh)
    {
      turn_signal_ = -1;
      ROS_DEBUG("Local Planner: Turning Right");
    }
    else
    {
      // turn left
      if (target_lane_ < current_lane_)
      {
        turn_signal_ = 1;
        ROS_DEBUG("Local Planner: Changing Lane Left");
      }
      // turn right
      else if (target_lane_ > current_lane_)
      {
        turn_signal_ = -1;
        ROS_DEBUG("Local Planner: Changing Lane Right");
      }
      else
      {
        turn_signal_ = 0;
      }
    }
  }

  std_msgs::Int16 turn_signal_msg;
  turn_signal_msg.data = turn_signal_;
  turn_signal_pub.publish(turn_signal_msg);
}

}  // namespace fop

int main(int argc, char** argv)
{
  ros::init(argc, argv, "frenet_optimal_planner_node");
  fop::FrenetOptimalPlannerNode frenet_optimal_planner_node;
  ros::spin();  // spin the ros node.
  return 0;
}
