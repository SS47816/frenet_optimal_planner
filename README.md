# frenet_optimal_planner

Implementation of Frenet Optimal Planning Algorithm in ROS

![image](./pics/demo_2d.png "Demo 2D")
![image](./pics/demo_3d.png "Demo 3D")

## Features

## Dependencies
* C++11 above
* CMake: 3.0.2 above
* ROS Packages:
  * roscpp
  * rospy
  * std_msgs
  * nav_msgs
  * geometry_msgs
  * autoware_msgs
  * message_generation
  * dynamic_reconfigure
  * tf
  * tf2_ros

## Installation
Clone the repo and install dependencies:
```bash
# clone the repo
git clone https://github.com/SS47816/frenet_optimal_planner.git
cd frenet_optimal_planner

# install dependencies
rosdep install --from-paths src --ignore-src -r -y

# build using `catkin_make` , or you can build with `catkin build`
catkin_make
# source 
source devel/setup.bash
```

## Usage

Launch the Planner node by running:
```bash
# clone the repo
roslaunch frenet_optimal_planner frenet_optimal_planner.launch
```

## Contribution
You are welcome contributing to the package by opening a pull-request

We are following: 
[Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html), 
[C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#main), 
and [ROS C++ Style Guide](http://wiki.ros.org/CppStyleGuide)

## License
TBD