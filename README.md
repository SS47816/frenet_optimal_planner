# frenet_optimal_planner

[![CodeFactor](https://www.codefactor.io/repository/github/ss47816/lgsvl_utils/badge)](https://www.codefactor.io/repository/github/ss47816/frenet_optimal_planner)
![Code Grade](https://api.codiga.io/project/30669/status/svg)
![Code Quality Score](https://api.codiga.io/project/30669/score/svg)
![GitHub Repo stars](https://img.shields.io/github/stars/ss47816/frenet_optimal_planner?color=FFE333)
![GitHub Repo forks](https://img.shields.io/github/forks/ss47816/frenet_optimal_planner?color=FFE333)

![Ubuntu](https://img.shields.io/badge/OS-Ubuntu-informational?style=flat&logo=ubuntu&logoColor=white&color=2bbc8a)
![ROS](https://img.shields.io/badge/Tools-ROS-informational?style=flat&logo=ROS&logoColor=white&color=2bbc8a)
![C++](https://img.shields.io/badge/Code-C++-informational?style=flat&logo=c%2B%2B&logoColor=white&color=2bbc8a)

Implementation of the Frenet Optimal Planning Algorithm (with modifications) in ROS.

The original paper "Optimal trajectory generation for dynamic street scenarios in a Frenét Frame" can be found [here](https://ieeexplore.ieee.org/document/5509799)

## Demo
![image](./pics/demo_2d.png "Demo 2D")

## Features & Modifications
1. We improved the framework to adopt the pipeline used in `Apollo Lattice Planner` to speed up the planning.
2. We add an asynchronous collision checking module as an option (can be switched on by setting `use_async = True`), which can significantly improve the planning frequency.
3. We add a trajectory concatenation strategy to keep the trajectories consistent between adjacent planning cycles.

## Dependencies
Our package is only based on standard ROS pkgs, with no other external dependencies:
* C++11 above
* CMake: 3.0.2 above
* Eigen (included)
* ROS Packages:
  * roscpp
  * rospy
  * tf
  * tf2_ros
  * std_msgs
  * nav_msgs
  * geometry_msgs
  * autoware_msgs
  * message_generation
  * visualization_msgs
  * dynamic_reconfigure

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
# Launch nodes
roslaunch frenet_optimal_planner frenet_optimal_planner.launch
```

## Contribution
You are welcome contributing to the package by opening a pull-request

We are following: 
[Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html), 
[C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#main), 
and [ROS C++ Style Guide](http://wiki.ros.org/CppStyleGuide)

## License
Our [`frenet_optimal_planner`](https://github.com/SS47816/frenet_optimal_planner) ROS package is licensed under [Apache License 2.0](https://github.com/SS47816/frenet_optimal_planner/blob/main/LICENSE)

The included Eigen Library follows its own [Mozilla Public License v. 2.0](http://mozilla.org/MPL/2.0/)
