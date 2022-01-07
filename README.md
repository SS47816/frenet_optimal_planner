# frenet_optimal_planner
Implementation of Frenet Optimal Planning Algorithm in ROS

![image](./pics/micron_0.png "Demo 1")
![image](./pics/micron_1.png "Demo 2")
![image](./pics/e8_0.png "Demo 3")

## Features

## TODOs
* **lane info**: change lane info input to standard format
* **obstacle frame**: remove frame transform for obstacles 

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