Avoiding Obstacles with VFF
===========================

This repository contains a ROS 2 package that implements obstacle avoidance using the Virtual Force Field (VFF) algorithm. The algorithm computes control commands for a TurtleBot by combining three vectors:
- Attractive Vector: Always points forward to guide the robot along a straight path.
- Repulsive Vector: Derived from laser scan data, it repels the robot away from nearby obstacles.
- Resultant Vector: The sum of the attractive and repulsive vectors, which is used to compute the final velocity commands.

A key enhancement in this implementation is a slowdown mechanism: when an obstacle is detected within a specified threshold distance, the robot’s linear speed is scaled down proportionally to the distance from the obstacle. Additionally, visual markers are published (in RViz2) to display the attractive (blue), repulsive (red), and resultant (green) vectors for debugging.

Package Structure
-----------------
vff_avoidance/
  ├── CMakeLists.txt
  ├── package.xml
  ├── config/
  │     └── AvoidanceNodeConfig.yaml
  ├── include/
  │     └── vff_avoidance/
  │           └── AvoidanceNode.hpp
  ├── launch/
  │     └── avoidance_vff.launch.py
  └── src/
         ├── avoidance_vff_main.cpp
         └── vff_avoidance/
               └── AvoidanceNode.cpp

Prerequisites
-------------
- Ubuntu with ROS 2 Humble installed.
- (Optional) TurtleBot3 simulation packages if you plan to test in simulation:
    - ros-humble-turtlebot3
    - ros-humble-turtlebot3-gazebo

Installation and Build Instructions
-------------------------------------
1. Clone the Repository:
   Navigate to your ROS 2 workspace 'src' directory and clone the repository:
      cd ~/ros2_ws/src
      git clone https://github.com/YourUserName/GroupName-VFF-Avoidance.git

2. Build the Package:
   Return to your workspace root and build the package using colcon:
      cd ~/ros2_ws
      colcon build --packages-select vff_avoidance

3. Source the Workspace:
   Once the build completes, source the workspace:
      source install/setup.bash

Running the Node
----------------
1. Launch the VFF Avoidance Node:
   Use the provided launch file to start the node:
      ros2 launch vff_avoidance avoidance_vff.launch.py

2. (Optional) Launch TurtleBot Simulation:
   If testing in simulation, open a new terminal and run:
      export TURTLEBOT3_MODEL=burger
      ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

3. (Optional) Visualize Debug Markers in RViz2:
   To view the published vector markers, run:
      ros2 run rviz2 rviz2
   Then add a display for the MarkerArray topic (e.g., /vff_markers).

Configuration
-------------
All parameters are defined in the YAML file located at config/AvoidanceNodeConfig.yaml. Key configurable parameters include:

- Sensor and Topic Names:
    - laser_topic: Laser scan topic (default: /scan)
    - cmd_vel_topic: Velocity command topic (default: /cmd_vel)
    - marker_topic: Marker visualization topic (default: /vff_markers)

- Gains and Speeds:
    - attractive_gain: Gain for the attractive vector (default: 1.0)
    - repulsive_gain: Gain for the repulsive vector (default: 1.5)
    - max_linear_speed: Maximum linear speed (default: 0.5)
    - max_angular_speed: Maximum angular speed (default: 1.0)

- Obstacle Detection and Slowdown:
    - obstacle_distance_threshold: Distance threshold (in meters) within which obstacles influence the robot and trigger a slowdown (default: 1.5).
      The robot’s linear speed is scaled by the ratio (closest obstacle distance / threshold) when an obstacle is detected within this range.
    - min_valid_scan_age: Maximum age for a valid laser scan (default: 2.0 seconds)

Authors
-------
- Charbel Abi Saad
- Rowan Hakim

