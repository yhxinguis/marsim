# ROS2/Gazebo Marine Simulator

ROS2/Gazebo Marine Simulator (currently under development) aims to be a comprehensive marine simulation environment utilizing ROS2 (Robot Operating System 2) and Gazebo. 

Read the marsim1.pdf file for information and details on the simulator. The pdf covers the fundamental components of the simulation stack, including Docker containerization, ROS2 architecture, and Gazebo simulation capabilities.

Key topics include:

Docker Engine setup for creating a consistent development environment
ROS2 framework, its architecture, and comparison with ROS1
Gazebo simulator and its integration with ROS2 via ros_gz_bridge
Implementation of a BlueROV2 model in the ROS2/Gazebo environment
Custom plugins and scripts for ocean current simulation, sensor data processing, and robot control
Practical aspects of running the simulation, including launch procedures and visualization tools


# Licensing

The ROS2/Gazebo Marine Simulator is licensed under the Apache License 2.0.


# Project License

Copyright 2024 Yihan Xing

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.

You may obtain a copy of the License at:
http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.


# Third-Party Licenses

Marine Simulator Gazebo (marsim_gazebo) package was extensively modified from and built on clydemcqueen/bluerov2_ignition, which is licensed under the MIT liense.

Copyright 2023 Clyde McQueen
License: MIT
https://github.com/clydemcqueen/bluerov2_ignition/tree/main


# Contributing:

Contributions to this project are welcome. By submitting a contribution, you agree to license your work under the same Apache License 2.0.


# Commands:

ros2 launch marsim_gazebo empty_world.launch.py

ros2 launch marsim_gazebo spawn_bluerov2.launch.py

ros2 launch marsim_gazebo start_bridge.launch.py

ros run marsim_gazebo ros_keyboard_controller bluerov2
