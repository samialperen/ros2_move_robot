# ros2_move_robot

## Overview

This package implements open loop velocity control in ROS2. It contains a class to move the robot in square as a proof of concept.

**Keywords:** open loop velocity control, ros2, draw squares

### License

The source code is released under a [MIT license](ros2_move_robot/LICENSE).

**Author: Sami Alperen Akgun<br />

The PACKAGE NAME package has been tested under [ROS2] Foxy on Ubuntu 20.04.
This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.


![ROS2 Drawing squares](doc/draw_square.gif)


## Installation

Clone the repository under your ROS2 workspace. 

    cd your_ros2_ws/src
    git clone https://github.com/samialperen/ros2_move_robot
    

## Building

You first need to build msgs package then the main package. Do not forget to use symlink for the main package.

	cd your_ros2_ws
	colcon build --packages-select ros2_draw_squares_custom_msgs
	colcon build --symlink-install --packages-select ros2_draw_squares

## Usage

Run the service server node with

	ros2 launch ros2_draw_squares move_robot_in_squares_service_server.launch.py

Run the service client node with

	ros2 launch ros2_draw_squares call_move_robot_in_squares_service_server.launch.py


## Launch files

* **move_robot_in_squares_service_server.launch.py:** Starts the service server which is responsible for making robot to draw squares by publishing to /cmd_vel

* **call_move_robot_in_squares_service_server.launch.py:** Starts the service client which calls the service to make robot to draw squares

## Nodes

### ros2_draw_squares_node

Makes robot to draw square using open loop velocity control


#### Published Topics

* **`/cmd_vel`** ([geometry_msgs/msg/Twist])

	The velocity message to control robot's trajectory


#### Services

* **`move_robot_in_square`** ([ros2_draw_squares_custom_msgs/srv/square_service_message.hpp])

	Makes robot to draw square

		ros2 service call ros2_draw_squares_custom_msgs/srv/square_service_message /move_robot_in_square "{square_size: 10.0, square_number: 2}"


#### Parameters

* **`square_size`** (double, positive)

	Determines how big the square will be

* **`square_number`** (int, min: 1)

	The number of times robot executes the square movement



## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/samialperen/ros2_move_robot/issues).

[geometry_msgs/msg/Twist]: (https://docs.ros2.org/foxy/api/geometry_msgs/msg/Twist.html)
[ROS2]: https://docs.ros.org/en/foxy/index.html
