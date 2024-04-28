# ROS 2 Turtlesim Workspace

This ROS 2 workspace contains code for controlling a turtle robot in the Turtlesim environment. The workspace includes the Turtlesim package along with a custom controller for the turtle robot, which can also create a Sierpinski fractal.


## Prerequisites

- Python 3
- ROS 2 (Foxy or later)

## Installation

1. Install ROS 2 on your system by following the instructions provided in the ROS 2 documentation: [ROS 2 Installation](https://docs.ros.org/en/foxy/Installation.html).

2. Clone this repository to your ROS workspace.

3. Build the ROS workspace:
		colcon build --symlink-install

## Usage

1. Move to the workspace's directory.

2. Run the Turtlesim node:
		ros2 run turtlesim turtlesim_node

3. Run the Turtlesim Controller node:
		ros2 run ros2_course turtlesim_controller

### or

3. Run the launch.py file: 
		ros2 launch ros2_course launch.py

### optional

4. set level parameter:
		ros2 param set turtlesim_controller level [level]
	
## Description

The ROS 2 Turtlesim Workspace provides a platform for simulating turtle robot behaviors using ROS 2. It includes the following components:

- **Turtlesim package**: This package provides a simple simulator for a turtle robot in a 2D environment. It allows users to control the turtle's movement and observe its behavior.

- **Turtlesim Controller**: This Python script implements a controller for the turtle robot, allowing it to perform various movements and tasks. It utilizes ROS 2 for communication with the Turtlesim node and implements functionalities such as controlled movement and drawing a Sierpinski triangle.

## License

This code is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

*This code is based on the ROS 2 tutorials and examples.*




