# Path Planning and Control Framework for TurtleBot3

## Overview

This framework, developed by Vladimir Petrov, provides a comprehensive solution for path planning and control of a TurtleBot3 robot in a simulated environment using ROS (Robot Operating System). It includes nodes for detecting obstacles, planning paths, and controlling the robot's movement, allowing the TurtleBot to autonomously navigate through its environment, avoid obstacles, and reach its target.

## Features

- **Obstacle Detection**: Real-time obstacle detection using laser scans from the TurtleBot3, identifying walls and open spaces.
- **Path Planning**: A breadth-first search (BFS)-based path planning algorithm to navigate the TurtleBot from a start position to a goal position on a grid.
- **Robot Control**: A PID-based control system for the TurtleBot to follow the planned path and navigate around obstacles.
- **Gazebo Integration**: Simulate the TurtleBot3 in a Gazebo world and visualize the robot's movement and path in RViz.
  
## Components

### Nodes

1. **`detection_node`**: 
   - This node processes the laser scan data from the TurtleBot3 and detects obstacles in the environment.
   - Publishes the detected obstacle data to `/obstacles`.
   
2. **`pp_node`**: 
   - The path planning node, which uses a BFS algorithm to find the optimal path from the robot’s current position to the goal.
   - Publishes the planned path to the `/planner` topic.
   
3. **`node_bot`**: 
   - A control node that uses PID control to navigate the TurtleBot along the planned path, adjusting the robot's velocity based on the current position and heading.

### Launch Files

1. **`run_nodes.launch`**: 
   - Launches the detection, path planning, and robot control nodes. This is the main entry point for running the entire system.
   
2. **`source.launch`**: 
   - Launches the Gazebo simulator with an empty world and spawns the TurtleBot3 in the environment. This allows for a visual simulation of the TurtleBot's movements.
   
### Configurations

- **Configuration File**: 
  - The framework uses a configuration file (`config.yaml`) located in the `config/` directory. This file contains parameters such as the robot model, simulation settings, and PID controller gains.
  
- **World Files**:
  - The `worlds/` directory contains the Gazebo world files, including `EE3305_2022.world`, where the robot navigates through predefined environments.

### Key Files

1. **`include/definitions.hpp`**:
   - Defines constants and data structures used throughout the system, such as the grid size, goal coordinates, and distance threshold for obstacle detection.
   
2. **`src/detection_node.cpp`**:
   - The implementation of the obstacle detection node.
   
3. **`src/pp_node.cpp`**:
   - The implementation of the path planning node using the BFS algorithm.
   
4. **`src/node_bot.cpp`**:
   - The implementation of the control node that manages the robot's movements and velocity based on the planned path.

## Installation and Setup

### Prerequisites

- **ROS**: This framework is built using ROS. Make sure you have ROS installed on your system.
- **TurtleBot3 Packages**: Ensure that TurtleBot3-related ROS packages are installed.
- **Gazebo**: Install Gazebo for the robot simulation environment.

### Installation

1. Clone the repository into your catkin workspace:
    ```bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    git clone https://github.com/yourusername/path_planner_vpetrov.git
    ```

2. Build the workspace:
    ```bash
    cd ~/catkin_ws
    catkin_make
    ```

3. Source the workspace:
    ```bash
    source devel/setup.bash
    ```

### Running the Simulation

1. Start the simulation in Gazebo:
    ```bash
    roslaunch path_planner_vpetrov source.launch
    ```

2. Run the nodes for obstacle detection, path planning, and control:
    ```bash
    roslaunch path_planner_vpetrov run_nodes.launch
    ```

3. Visualize the robot's movements and path in RViz:
    ```bash
    rosrun rviz rviz
    ```

## Usage

- The robot will autonomously navigate towards a predefined goal while avoiding obstacles.
- You can modify the goal location and other parameters in the `config.yaml` file.

### Customization

- **Goal Position**: Change the goal coordinates in the `config/config.yaml` file to specify a different target for the TurtleBot.
- **Path Planning Algorithm**: The current implementation uses BFS for pathfinding. You can replace this algorithm with more advanced methods if needed.
- **PID Controller Tuning**: The PID gains used for robot control can be adjusted in the `config.yaml` file to improve the performance based on the environment.

## Contributions

Feel free to submit issues or pull requests to improve this project. Contributions are welcome!

## License

This project is licensed under the MIT License. See the `LICENSE` file for more details.

## Acknowledgements

Developed by Vladimir Petrov.
