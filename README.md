# tcebot_gz

This ROS2 package provides a simulation for the tcebot in Gazebo Harmonic. It allows users to spawn the robot in a desired world and supports spawning multiple robots for testing and simulation.

## Table of Contents

- [Description](#description)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)

## Description

This package is designed to simulate the tcebot in the Gazebo Harmonic environment. It provides a launch file that spawns the robot in a specified world, and users can easily spawn multiple robots for their simulations.

## Prerequisites

Make sure you have the following installed:

- ROS2 (Jazzy)
- Gazebo (Harmonic)
- tcebot_description
- slam_toolbox
- navigation2
- nav2-bringup

## Installation

### Steps

1. Clone the repository:
   ```bash
   git clone https://github.com/dhanushshettigar/tcebot_gz.git
   ```
2. Build the package:
    ```bash
    colcon build
    ```
3. Source the workspace:
    ```bash
    source install/setup.bash
    ```

## Usage

### tcebot Simulation
Once the package is built and sourced, you can spawn the tcebot in the desired world with the following command:

  ```bash
  export QT_QPA_PLATFORM=xcb && ros2 launch tcebot_gz tcebot_gz.launch.py
  ```

This will launch the simulation and spawn the robot in the chosen environment. You can customize the world and robot settings within the launch file.

![TCE Robot](https://raw.githubusercontent.com/dhanushshettigar/tcebot_gz/refs/heads/main/media/tcebot_simulation.png)
![TCE Robot](https://raw.githubusercontent.com/dhanushshettigar/tcebot_gz/refs/heads/main/media/tcebot_sim_rviz.png)

