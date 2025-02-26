# TCEBot Gazebo Simulation

## Prerequisites
- ROS2 - Jazzy
- Gazebo (Harmonic)
- Nav2 Stack
- Slam_toolbox

## Installation

Clone the repository into your ROS 2 workspace and build it:

```bash
cd ~/ros2_ws/src
git clone <repository-url>
cd ~/ros2_ws
colcon build --packages-select tcebot_gz
source install/setup.bash
```

## Usage

### Spawning in the Desired World

![TCE ROBOT Spwan](https://raw.githubusercontent.com/dhanushshettigar/tcebot_gz/refs/heads/main/media/gz-sim.png)

After building and sourcing, run:

```bash
export QT_QPA_PLATFORM=xcb && ros2 launch tcebot_gz tcebot_simulation.launch.py headless:=False
```

Rviz will load the saved map (default: Depot Map) and Gazebo will load the world Depot.

### Navigation

![TCE ROBOT Spwan](https://raw.githubusercontent.com/dhanushshettigar/tcebot_gz/refs/heads/main/media/nav2-1.png)
![TCE ROBOT Spwan](https://raw.githubusercontent.com/dhanushshettigar/tcebot_gz/refs/heads/main/media/nav2-2.png)
![TCE ROBOT Spwan](https://raw.githubusercontent.com/dhanushshettigar/tcebot_gz/refs/heads/main/media/nav2-c.png)

Once **RViz** is launched, follow these steps:

- Use the top bar **"2D Pose Estimate"** tool to set the initial pose of the robot. This helps Nav2 localize the robot in the loaded map.

- Wait for the system to detect the robot’s position.

- Use the top bar **"2D Goal Pose"** tool to set a goal. The robot will autonomously navigate to the given location.
