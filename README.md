# EE/CS 265A - Autonomous Racing Workspace

ROS 2 Humble workspace for F1Tenth autonomous racing with Ignition Gazebo simulation.

## Packages

| Package | Description |
|---------|-------------|
| `f1tenth_gazebo` | Gazebo simulation with 3 colored race cars (Red, Blue, Green) and a race track |
| `ee_cs_265a` | Course package for autonomous racing algorithms (wall following, gap finding, etc.) |

## Prerequisites

| Requirement | Version |
|-------------|---------|
| Ubuntu | 22.04 (Jammy) |
| ROS 2 | Humble |
| Gazebo | Ignition Fortress (6.x) |

## Installation

```bash
# Install dependencies
sudo apt install -y \
    ros-humble-ros-gz \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-interfaces \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-teleop-twist-keyboard

# Build workspace
cd ~/ee_cs_265A_ws
colcon build
source install/setup.bash
```

## Usage

### Launch Simulation

```bash
source ~/ee_cs_265A_ws/install/setup.bash
ros2 launch f1tenth_gazebo f1tenth_ign.launch.py
```

### Drive the Cars

```bash
# Drive RED car
ros2 topic pub /red/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 3.0}, angular: {z: 0.0}}" -r 10

# Drive BLUE car
ros2 topic pub /blue/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.5}, angular: {z: 0.0}}" -r 10

# Drive GREEN car
ros2 topic pub /green/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0}, angular: {z: 0.0}}" -r 10
```

### Keyboard Teleop

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/red/cmd_vel
```

## ROS 2 Topics

### Velocity Commands (geometry_msgs/msg/Twist)

| Car | Topic |
|-----|-------|
| Red | `/red/cmd_vel` |
| Blue | `/blue/cmd_vel` |
| Green | `/green/cmd_vel` |

### Odometry (nav_msgs/msg/Odometry)

| Car | Topic |
|-----|-------|
| Red | `/red/odometry` |
| Blue | `/blue/odometry` |
| Green | `/green/odometry` |

## Workspace Structure

```
ee_cs_265A_ws/
├── src/
│   ├── f1tenth_gazebo/   # worlf and robot package
│   │   ├── launch/
│   │   │   ├── f1tenth_ign.launch.py
│   │   │   └── f1tenth_gazebo.launch.py
│   │   ├── urdf/
│   │   │   ├── racecar.xacro
│   │   │   ├── racecar_red.xacro
│   │   │   ├── racecar_blue.xacro
│   │   │   ├── racecar_green.xacro
│   │   │   ├── racecar.gazebo
│   │   │   ├── macros.xacro
│   │   │   └── materials.xacro
│   │   ├── worlds/
│   │   │   └── track.sdf
│   │   └── meshes/
│   │       ├── chassis.stl
│   │       ├── left_wheel.stl
│   │       ├── right_wheel.stl
│   │       ├── hinge.stl
│   │       └── hokuyo.stl
│   └── ee_cs_265a/              # algorithms
│       ├── ee_cs_265a/
│       ├── launch/
│       ├── package.xml
│       └── setup.py
└── README.md
```

## Track Details

| Property | Value |
|----------|-------|
| Outer Dimensions | 24m x 16m |
| Track Width | 4m |
| Outer Walls | White |
| Inner Walls | Orange |
| Chicanes | Red |

## References

- [F1Tenth](https://f1tenth.org/)
- [ROS 2 Humble](https://docs.ros.org/en/humble/)
- [Ignition Gazebo](https://gazebosim.org/)
