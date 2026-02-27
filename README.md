# F1Tenth Gazebo Simulation for ROS 2 Humble

A multi-car autonomous racing simulation package for F1Tenth vehicles in Ignition Gazebo (Fortress) with ROS 2 Humble.

## Features

- 🏎️ **3 Colored Race Cars**: Red, Blue, and Green F1Tenth vehicles
- 🏁 **Complete Racing Track**: Oval track with walls, chicanes, and start/finish line
- 🎮 **Independent Control**: Each car controlled via separate ROS 2 topics
- 🔧 **Differential Drive**: Realistic vehicle dynamics
- 📡 **ROS 2 Integration**: Full topic bridge between Ignition and ROS 2

---

## Prerequisites

| Requirement | Version |
|-------------|---------|
| Ubuntu | 22.04 (Jammy) |
| ROS 2 | Humble |
| Gazebo | Ignition Fortress (6.x) |

---

## Installation

### Option 1: Quick Install Script

```bash
# Create workspace
mkdir -p ~/f1tenth_ws/src
cd ~/f1tenth_ws/src

# Clone/copy the f1tenth_gazebo package here
# Then run:
cd ~/f1tenth_ws/src/f1tenth_gazebo
chmod +x install.sh
./install.sh
```

### Option 2: Manual Installation

#### Step 1: Install ROS 2 Humble (if not already installed)

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 apt repository
sudo apt install software-properties-common curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Desktop
sudo apt update
sudo apt install ros-humble-desktop -y

# Add to bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### Step 2: Install Gazebo and Dependencies

```bash
# Ignition Gazebo and ROS bridge
sudo apt install -y \
    ros-humble-ros-gz \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-interfaces

# ROS 2 packages
sudo apt install -y \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-teleop-twist-keyboard
```

#### Step 3: Create Workspace and Build

```bash
# Create workspace
mkdir -p ~/f1tenth_ws/src
cd ~/f1tenth_ws/src

# Copy f1tenth_gazebo package to src/
# (Copy your f1tenth_gazebo folder here)

# Build
cd ~/f1tenth_ws
colcon build --packages-select f1tenth_gazebo
source install/setup.bash
```

---

## Usage

### Launch Simulation

```bash
# Terminal 1: Launch Gazebo with 3 cars
source ~/f1tenth_ws/install/setup.bash
ros2 launch f1tenth_gazebo f1tenth_ign.launch.py
```

**Note:** Click the **Play button** (▶️) at the bottom-left of Gazebo to start simulation.

### Drive the Cars

```bash
# Terminal 2: Drive RED car
ros2 topic pub /model/car1_red/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 3.0}, angular: {z: 0.0}}" -r 10

# Terminal 3: Drive BLUE car
ros2 topic pub /model/car2_blue/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.5}, angular: {z: 0.0}}" -r 10

# Terminal 4: Drive GREEN car
ros2 topic pub /model/car3_green/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0}, angular: {z: 0.0}}" -r 10
```

### Keyboard Teleop

```bash
# Control RED car with keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/model/car1_red/cmd_vel
```

**Keys:**
| Key | Action |
|-----|--------|
| `i` | Forward |
| `,` | Backward |
| `j` | Turn Left |
| `l` | Turn Right |
| `k` | Stop |

### Stop All Cars

```bash
ros2 topic pub /model/car1_red/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0}, angular: {z: 0}}" --once
ros2 topic pub /model/car2_blue/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0}, angular: {z: 0}}" --once
ros2 topic pub /model/car3_green/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0}, angular: {z: 0}}" --once
```

---

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
---

## Package Structure

```
f1tenth_gazebo/
├── launch/
│   └── f1tenth_ign.launch.py     # Ignition Gazebo launch
├── urdf/
│   ├── racecar.xacro             # Base vehicle model
│   ├── racecar_red.xacro         # Red variant
│   ├── racecar_blue.xacro        # Blue variant
│   ├── racecar_green.xacro       # Green variant
│   ├── racecar.gazebo            # Gazebo plugins
│   ├── macros.xacro              # URDF macros
│   └── materials.xacro           # Colors/materials
├── worlds/
│   └── track.sdf                 # Race track world
├── meshes/
│   ├── chassis.stl
│   ├── left_wheel.stl
│   ├── right_wheel.stl
│   ├── hinge.stl
│   └── hokuyo.stl
├── package.xml
├── setup.py
├── install.sh
└── README.md
```

---

## Track Details

| Property | Value |
|----------|-------|
| Outer Dimensions | 24m × 16m |
| Track Width | 4m |
| Outer Walls | White |
| Inner Walls | Orange |
| Chicanes | Red |
| Start/Finish |

---

## License

MIT License

## References

- [F1Tenth](https://f1tenth.org/)
- [ROS 2 Humble](https://docs.ros.org/en/humble/)
- [Ignition Gazebo](https://gazebosim.org/)
