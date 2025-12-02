# Assignment1_rt package
## Overview
`assignment1_rt` is a ROS2 package designed to prevent collisions between turtles in the turtlesim simulation environment and to ensure that each turtle avoids contact with the boundaries.

### Features
- Spawn a second turtle in the `turtlesim_node`
- Avoid the collision between turtles
- Avoid the collision with boundaries
- If a turtle is too close to the other turtle or boundaries, steer it away in a secure position

## Installation
### Prerequisites
- ROS2 Jazzy or newer
- Ubuntu 24.04

### Create the workspace
```
mkdir -p ~/ros_ws/src
```
### Install turtelsim
```
sudo apt update
sudo apt install ros-jazzy-turtlesim
```
### Clone the package
```
cd ~/ros_ws/src
git clone https://github.com/TZTozz/Assignment1_rt.git
cd ..
colcon build
```

## Usage
For every new terminal:
```
cd ~/ros_ws
source install/local_setup.sh
```
Open a terminal for each one of the following commands
```
ros2 run turtlesim turtlesim_node
```
```
ros2 run assignment1_rt spawner
```
```
ros2 run assignment1_rt UI
```
```
ros2 run assignment1_rt distance
```

## Nodes
<img width="1099" height="223" alt="rosgraph" src="https://github.com/user-attachments/assets/9dc0af3a-4307-4f9d-9658-0ef15c1a1940" />

### Spawner
Spawn a second turtle in the `turtlesim_node`

### UI
- Ask the user which turtle they want to move and what linear and angular velocities should be applied
- Move the selected turtle for a second with the desired velocity
- If the turtle enters a prohibited zone, stop it for one second and then move it backward until it exits the restricted area

### Distance 
- Read the position of the two turtles
- If they are too close or near boundaries publish on the topic `stop`

## Topic
| Topic | Type | Description |
| --- | --- | --- |
| /pose | turtlesim/msg/Pose | Position of the turtle |
| /cmd_vel | geometry_msgs/msg/Twist | Control the velocity of the turtle |
| /stop | /std_msgs/msg/Bool | Is true if a turtle is in a prohibited zone |
| /distance | /std_msgs/msg/Float32 | Distance between turtles |









