# Assignment 2: Joint Position Controller

This assignment focuses on building a ROS 2 position controller node for the SCARA robot in Gazebo. The controller should read the robot joint states, compute the control effort for the last joint, and drive that joint to a requested reference position.

## Project Objective

### 1. Fix the Joints
Modify the robot description so that all joints except the last one are set to `fixed`. Only the final joint should remain movable and available for control.

### 2. Write a Position Controller Node
Develop a ROS 2 node that reads the joint positions from Gazebo and sends effort commands to the last joint through `/gazebo/apply_joint_effort`.

a) Read the current joint positions from Gazebo.
b) Design and tune a PD controller for the last joint. Parameter tuning can be done experimentally; no manual calculation is required.
c) Implement a service that receives a reference position for the last joint and drives the joint to that target.
d) Record both the reference position and the actual joint position in a text file, then plot the results in Matlab.

## To launch Gazebo
"not working yet"
```bash
ros2 launch assign2 gazebo_classic.launch.py
```

## To call the Gazebo effort service
First, check which effort service is available:

```bash
ros2 service list | grep apply_joint_effort
```

In Gazebo Classic, the service is commonly available as `/gazebo/apply_joint_effort`.

- `effort: 1.0` is move downward.
- `effort: -51.0` is move upward.

Example service call:

```bash
ros2 service call /gazebo/apply_joint_effort gazebo_msgs/srv/ApplyJointEffort '{
  joint_name: joint3,
  effort: 1.0,
  start_time: {sec: 0, nanosec: 0},
  duration: {sec: 1, nanosec: 0}
}'
```

If your setup exposes `/apply_joint_effort` instead, use the same command with that service name:

```bash
ros2 service call /apply_joint_effort gazebo_msgs/srv/ApplyJointEffort '{
  joint_name: joint3,
  effort: -51.0,
  start_time: {sec: 0, nanosec: 0},
  duration: {sec: 1, nanosec: 0}
}'
```
