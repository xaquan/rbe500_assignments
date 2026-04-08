# Assignment 2: Joint Position Controller

This assignment focuses on building a ROS 2 position controller node for the SCARA robot in Gazebo. The controller should read the robot joint states, compute the control effort for the last joint, and drive that joint to a requested reference position.

## Project Objective

### 1. Fix the Joints
Modify the robot description so that all joints except the last one are set to `fixed`. Only the final joint should remain movable and available for control.

### 2. Write a Position Controller Node
Develop a ROS 2 node that reads the joint positions from Gazebo and sends effort commands to the last joint through `/model/scara_robot/joint/joint3/cmd_force`.

a) Read the current joint positions from Gazebo.
b) Design and tune a PD controller for the last joint. Parameter tuning can be done experimentally; no manual calculation is required.
c) Implement a service that receives a reference position for the last joint and drives the joint to that target.
d) Record both the reference position and the actual joint position in a text file, then plot the results in Matlab.

##  To build the package:
```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select assign2
source install/setup.bash
```
## To launch Gazebo
```bash
ros2 launch assign2 gz_sim.launch.py
```

## To pub the Gazebo effort topic
First, check which effort service is available:

```bash
ros2 topic list | grep /model/scara_robot/joint/joint3/cmd_force
```

- `data: 3.0` is move downward.
- `data: -199.0` is move upward.

"The effort determined by the mass and dynamic setting in the model configuration"

Example pub effort:

```bash
ros2 topic pub --once /model/scara_robot/joint/joint3/cmd_force std_msgs/msg/Float64 "{data: -199.0}"
```

Echo joint states topic in another terminal to see the joint changing

```bash
ros2 topic echo /joint_states
```

## Test joint control client
```bash
ros2 run assign2 joint_control_client joint3 0.1
```

After sending the joint position, the control service will log the position of current and target to csv file in `/joint_data`. Data is used to plot in matlab

```bash
.
├── build
├── install
├── joint_data
├── log
└── src
```

example data:
```bash
timestamp,current_position,target_position,error,effort
1775638086179752984,0.2000,0.2000,-0.0000,-98.1000
1775638086181376913,0.2000,0.2000,-0.0000,-98.1000
1775638086183048163,0.2000,0.2000,-0.0000,-98.1000
```