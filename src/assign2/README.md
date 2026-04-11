# Assignment 2: Joint Position Controller

This assignment focuses on building a ROS 2 position controller node for the SCARA robot in Gazebo. The controller should read the robot joint states, compute the control effort for the last joint, and drive that joint to a requested reference position.

## Project Objective

### 1. Fix the Joints
Modify the robot description so that all joints except the last one are set to `fixed`. Only the final joint should remain movable and available for control.

### 2. Write a Position Controller Node
Develop a ROS 2 node that reads the joint positions from Gazebo and sends effort commands to the last joint through `/model/scara_robot/joint/joint3/cmd_force`.

- [x] 1. Read the current joint positions from Gazebo.
- [x] 2. Design and tune a PD controller for the last joint. Parameter tuning can be done experimentally; no manual calculation is required.
- [x] 3. Implement a service that receives a reference position for the last joint and drives the joint to that target.
- [x] 4. Record both the reference position and the actual joint position in a text file, then plot the results in Matlab.

## PD Design:
```bash
m = 10 # mass 10kg
b = 0.5 # Friction
xi = 1 # Critically damped, not overshoot
ts = 0.5 # Settling time 0.5s/m
mg = m * -9.81  # Estimate gravity disturbance based on current joint position
wn = 5.3/(ts*xi) # For the error is 0.5%
kp = wn**2 * m
kd = 2*xi*wn*m - b

pd_output = self._PD_Controller(error, kp, kd, self.dt)
effort = pd_output + mg  # Compensate for estimated gravity disturbance
```
![Close loop control diagram](/src/assign2/images/CLCD.jpeg)

##  To build the package:
```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select assign2 assignment_interfaces
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

- `data: -50` is move downward.
- `data: -100.0` is move upward.

"The effort determined by the mass and dynamic setting in the model configuration"

Example pub effort:

```bash
ros2 topic pub --once /model/scara_robot/joint/joint3/cmd_force std_msgs/msg/Float64 "{data: -100.0}"
```

Echo joint states topic in another terminal to see the joint changing

```bash
ros2 topic echo /joint_states
```

## Test joint control client

Change `0.1` to any position. The limit of the joint is 0.0-0.2 `downward`

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

Example data:
```bash
timestamp,current_position,target_position,error,effort
...
1775638106491435074,0.0075,0.2000,0.1925,15.2790
1775638106493250178,0.0080,0.2000,0.1920,15.3894
1775638106495151849,0.0084,0.2000,0.1916,15.3518
1775638106496982776,0.0089,0.2000,0.1911,15.5644
1775638106498457549,0.0099,0.2000,0.1901,16.0055
1775638106500085956,0.0103,0.2000,0.1897,17.1671
1775638106501805157,0.0112,0.2000,0.1888,18.2227
1775638106503511906,0.0125,0.2000,0.1875,-24.9449
1775638106505087567,0.0132,0.2000,0.1868,-29.2695
1775638106506692219,0.0146,0.2000,0.1854,-34.3632
1775638106508437442,0.0160,0.2000,0.1840,-40.6909
1775638106510318270,0.0174,0.2000,0.1826,-47.0746
1775638106511939058,0.0182,0.2000,0.1818,-51.4825
1775638106513526981,0.0197,0.2000,0.1803,-56.8073
...
```

## Plot target and current positions

![Tracking position 1](/src/assign2/images/Position%20Plot1.png)

![Tracking position 2](/src/assign2/images/Position%20Plot2.png)

