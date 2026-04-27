#Assignment 3

##  To build the package:
```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select assign3 assignment_interfaces
source install/setup.bash
```
## To launch Gazebo
```bash
ros2 launch assign3 gz_sim.launch.py control_mode:="force"
```
## Check which effort service is available
```bash
ros2 topic list | grep cmd_force
```

Example pub effort:

```bash
ros2 topic pub --once /model/scara_robot/joint/joint3/cmd_vel std_msgs/msg/Float64 "{data: 1}"
```

# Testing

## Part 1
### Test for joints to ee velocities
```bash
ros2 service call /joints_to_ee_velocities_service assignment_interfaces/srv/JointsToEEVelocities "
    {
        joints_velocities: [-12.8304, 35.3642, 10.0000], 
        current_positions: [1.05, 1.05, 0]
    }"
```
### Test for ee to joint velocities
```bash
ros2 service call /ee_to_joints_velocities_service assignment_interfaces/srv/EEToJointsVelocities "
    {
        ee_velocities: [5.0, 5.0, 10.0000], 
        current_positions: [1.05, 1.05, 0]
    }"
```

## Part 2
```bash
ros2 launch assign3 gz_sim.launch.py control_mode:="force"
```

### Test joint control client

Change `0.1` to any position. 

Joint 1: Limit lower="-2.748893571891069" upper="2.748893571891069"
Joint 2: Limit lower="-2.0943951023931953" upper="2.0943951023931953"
Joint 3: The limit of the joint is 0.0-0.2 `downward`

```bash
ros2 run assign3 joint_control_client joint1 0.1
ros2 run assign3 joint_control_client joint2 0.1
ros2 run assign3 joint_control_client joint3 0.1
```

## Part 3 Velocity control

```python
    self.PD_params[self.joint_names.index('joint1')]['kp'] = 10.0
    self.PD_params[self.joint_names.index('joint1')]['kd'] = 2.0

    self.PD_params[self.joint_names.index('joint2')]['kp'] = 5.0
    self.PD_params[self.joint_names.index('joint2')]['kd'] = 1.0

    self.PD_params[self.joint_names.index('joint3')]['kp'] = 2.0
    self.PD_params[self.joint_names.index('joint3')]['kd'] = 0.0
```

```bash
ros2 launch assign3 gz_sim.launch.py control_mode:="velocity"
```

### Test joint control client

Change `0.1` to any position. 

Joint 1: Limit lower="-2.748893571891069" upper="2.748893571891069"
Joint 2: Limit lower="-2.0943951023931953" upper="2.0943951023931953"
Joint 3: The limit of the joint is 0.0-0.2 `downward`

```bash
ros2 run assign3 joint_control_client joint1 1
ros2 run assign3 joint_control_client joint2 1
ros2 run assign3 joint_control_client joint3 1
```