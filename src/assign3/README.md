#Assignment 3

##  To build the package:
```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select assign3 assignment_interfaces
source install/setup.bash
```
## To launch Gazebo
```bash
ros2 launch assign3 gz_sim.launch.py
```

# Testing

## Test for joints to ee velocities
```bash
ros2 service call /joints_to_ee_velocities_service assignment_interfaces/srv/JointToEEVelocities "{joint_velocities: [-12.8304, 35.3642, 10.0000]}"
```

## Test for ee to joint velocities
```bash
ros2 service call /ee_to_joints_velocities_service assignment_interfaces/srv/EEToJointsVelocities "{ee_velocities: [5.0, 5.0, 10.0000]}"
```