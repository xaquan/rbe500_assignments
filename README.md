## Run the Assignment 1

Build the packages and source it:

```bash
colcon build
source install/setup.bash
```

To launch RViz:

```bash
ros2 launch assign1 display.launch.py
```
To launch gazebo:

```bash
ros2 launch assign1 gazebo.launch.py
```

Commnad to add urdf model to gazebo

```bash
ign service -s /world/empty/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 1000 --req 'sdf_filename: "/path_to/model.urdf", name: "model_name"'
```

Command to bridge ros2 and gazebo using joint_trajectory data type

```bash
ros2 run ros_gz_bridge parameter_bridge /model/scara_robot/joint_trajectory@trajectory_msgs/msg/JointTrajectory]ignition.msgs.JointTrajectory
```

Gazebo - topicpublish joint poisition

```bash
ign topic -t /model/scara_robot/joint_trajectory \
  -m ignition.msgs.JointTrajectory \
  -p 'joint_names: "joint1"
joint_names: "joint2"
joint_names: "joint3"
points {
  positions: 0.5
  positions: 0.3
  positions: 0.1
  time_from_start {
    sec: 2
    nsec: 0
  }
}'
```

Ros2 - topic publish joint poisition

```bash
ros2 topic pub --once /model/scara_robot/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['joint1', 'joint2', 'joint3'],
  points: [
    {
      positions: [0.5, 0.5, 0.3],
      time_from_start: {sec: 2, nanosec: 0}
    }
  ]
}"
```