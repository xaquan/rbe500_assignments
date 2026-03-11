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
ros2 launch assign1 gz_sim.launch.py
```

Check topic list:

```bash
ros2 topic list
> /joint_states
> /model/scara_robot/joint_trajectory
> /model/scara_robot/pose
```

Echo topic containt only link4 with the next 10 lines:

```bash
ros2 topic echo /model/scara_robot/pose | grep -A 10 'link4'
>child_frame_id: scara_robot/link4
> transform:
>    translation:
>      x: 1.20000006531234
>      y: 7.36532871694378e-17
>      z: 0.29999983208325537
>    rotation:
>      x: 0.9999999999999941
>      y: -7.9175019937427e-18
>      z: -1.0885391262826189e-07
>      w: 2.1518559089703904e-16
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