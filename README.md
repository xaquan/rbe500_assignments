# Assignment 1

## Summary

This assignment is to: 
- Create a scara robot model for gazebo simulator.
- Create a topic listen to robot joint position to calcuate forward kinematic to the ee pose,
then publish to out as pose msg.
- Create a service taking pose and calculate inverse kinematic to the joints.
  + Create the interface which input is Pose msg type and repone with array

```bash
.
├── assign1
│   ├── CMakeLists.txt
│   ├── LICENSE
│   ├── README.md
│   ├── assign1
│   │   ├── __init__.py
│   │   ├── converter_helper.py
│   │   ├── fw_solver.py
│   │   ├── inv_service.py
│   │   └── scara_kinematic_model.py
│   ├── config
│   │   └── gui.config
│   ├── launch
│   │   ├── display.launch.py
│   │   └── gz_sim.launch.py
│   ├── package.xml
│   ├── resource
│   │   └── assign1
│   ├── rviz
│   │   └── display.rviz
│   ├── scripts
│   ├── setup.cfg
│   ├── setup.py
│   ├── test
│   │   ├── test_copyright.py
│   │   ├── test_flake8.py
│   │   └── test_pep257.py
│   ├── urdf
│   │   └── scara.urdf.xacro
│   └── worlds
│       └── empty.sdf
└── assign1_interfaces
    ├── CMakeLists.txt
    ├── package.xml
    └── srv
        └── PoseToJointAngles.srv
```


## Clone the project

Create new folder

```bash
mkdir assignments
```

```bash
git clone https://github.com/xaquan/rbe500_assignments.git
```

## Install package's dependencies

```bash
rosdep install --from-paths src --ignore-src -r -y
```
--from-paths src: Looks inside your src folder for any package.xml files.
--ignore-src: Tells it not to try and install packages that are already in your src folder (your own code).
-r: Continues installing even if one package fails.
-y: Automatically says "Yes" to all package manager prompts.

## Build the packages and source it:

Source ros
  source /opt/ros/<ros_name>/setup.bash
change ros_name to our version

```bash
source /opt/ros/humble/setup.bash
```

```bash
colcon build
source install/setup.bash
```

##  To launch RViz:

```bash
ros2 launch assign1 display.launch.py
```
##  To launch gazebo:

```bash
ros2 launch assign1 gz_sim.launch.py
```

## Check topic list:

```bash
ros2 topic list
```
result:
```bash
 /joint_states
 /model/scara_robot/joint_trajectory
 /model/scara_robot/pose
```

Echo topic containt only link4 with the next 10 lines:

```bash
ros2 topic echo /model/scara_robot/pose | grep -A 10 'link4'
```
result:
```bash
child_frame_id: scara_robot/link4
 transform:
    translation:
      x: 1.20000006531234
      y: 7.36532871694378e-17
      z: 0.29999983208325537
    rotation:
      x: 0.9999999999999941
      y: -7.9175019937427e-18
      z: -1.0885391262826189e-07
      w: 2.1518559089703904e-16
```

Echo joint states topic

```bash
ros2 topic echo /joint_states
```
result:

```bash
header:
  stamp:
    sec: 529
    nanosec: 644000000
  frame_id: ''
name:
- joint1
- joint2
- joint3
- joint4
position:
- -1.2864280262544412
- 0.636443339304829
- 0.249173553719023
- 0.0
velocity:
- 6.106561853343949e-20
- 1.2409622328558278e-17
- 3.7226746869303996e-15
- 0.0
effort:
- 0.0
- 0.0
- 0.0
- 0.0
---
```

## Manual bridge topic ros2 <> gazebo

### Commnad to add urdf model to gazebo

```bash
ign service -s /world/empty/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 1000 --req 'sdf_filename: "/path_to/model.urdf", name: "model_name"'
```

### Command to bridge ros2 and gazebo using joint_trajectory data type

```bash
ros2 run ros_gz_bridge parameter_bridge /model/scara_robot/joint_trajectory@trajectory_msgs/msg/JointTrajectory]ignition.msgs.JointTrajectory
```

### Gazebo - topicpublish joint poisition

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

### Ros2 - topic publish joint poisition

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
### Topic publish joint controller

```bash
ros2 topic pub --once /joint_group_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.57, 0.5, 0.0, 1.2]}"
```

## Clean colcon

```bash
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH
```

## Test service client



## Test service convert pose to joint positions

position:
- 0.8937289445557175
- 1.1645558974513888
- 6.646935660075472e-07

```bash
ros2 run assign1 ik_solver 0.11798657510175536 0.6599652378111799 0.20000008879895975 0.5155538571770035 0.8568571761675969 -3.8640312619604363e-08 -6.258948909451002e-08
```

OR

```bash
ros2 service call /pose_to_joint_angles assign1_interfaces/srv/PoseToJointAngles "{
  ee_pose: {
    position: {
      x: 0.11798657510175536,
      y: 0.6599652378111799,
      z: 0.20000008879895975,
    },
    orientation: {
      x: 0.5155538571770035,
      y: 0.8568571761675969,
      z: -3.8640312619604363e-08,
      w: -6.258948909451002e-08,
    }
  }
}"
```


position:
- 0.8937289445557175
- -1.164555897451389
- 1.7251300655194996e-14

```bash
ros2 service call /pose_to_joint_angles assign1_interfaces/srv/PoseToJointAngles "{
  ee_pose: {
    position: {
      x: 0.6191720269781495,
      y: 0.2571011035588956,
      z: 0.19999253768019631,
    }, 
    orientation: {
      x: 0.9908455965788017,
      y: -0.13500001385330784,
      z: -7.345701144105147e-08,
      w: 1.032493335218514e-08,
    }
  }
}"
```

position:
- -1.8145405844010012
- -1.489548240926195
- 1.8264391023626713e-14

```bash
ros2 service call /pose_to_joint_angles assign1_interfaces/srv/PoseToJointAngles "{
  ee_pose: {
    position: {
      x: -0.45399130483070715,
      y: -0.3800748181239815,
      z: 0.2000001729892989,
    }, 
    orientation: {
      x: 0.08115872552748345,
      y: 0.9967011895602158,
      z: -5.4062051934750056e-09,
      w: -7.266067768353194e-08,
    }
  }
}"
```

position:
- 0.6770673822391795
- 0.8395635539765824
- 2.4658192120921487e-14

```bash
ros2 service call /pose_to_joint_angles assign1_interfaces/srv/PoseToJointAngles "{
  ee_pose: {
    position: {
      x: 0.36968464038844434,
      y: 0.6314160969609153,
      z: 0.2000004303939951,
    }, 
    orientation: {
      x: 0.725995492011969,
      y: 0.6876994587595946,
      z: 1.6757923795314082e-07,
      w: -1.7780160732920303e-07,
    }
  }
}"
```

position:
- 0.6229019916600453
- 1.435382850347061
- 0.1429752066115816

```bash
ros2 service call /pose_to_joint_angles assign1_interfaces/srv/PoseToJointAngles "{
  ee_pose: {
    position: {
      x: 0.4565503656759636,
      y: -0.3988568604753969,
      z: 0.1999924019306974,
    }, 
    orientation: {
      x: 0.9985334138511206,
      y: 0.05413890858542786,
      z: -7.383378110633561e-08,
      w: -4.528704393758198e-09,
    }
  }
}"
```