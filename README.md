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

# [Assignment 1 README](project1/README.md)

##  To launch RViz:
```bash
ros2 launch assign1 display.launch.py
```
##  To launch gazebo:
```bash
ros2 launch assign1 gz_sim.launch.py
```
