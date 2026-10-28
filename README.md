# Mechatronics Assessment: ROS 2 Humble Surveillance Robot

Documentation of a ROS 2 Humble project in which a custom differential-drive robot (`smith_bot`) is designed, mapped, and programmed to patrol a simulated office. The work covers workspace setup, SLAM mapping, an autonomous surveillance route, rosbag recording, and robot design in URDF/Xacro.

> **Note:** The original workspace stopped working, so the project was rebuilt from scratch and documented from that point.

## Table of contents

0. [Environment setup](#0-environment-setup)
1. [Create a workspace and Python package](#1-create-a-workspace-and-python-package)
2. [Create an occupancy grid map](#2-create-an-occupancy-grid-map)
3. [Surveillance program](#3-surveillance-program)
4. [Record robot movements with rosbag](#4-record-robot-movements-with-rosbag)
5. [Design a robot](#5-design-a-robot)
6. [Observations and issues](#observations-and-issues)

## 0. Environment setup

The project ran on a personal laptop with Ubuntu installed in a dual-boot setup alongside Windows, using **ROS 2 Humble**.

### Dependencies

Following *Lab 0: Install additional files and dependencies* from the mechatronics labs:

```bash
sudo apt install terminator   # shift-control-e and shift-control-o to split panes
sudo apt install ros-humble-gazebo-ros2-control
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-turtlesim
sudo apt install ros-humble-robot-localization
sudo apt install ros-humble-joy
sudo apt install ros-humble-joy-teleop
sudo apt install ros-humble-joint-state-publisher
```

Also install the CycloneDDS middleware:

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

### Nav2 parameter change

In the TurtleBot3 `waffle.yaml` parameters file, the AMCL robot model type was changed to the differential motion model:

```yaml
resample_interval: 1
#robot_model_type: "differential"
robot_model_type: "nav2_amcl::DifferentialMotionModel"
save_pose_rate: 0.5
```

### Background learning

The Linux and Python introductions on [The Construct](https://www.theconstructsim.com/) were completed before starting.

## 1. Create a workspace and Python package

The package metadata (`package.xml` and `setup.py`) should include a package name, description, maintainer, and license type.

### Create the workspace

```bash
mkdir -p adam_ws
ls
```

### Create the package

From inside `adam_ws/src`:

```bash
ros2 pkg create --build-type ament_python --node-name smith_node smith_bot
```

### Build

From the workspace root:

```bash
colcon build
```

The build finished successfully (`Finished <<< smith_bot`).

### Update package metadata

`package.xml` after editing:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>smith_bot</name>
  <version>0.0.0</version>
  <description>adam project</description>
  <maintainer email="smitha95@student.eit.ac.nz">adam</maintainer>
  <license>MIT</license>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### Populate the package folders

The package was given the following folders: `config`, `launch`, `log`, `meshes`, `models`, `resource`, `rviz`, `smith_bot`, `test`, `urdf`, and `worlds`.

![Package folder layout](images/package-folders.png)

### Update `setup.py`

The default `setup.py` was changed so that the new folders are installed and the project's nodes are registered as console scripts:

```python
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'smith_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adam',
    maintainer_email='smitha95@student.eit.ac.nz',
    description='adam project',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'smith_node = smith_bot.smith_node:main',
            'occupancy_grid_pub = smith_bot.occupancy_grid_pub:main',
            'sdf_spawner = smith_bot.spawn_entity:main',
            'maze_solver = smith_bot.maze_solver:main',
            'autonomous_waiter_lite = smith_bot.hotel_waiter_single_goal:start_app',
            'autonomous_waiter = smith_bot.hotel_waiter_multi_button:start_app',
        ],
    },
)
```

## 2. Create an occupancy grid map

The map was built with SLAM while driving the robot around the simulated office. Use one terminal for each of the following.

**Terminal 1: launch the simulation world**

```bash
cd adam_ws
ros2 launch smith_bot final_world.launch.py
```

**Terminal 2: start SLAM Toolbox**

```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
```

**Terminal 3: drive the robot with the keyboard**

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

**Terminal 4: open RViz**

```bash
ros2 launch turtlebot3_bringup rviz2.launch.py
```

RViz did not show a live version of the map, so progress was checked by saving the map every few rooms.

![RViz showing odom and base_footprint frames](images/rviz-odom-frame.png)

### Save the map

Once every room had been covered:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/final_map.yaml
```

> The path may change as the workspace is altered.

The saved map was then opened in GIMP and the edges were cleaned up.

![Cleaned occupancy grid map](images/occupancy-grid-map.png)

## 3. Surveillance program

The surveillance script is based on the [`demo_security.py`](https://github.com/ros-navigation/navigation2/blob/main/nav2_simple_commander/nav2_simple_commander/demo_security.py) example from the Nav2 Simple Commander. It was adapted to patrol the provided `officev4` simulation world.

Changes from the demo:

- Defined a custom **security route**, because none was provided
- Added more comments

![Security route waypoints, part 1](images/security-route-1.png)
![Security route waypoints, part 2](images/security-route-2.png)
![Security route waypoints, part 3](images/security-route-3.png)

## 4. Record robot movements with rosbag

Robot movements were recorded with `rosbag`.

1. Launch the robot in Gazebo:

   ```bash
   ros2 launch smith_bot maze_navigation.py
   ```

2. List all available topics:

   ```bash
   ros2 topic list
   ```

3. Record only the topics that describe the robot's movement. The command shown in the screenshot includes:

   ```bash
   ros2 bag record -o ros_bag /goal_pose /waypoint_follower/transition_event /local_plan /amcl_pose
   ```

   The screenshot is cut off at the edge of the terminal, so additional topics may have been included.

4. Start `rosbag` first, then start the security program while recording.

> **Tip:** Don't record everything with `-a`. On the first attempt this passed `-all` as the topic selection and the bag exceeded 100 GB very quickly. Selecting only the relevant topics keeps the file size manageable.

## 5. Design a robot

The robot was designed by following several URDF tutorials, with inspiration taken from SamBot and TurtleBot3. VS Code was used with the ROS 2 and URDF preview extensions, which made the URDF/Xacro files easier to edit and view.

### Design specification

| Property | Value |
|---|---|
| Body diameter | 400 mm |
| Body height | 800 mm |
| Wheel diameter | 150 mm |
| Sensors | LiDAR, camera, IMU, odometry |

### Iterations

The first iteration used a radius of 400, which gave a diameter of 800 and was twice the intended size.

![First iteration](images/robot-iteration-1a.png)
![First iteration, alternate view](images/robot-iteration-1b.png)
![URDF code, early iteration](images/robot-urdf-code.png)

Many iterations followed, with most of the difficulty coming from XML syntax and formatting. The final robot is within specification.

![Final robot](images/robot-final-1.png)
![Final Xacro code](images/robot-final-code.png)
![Final robot, view 2](images/robot-final-2.png)
![Final robot, view 3](images/robot-final-3.png)
![Final robot, view 4](images/robot-final-4.png)
![Final robot, view 5](images/robot-final-5.png)

### Verify in RViz

The model was checked in RViz with the display launch file:

```bash
ros2 launch smith_bot display.launch.py
```

![Robot in RViz](images/robot-rviz-display.png)

### Convert Xacro to URDF to SDF

The Xacro file was converted to URDF, and the URDF was then converted to SDF for loading into Gazebo:

```bash
cd urdf
xacro smith_bot5.xacro > smith_bot5.urdf
gz sdf -p /smith_bot5.urdf > /home/adam/adam_ws/src/smith_bot/models/smith_bot5.sdf
```

An xml model file, a config file, and an SDF 1.4 version were then created.

> **Known quirk:** A `File doesn't exist` error occasionally appeared during conversion, but the output file still showed up in the specified location.

## Observations

### No live visualisation tool

There was no live visualisation tool available while designing the robot. This was raised in class with an example, and it was considered for use in later classes.

### Navigation struggles with sharp angles

The security path struggled with large angles. Several hours went into improving repeatability, without success. Two approaches were tried.

**1. Reducing goal tolerances** in the Nav2 controller parameters:

![Goal checker tolerances](images/goal-checker-params.png)
![DWB planner parameters](images/dwb-params.png)

**2. Changing the default behaviour tree** XML in the `bt_navigator` configuration:

![BT navigator parameters](images/bt-navigator-params.png)

Neither attempt fixed the problem. It is still believed that implementing these two changes properly would let the robot drop a pose close enough to its goal (based on a radius) and improve reliability even at sharp angles.

### rosbag topic selection

Selecting all topics made the bag grow past 100 GB almost immediately, so topics needed to be chosen carefully (see [section 4](#4-record-robot-movements-with-rosbag)).

### SLAM versus no SLAM

The first map was very messy because SLAM was not in use. With SLAM running, the robot could be driven somewhat faster and the map was much cleaner, though driving still had to be fairly slow.

### Camera missing in Gazebo

The `tf2` view-frames tool showed the transform tree was set up correctly, but the camera does not appear in the Gazebo simulation. The suspected cause is the `robot_state_publisher`.

![TF frames](images/tf-frames.png)
