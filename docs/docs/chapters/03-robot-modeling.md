---
sidebar_position: 3
title: Robot Modeling with URDF and TF
---

# Chapter 3: Robot Modeling with URDF and TF

## Introduction
Accurate representation of robots is fundamental for simulation, planning, and control in Physical AI. This chapter introduces the Universal Robot Description Format (URDF) for modeling robot kinematics and dynamics, and the TF (Transform) system for managing coordinate frames. We will explore how to create and visualize URDF models, and understand the importance of TF for spatial reasoning in robotics.

## Core Concepts

### Universal Robot Description Format (URDF)
URDF is an XML format used in ROS 2 to describe a robot's kinematic and dynamic properties. It defines the robot's structure as a tree of `link` and `joint` elements.
- **Links**: Represent the rigid bodies of the robot (e.g., base, arm segments, end-effector). Each link has visual, inertial, and collision properties.
- **Joints**: Define the kinematic and dynamic properties of the connection between two links. Joints can be `revolute` (rotating), `prismatic` (sliding), `fixed`, `continuous`, `planar`, or `floating`.

URDF models allow for:
- **Visualization**: Displaying the robot in simulation environments like Gazebo or RViz.
- **Kinematics**: Calculating the position and orientation of robot parts.
- **Dynamics**: Simulating the physical behavior of the robot, considering mass, inertia, and forces.

### TF (Transform) System
The TF (Transform) system in ROS 2 keeps track of multiple coordinate frames and allows you to transform points, vectors, and other entities between them. A robot typically has many coordinate frames (e.g., base link, camera frame, end-effector frame), and TF provides a robust way to manage their relationships over time.

Key aspects of TF:
- **Transform Trees**: All coordinate frames are organized into a tree structure, with transformations defined between parent and child frames.
- **Broadcasters**: Nodes that publish the transformations between frames.
- **Listeners**: Nodes that receive and use the transformations to convert data between frames.
- **Time-variant Data**: TF handles time-stamped transformations, allowing for accurate conversion of data that was recorded at different points in time.

## Practical Application / Code Examples

Let's create a simple two-link robot URDF and visualize it using ROS 2 tools.

### 1. Create a Robot Description Package
```bash
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
ros2 pkg create --build-type ament_cmake my_robot_description
```

### 2. Define `simple_robot.urdf` (`my_robot_description/urdf/simple_robot.urdf`)
```xml
<?xml version="1.0"?>
<robot name="simple_robot">

  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.6 0.4 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <link name="link1">
    <visual>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="100"/>
  </joint>

</robot>
```

### 3. Update `CMakeLists.txt` (`my_robot_description/CMakeLists.txt`)
```cmake
cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(my_robot_description)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(rviz2 REQUIRED)
find_package(robot_state_publisher REQUIRED)

install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})

ament_package()
```

### 4. Create a Launch File for Visualization (`my_robot_description/launch/display_robot.launch.py`)
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get URDF file path
    urdf_file = os.path.join(
        get_package_share_directory('my_robot_description'),
        'urdf',
        'simple_robot.urdf')

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_file).read()}])

    # RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('my_robot_description'), 'rviz', 'urdf.rviz')])

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node
    ])
```

### 5. Create a basic RViz configuration file (`my_robot_description/rviz/urdf.rviz`)
```yaml
Panels:
  - Class: rviz_default_plugins/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /TF1
        - /RobotModel1
      Splitter Ratio: 0.5
    Tree Height: 200
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/TF
      Alpha: 0.5
      Color: 0 255 0
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: true
      Update Interval: 0
      Value: true
    - Class: rviz_default_plugins/RobotModel
      Alpha: 0.5
      Collision Enabled: false
      Links:
        All Links Enabled: true
        Expand Joint Details: false
        Expand Link Details: false
        Expand Tree: false
        Link Tree Style: "" # empty string means list
        base_link:
          Alpha: 1
          Show Axes: false
          Show Trail: false
        link1:
          Alpha: 1
          Show Axes: false
          Show Trail: false
      Name: RobotModel
      Robot Description: robot_description
      TF Prefix: ""
      Update Interval: 0
      Value: true
      Visual Enabled: true
  Enabled: true
  Global Options:
    Background Color: 48 48 48
    Fixed Frame: base_link
    Frame Rate: 30
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/SetInitialPose
      Topic: /initialpose
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
    - Class: rviz_default_plugins/PublishPoint
      Single click: true
      Topic: /clicked_point
    - Class: rviz_default_plugins/MovePanel
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 2.5
      Focal Point: 0 0 0
      Near Clip Distance: 0.01
      Yaw: 0
      Pitch: 0.5
      Target Frame: <Fixed Frame>
    Saved: ~ # empty list or object


Window Geometry:
  Displays Enabled: true
  Hide Menus: false
  Hide Status Bar: false
  Modified: false
  Show Toolbar: true
  Splitter Ratio: 0.8
  Staus Bar Height: 20
  Full Screen: false



Keybinding Manager:
  "Shift+F10":
    Action: "Reload"
    Category: "File"
  "Control+S":
    Action: "Save"
    Category: "File"


Time:
  Frame Rate: 30
  "Sync To Current Time": true
  Update Interval: 0.03333333333333333
  Value: 1701900000.000000000
  Widgets:
    Current:
      Controller:
        Class: rviz_default_plugins/Playback
        Rate: 1
        Loop: false
    Saved: ~ # empty list or object



```

### 6. Build and Launch
```bash
cd ~/robot_ws
colcon build
source install/setup.bash
ros2 launch my_robot_description display_robot.launch.py
```
This will launch RViz2, where you can see your `simple_robot` model.

## Diagrams and Visualizations

A conceptual diagram of a URDF robot model and TF frames:

```mermaid
graph TD
    A[Robot Description (URDF)] --> B[Links]
    A --> C[Joints]
    B --> D[Visual Properties]
    B --> E[Inertial Properties]
    B --> F[Collision Properties]
    C --> G[Joint Type (Revolute, Prismatic, Fixed)]
    C --> H[Parent Link]
    C --> I[Child Link]
    J[TF Tree] --> K[Frame A]
    J --> L[Frame B]
    K --> M[Transform (Rotation + Translation)]
    M --> L
```

## Summary
URDF and TF are indispensable tools for robot modeling and spatial reasoning in ROS 2. URDF defines a robot's physical structure, enabling visualization and simulation, while TF dynamically manages the relationships between coordinate frames. Together, they provide a consistent and powerful framework for developing complex robotic applications, ensuring that all components have an accurate understanding of their position and orientation in the world.

## Exercises / Discussion Questions
1.  Modify the `simple_robot.urdf` to add a second revolute joint and another link. Visualize the changes in RViz2.
2.  Explain how the TF system handles time-variant transformations. Why is this important for robotics?
3.  How would you incorporate sensor frames (e.g., camera, lidar) into the URDF and TF tree of a mobile robot?

## References
- ROS 2 Documentation. (n.d.). *URDF Overview*. Retrieved from https://docs.ros.org/en/humble/Tutorials/URDF/URDF-Crate.html
- ROS 2 Documentation. (n.d.). *Understanding TF2*. Retrieved from https://docs.ros.org/en/humble/Tutorials/Tf2/Tf2-Introduction.html
---
