---
sidebar_position: 6
title: Planning and Navigation in ROS 2
---

# Chapter 6: Planning and Navigation in ROS 2

## Introduction
For a Physical AI robot to operate autonomously, it must be able to plan its actions and navigate its environment effectively. This chapter focuses on planning and navigation concepts within ROS 2, covering topics such as path planning, motion control, and the integration of these functionalities using the Nav2 stack. A well-designed navigation system is crucial for enabling robots to reach goals while avoiding obstacles.

## Core Concepts

### Path Planning
Path planning involves determining a collision-free route for a robot from a starting point to a target destination. This can be broken down into global and local planning:
- **Global Planner**: Generates an initial path from the start to goal, typically on a static map. This path might be optimal but doesn't account for dynamic obstacles.
- **Local Planner**: Refines the global path dynamically, considering real-time sensor data to avoid immediate obstacles and smooth the robot's trajectory.

**Common Planning Algorithms:**
- **Dijkstra's Algorithm / A* (A-star)**: Widely used for global path planning on grid maps.
- **RRT (Rapidly-exploring Random Tree) / PRM (Probabilistic Road Map)**: Sampling-based algorithms often used in high-dimensional or complex environments.
- **DWA (Dynamic Window Approach) / TEB (Timed Elastic Band)**: Popular local planning algorithms that generate safe and dynamically feasible trajectories.

### Motion Control
Motion control is the execution of planned paths. It involves generating velocity commands for the robot's actuators to follow the desired trajectory. Controllers translate desired positions or velocities into motor commands.
- **PID Controllers**: Proportional-Integral-Derivative controllers are commonly used to regulate robot joint positions or wheel velocities.
- **Trajectory Tracking**: Ensuring the robot accurately follows the planned path, often accounting for kinematic and dynamic constraints.

### Nav2 Stack in ROS 2
Nav2 is the next-generation navigation stack for ROS 2, providing a full suite of tools for autonomous mobile robot navigation. It is highly modular and configurable, allowing developers to choose different plugins for each stage of the navigation process.

**Key Components of Nav2:**
- **`amcl` (Adaptive Monte Carlo Localization)**: For probabilistic localization using a 2D occupancy grid map and sensor data (e.g., LiDAR).
- **`map_server`**: Provides map data to other Nav2 components.
- **`global_planner`**: Generates a long-term path.
- **`local_planner` (Controller Server)**: Generates velocity commands to follow the path and avoid obstacles.
- **`recovery_behaviors`**: Handles situations where the robot gets stuck or encounters an unexpected state (e.g., rotating in place, clearing costmaps).
- **`bt_navigator` (Behavior Tree Navigator)**: Orchestrates the entire navigation process using behavior trees, allowing for flexible and robust navigation logic.
- **`costmap_2d`**: Maintains a 2D occupancy grid map of the environment, incorporating static map data and dynamic obstacle information from sensors.

## Practical Application / Code Examples

### 1. Simple Nav2 Setup (Conceptual Walkthrough)

This walkthrough outlines the general steps to set up Nav2 for a robot, assuming you have a robot URDF (from Chapter 3) and a simulation environment (from Chapter 4) ready.

1.  **Map Creation**: Generate a map of your environment. This can be done by teleoperating your robot in a simulated environment and running a SLAM (Simultaneous Localization and Mapping) node (e.g., `slam_toolbox`).
    ```bash
    # Example: launch SLAM Toolbox and save map
    ros2 launch slam_toolbox online_async_launch.py
    # After mapping, save the map
    ros2 run nav2_map_server map_saver_cli -f my_environment_map
    ```
2.  **Launch Nav2**: Use a Nav2 launch file to bring up all necessary components. Nav2 provides `bringup` packages for common setups.
    ```bash
    # Example: launch Nav2 with your map and robot
    ros2 launch nav2_bringup bringup_launch.py \
        map:=/path/to/my_environment_map.yaml \
        use_sim_time:=True # Set to False for real robot
    ```
3.  **Send Navigation Goals**: Use RViz2's "2D Nav Goal" tool or a ROS 2 command line interface to send navigation goals to your robot.
    ```bash
    # Example: send a simple navigation goal via command line
    ros2 action send_goal navigate_to_pose nav2_msgs/action/NavigateToPose \
    '{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}' \
    --feedback
    ```

### 2. Custom Global Planner Plugin (Conceptual)

This illustrates the structure of a custom global planner plugin for Nav2 in Python. You would implement your path planning logic in the `createPlan` method.

```python
import rclpy
from rclpy.node import Node
from nav2_core.global_planner import GlobalPlanner
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class CustomGlobalPlanner(Node, GlobalPlanner):
    def __init__(self):
        super().__init__('custom_global_planner')
        GlobalPlanner.__init__(self)

    def configure(self, parent_node, name, tf, costmap_ros):
        self.parent_node = parent_node
        self.tf = tf
        self.costmap = costmap_ros.getCostmap()
        self.global_frame = costmap_ros.getGlobalFrameID()
        self.name = name
        self.parent_node.get_logger().info(f'Initializing custom global planner: {self.name}')

    def createPlan(self, start, goal):
        self.parent_node.get_logger().info(f'Creating plan from {start.pose.position} to {goal.pose.position}')
        path = Path()
        path.header.frame_id = self.global_frame
        path.header.stamp = self.parent_node.get_clock().now().to_msg()

        # --- Implement your custom path planning logic here ---
        # For demonstration, a direct line path
        path.poses.append(start)
        # Intermediate points could be added based on algorithm
        path.poses.append(goal)
        # --------------------------------------------------------

        return path

    def activate(self):
        self.parent_node.get_logger().info(f'Activating custom global planner: {self.name}')

    def deactivate(self,):
        self.parent_node.get_logger().info(f'Deactivating custom global planner: {self.name}')

    def cleanup(self):
        self.parent_node.get_logger().info(f'Cleaning up custom global planner: {self.name}')

# To make this a plugin, it needs to be registered in your package's `plugin.xml`
# and loaded by the Nav2 stack.
```

## Diagrams and Visualizations

### Nav2 Stack High-Level Architecture
```mermaid
graph TD
    A[Robot Hardware/Simulation] --> B(Sensors / Actuators)
    B --> C[Robot Localization (AMCL)]
    B --> D[Costmap2D (Static Map + Sensor Data)]
    C --> E[Behavior Tree Navigator]
    D --> E
    E --> F[Global Planner]
    E --> G[Local Planner (Controller)]
    F --> E
    G --> E
    E --> H[Recovery Behaviors]
    H --> E
    G --> B
```

### Global vs. Local Planning
```mermaid
graph LR
    Start[Start Pose] -- Global Path (Static Map) --> Goal[Goal Pose]
    SubGraph[Obstacles / Dynamic Environment]
        Start -- Local Path (Dynamic Avoidance) --> Temp1(Intermediate Point 1)
        Temp1 --> Temp2(Intermediate Point 2)
        Temp2 --> Goal
    end
```

## Summary
Autonomous robot operation hinges on effective planning and navigation. Path planning, encompassing global and local strategies, generates collision-free routes, while motion control ensures accurate trajectory execution. The ROS 2 Nav2 stack provides a comprehensive, modular framework for these functionalities, integrating components like AMCL for localization, costmaps for environmental representation, and various planners and recovery behaviors. Understanding and implementing these concepts is fundamental for developing robots capable of intelligent movement in complex environments.

## Exercises / Discussion Questions
1.  Explain the interaction between global and local planners in Nav2. Why is both necessary for robust navigation?
2.  Describe how the `costmap_2d` works and how it integrates sensor data with a static map to enable obstacle avoidance.
3.  Propose a custom recovery behavior for Nav2 to handle a robot stuck in a narrow corridor. Outline the logic and how it would interact with other Nav2 components.

## References
- ROS 2 Nav2 Documentation. (n.d.). *Nav2 Overview*. Retrieved from https://navigation.ros.org/
- Choset, H., Lynch, K. M., Hutchinson, S., Kantor, G. A., Burgard, W., Kavraki, L. E., & Thrun, S. (2005). *Principles of Robot Motion: Theory, Algorithms, and Implementations*. MIT Press.
---
