---
sidebar_position: 7
title: Control Systems and Actuation
---

# Chapter 7: Control Systems and Actuation

## Introduction
Effective control of robot actuators is paramount for Physical AI to execute planned movements and interact precisely with the environment. This chapter delves into the fundamentals of control systems, focusing on common control strategies like PID, and explores various types of actuators used in robotics, particularly those relevant to humanoid systems. Understanding how to command and sense the physical state of a robot is essential for bringing AI plans to life.

## Core Concepts

### Basics of Control Systems
A control system regulates the behavior of a dynamic system to achieve a desired output. Key concepts include:
- **Open-Loop Control**: Commands are sent to actuators without feedback from the system's output. Simple but susceptible to disturbances and model inaccuracies.
- **Closed-Loop Control (Feedback Control)**: Uses sensor feedback to continuously adjust control inputs, making the system more robust and accurate. This is the foundation of most robotic control.
- **Setpoint/Reference**: The desired value for the system's output (e.g., target joint angle, desired motor speed).
- **Process Variable**: The actual measured output of the system.
- **Error**: The difference between the setpoint and the process variable.

### PID Controllers
Proportional-Integral-Derivative (PID) controllers are the most widely used feedback control loops in industrial control systems and robotics due to their simplicity and effectiveness. A PID controller continuously calculates an error value and applies a correction based on three terms:
- **Proportional (P) Term**: Corrects the error based on its current value. A larger P-gain leads to a faster response but can cause oscillations.
- **Integral (I) Term**: Corrects accumulated past errors, helping to eliminate steady-state errors (offset).
- **Derivative (D) Term**: Corrects based on the rate of change of the error, helping to dampen oscillations and improve stability.

The output of a PID controller is typically a command (e.g., motor voltage, torque) to reduce the error.

### Actuators in Robotics
Actuators are components that convert energy (electrical, hydraulic, pneumatic) into physical motion, enabling robots to move and exert force. For humanoid robots, the ability to control multiple degrees of freedom (DoF) with high precision is critical.

**Common Types of Actuators:**
- **Electric Motors**: The most prevalent type in robotics, offering various forms:
    - **DC Motors**: Simple, inexpensive, but generally less precise.
    - **Brushless DC (BLDC) Motors**: High efficiency, long lifespan, and precise control, often used with sophisticated motor drivers.
    - **Servo Motors**: Integrated motor, gearbox, and control electronics, providing precise angular positioning. Common in smaller robots.
    - **Stepper Motors**: Move in discrete steps, useful for precise open-loop positioning, but can lose steps under load.
- **Hydraulic Actuators**: Provide very high power and force density, typically used in heavy-duty industrial robots or strong humanoid legs.
- **Pneumatic Actuators**: Operate with compressed air, offering high speeds and compliance, suitable for grippers or impact-tolerant movements.
- **Series Elastic Actuators (SEAs)**: Incorporate a spring in series with a motor, allowing for compliant interaction with the environment, force sensing, and shock absorption. Crucial for safe human-robot interaction in humanoids.

## Practical Application / Code Examples

### 1. Basic PID Controller Implementation (Python)

This example demonstrates a simple Python PID controller for a single joint, conceptually controlling its position. In ROS 2, this logic would typically reside in a controller node publishing velocity/effort commands.

```python
class PIDController:
    def __init__(self, kp, ki, kd, output_limits=(-1.0, 1.0)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits

        self.p_term = 0
        self.i_term = 0
        self.d_term = 0
        self.last_error = 0
        self.integral_limit = 0.5 # To prevent integral windup

    def compute(self, setpoint, process_variable, dt):
        error = setpoint - process_variable

        self.p_term = self.kp * error

        self.i_term += self.ki * error * dt
        # Anti-windup
        self.i_term = max(min(self.i_term, self.integral_limit), -self.integral_limit)

        self.d_term = self.kd * (error - self.last_error) / dt if dt > 0 else 0
        self.last_error = error

        output = self.p_term + self.i_term + self.d_term

        # Clamp output to limits
        output = max(min(output, self.output_limits[1]), self.output_limits[0])

        return output

# Example Usage (conceptual simulation loop)
if __name__ == '__main__':
    pid = PIDController(kp=0.5, ki=0.1, kd=0.05)
    setpoint = 1.0 # desired joint position
    current_position = 0.0
    dt = 0.01 # simulation time step

    print("Time | Setpoint | Current Pos | Error | Output Command")
    for t in range(100):
        error = setpoint - current_position
        control_command = pid.compute(setpoint, current_position, dt)

        # Simulate actuator response (very simplified)
        current_position += control_command * dt * 0.5 # robot moves towards target

        print(f"{t*dt:.2f} | {setpoint:.2f} | {current_position:.2f} | {error:.2f} | {control_command:.2f}")
        if abs(error) < 0.01 and abs(control_command) < 0.01:
            print(f"Target reached at time {t*dt:.2f}")
            break
```

### 2. ROS 2 Control with `ros2_control` (Conceptual)

`ros2_control` is a generic control framework for ROS 2 that provides a standardized way to interface with robot hardware and implement controllers. It decouples controllers from hardware interfaces, allowing for flexible and reusable control architectures.

**Key Components:**
- **Hardware Interface**: Manages communication with physical robot hardware (motors, sensors).
- **Controller Manager**: Loads, unloads, starts, and stops controllers.
- **Controllers**: Implement control logic (e.g., joint position controller, joint trajectory controller, force/torque controller).

**Configuration (Example `ros2_control` YAML for a simple robot):**
```yaml
# controllers/simple_robot_controller.yaml

controller_manager:
  ros__parameters:
    update_rate: 100 # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    joint_position_controller:
      type: position_controllers/JointGroupPositionController

      joint_position_controller:
        ros__parameters:
          joints:
            - joint1 # From our URDF in Chapter 3
          interface_name: position
          command_interface: position
          state_interface: position
          state_publish_rate: 50 # Hz

```

This YAML configuration would be loaded with `ros2_control` launch files to bring up the controller manager and specific controllers.

## Diagrams and Visualizations

### Closed-Loop Control System Diagram
```mermaid
graph LR
    S[Setpoint] --> E(Error Calculation)
    PV[Process Variable (Sensor Feedback)] --> E
    E --> PID(PID Controller)
    PID --> C[Control Command]
    C --> P{Process (Robot / Actuator)}
    P --> PV
```

### `ros2_control` High-Level Architecture
```mermaid
graph TD
    HW[Robot Hardware] --> HI[Hardware Interface]
    HI --> CM[Controller Manager]
    CM --> C1[Controller 1 (e.g., Joint Position)]
    CM --> C2[Controller 2 (e.g., Joint Velocity)]
    C1 --> HI
    C2 --> HI
    CM --> ROS2[ROS 2 System (Nodes, Topics)]
    ROS2 --> C1
    ROS2 --> C2
```

## Summary
Control systems are essential for enabling Physical AI robots to execute precise movements. Closed-loop feedback control, particularly using PID controllers, forms the backbone of most robotic control, ensuring accuracy and stability. A variety of actuators, from electric motors (BLDC, servo) to hydraulics and pneumatics, provide the physical force, with advancements like Series Elastic Actuators offering compliance for safe interaction. ROS 2's `ros2_control` framework provides a standardized and modular approach to integrate these control strategies and actuators, separating hardware interfaces from controller logic for flexible and reusable robotics solutions.

## Exercises / Discussion Questions
1.  Explain the phenomenon of "integral windup" in PID controllers and how anti-windup mechanisms address it. Why is this important in robotics?
2.  Compare Series Elastic Actuators (SEAs) with traditional rigid actuators. What are the advantages and disadvantages of SEAs for humanoid robots interacting with humans?
3.  Design a `ros2_control` configuration for a robot arm with three revolute joints, each controlled by a position controller. How would you expose these to ROS 2 topics for commanding?

## References
- Franklin, G. F., Powell, J. D., & Emami-Naeini, A. (2014). *Feedback Control of Dynamic Systems* (7th ed.). Pearson.
- `<ros2_control_docs_url_placeholder>` (Refer to official ROS 2 Control documentation for up-to-date links: https://control.ros.org/)
---
