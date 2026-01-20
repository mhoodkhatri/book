---
sidebar_position: 1
title: "Module 1: The Robotic Nervous System"
description: "Master ROS 2 fundamentals including nodes, topics, services, actions, and robot description with URDF for building humanoid robot communication systems."
keywords: [ROS 2, nodes, topics, services, actions, URDF, rclpy, robotics, middleware, publish-subscribe]
---

# Module 1: The Robotic Nervous System

**ROS 2 Fundamentals** | Weeks 3-5

---

## Module Overview

Just as the human nervous system coordinates signals between the brain, sensory organs, and muscles, ROS 2 (Robot Operating System 2) serves as the communication backbone for robotic systems. This module introduces the fundamental architecture that enables different components of a humanoid robot to perceive, process, and act in coordinated fashion.

ROS 2 is not an operating system in the traditional sense. Instead, ROS 2 provides middleware services that sit between the operating system and application code. These services include inter-process communication, hardware abstraction, device drivers, and tools for building, testing, and deploying robot software. The architecture enables developers to decompose complex robot behaviors into modular, reusable components that communicate through well-defined interfaces.

For humanoid robotics specifically, ROS 2 addresses critical challenges: coordinating dozens of joint actuators, fusing data from multiple sensors, maintaining real-time responsiveness for balance control, and enabling modularity so that perception, planning, and control can be developed independently. Production humanoid robots from Boston Dynamics, Agility Robotics, and research institutions worldwide rely on ROS 2 or similar middleware architectures.

This module establishes the foundation for all subsequent work in the textbook. The simulation environments in Module 2 communicate with your code through ROS 2 interfaces. The NVIDIA Isaac perception pipelines in Module 3 publish their outputs as ROS 2 messages. The vision-language-action systems in Module 4 translate language commands into ROS 2 action goals. Mastery of ROS 2 is therefore prerequisite for everything that follows.

---

## Prerequisites

Before starting this module, verify that you have:

- [ ] **Python programming experience**: Can write classes, use external libraries, handle exceptions, and work with callbacks
- [ ] **Linux command-line familiarity**: Can navigate directories, edit files with a terminal editor, and run shell commands
- [ ] **ROS 2 Humble installed**: Follow the [official installation guide](https://docs.ros.org/en/humble/Installation.html) for Ubuntu 22.04
- [ ] **Completed the [Introduction](/docs/intro)**: Understand the textbook structure and learning paths

### Environment Verification

Before proceeding, verify your ROS 2 installation:

```bash
# Source the ROS 2 environment
source /opt/ros/humble/setup.bash

# Verify ROS 2 is accessible
ros2 --version
# Expected: ros2 0.10.x

# Verify core packages
ros2 pkg list | grep -E "rclpy|std_msgs"
# Expected: rclpy, std_msgs in output
```

:::caution Installation Issues
If the commands above fail, consult the [Development Workstation](/docs/hardware/workstation) page for troubleshooting guidance before continuing.
:::

---

## What You Will Learn

By the end of this module, you will be able to:

1. **Design ROS 2 computational graphs** that decompose robot behavior into communicating nodes with clear responsibilities
2. **Implement publish-subscribe communication** using topics to stream sensor data and commands between nodes
3. **Build request-response services** for synchronous operations like configuration changes and state queries
4. **Create action servers and clients** for long-running tasks with progress feedback and cancellation support
5. **Write production-quality Python nodes** using rclpy with proper lifecycle management, executors, and parameter handling
6. **Define robot models in URDF** that accurately represent links, joints, and sensor placements for visualization and simulation

---

## Chapter Overview

### Chapter 1: Nodes and Topics

The first chapter introduces the two most fundamental ROS 2 concepts: nodes and topics. A node is an executable process that performs a specific function within the robot system. Nodes communicate by publishing messages to topics and subscribing to messages from topics. This publish-subscribe pattern enables loose coupling between components, allowing sensors, controllers, and planners to operate independently while sharing data.

**Key Topics**: Node creation, topic publishing, topic subscription, message types, Quality of Service (QoS) settings

[Begin Chapter 1 →](/docs/module-1-ros2/nodes-topics)

---

### Chapter 2: Services and Actions

While topics provide asynchronous data streams, many robot operations require synchronous request-response communication or long-running tasks with feedback. This chapter covers services for immediate request-response patterns and actions for tasks that may take significant time to complete. Actions are essential for robot behaviors like navigation, manipulation, and any operation that benefits from progress monitoring and cancellation.

**Key Topics**: Service servers, service clients, action servers, action clients, feedback mechanisms, goal handling

[Begin Chapter 2 →](/docs/module-1-ros2/services-actions)

---

### Chapter 3: Python Client Library (rclpy)

This chapter provides deep coverage of rclpy, the Python client library for ROS 2. Beyond basic node creation, the chapter addresses production concerns: executor models for handling callbacks, parameter declaration and dynamic reconfiguration, lifecycle nodes for managed startup and shutdown, and patterns for building robust, maintainable robot code.

**Key Topics**: Executors (single-threaded, multi-threaded), parameter handling, lifecycle nodes, callback groups, composition

[Begin Chapter 3 →](/docs/module-1-ros2/rclpy)

---

### Chapter 4: Robot Description (URDF)

The final chapter introduces URDF (Unified Robot Description Format), the standard XML format for describing robot geometry, kinematics, dynamics, and visual appearance. URDF models are essential for visualization in RViz, simulation in Gazebo, and motion planning with MoveIt. This chapter teaches you to create accurate robot descriptions that serve as the foundation for all subsequent simulation and planning work.

**Key Topics**: Links and joints, visual and collision geometry, inertial properties, sensor placement, Xacro macros

[Begin Chapter 4 →](/docs/module-1-ros2/urdf)

---

## Learning Path

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│   Chapter 1     │     │   Chapter 2     │     │   Chapter 3     │     │   Chapter 4     │
│ Nodes & Topics  │────▶│Services/Actions │────▶│     rclpy       │────▶│      URDF       │
└────────┬────────┘     └────────┬────────┘     └────────┬────────┘     └────────┬────────┘
         │                       │                       │                       │
         ▼                       ▼                       ▼                       ▼
   Publish-Subscribe        Request-Response       Production Code         Robot Models
   Communication            Patterns               Patterns                for Simulation
```

**Suggested Pace**:
- Chapter 1: 3-4 hours (foundational, take time to experiment)
- Chapter 2: 3-4 hours (builds directly on Chapter 1)
- Chapter 3: 4-5 hours (deeper Python patterns)
- Chapter 4: 3-4 hours (XML/Xacro syntax requires practice)
- **Total Module Time**: 13-17 hours over 2-3 weeks

---

## Module Outcomes and Next Steps

Upon completing this module, you will have built several working ROS 2 applications:

| Project | Description | Skills Demonstrated |
|---------|-------------|---------------------|
| Publisher-Subscriber Pair | Nodes exchanging sensor data | Topics, messages, QoS |
| Service-Based Calculator | Request-response computation | Service definition, server/client |
| Action-Based Timer | Long-running task with feedback | Action servers, goal handling |
| Simple Robot URDF | Two-link robot arm model | Links, joints, visualization |

These projects form the foundation for Module 2, where you will place your URDF robots into simulated worlds and observe your ROS 2 nodes interacting with physics-based sensors and actuators.

---

## Getting Started

Ready to build the nervous system of your robot? Begin with Chapter 1 to learn how nodes and topics enable modular, scalable robot software.

[Begin Chapter 1: Nodes and Topics →](/docs/module-1-ros2/nodes-topics)

---

## Additional Resources

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/) - Official API reference and tutorials
- [ROS 2 Design Documents](https://design.ros2.org/) - Architecture decisions and rationale
- [rclpy API Reference](https://docs.ros2.org/latest/api/rclpy/) - Python client library documentation
- [URDF Specification](http://wiki.ros.org/urdf/XML) - Complete URDF XML reference
