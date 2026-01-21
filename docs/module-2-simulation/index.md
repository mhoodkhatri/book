---
sidebar_position: 1
title: "Module 2: The Digital Twin"
description: "Create realistic robot simulations using Gazebo and Unity. Build digital twins to test robot behaviors safely before deploying on real hardware."
keywords: [Gazebo, Unity, simulation, digital twin, physics, sensors, robotics, testing, virtual environment]
---

# Module 2: The Digital Twin

**Simulation with Gazebo & Unity** | Weeks 6-8

---

## Module Overview

Imagine you are building a humanoid robot that needs to walk, pick up objects, and navigate through rooms. Every time you want to test a new walking algorithm, you would need to:

1. Upload the code to the physical robot
2. Power on the robot
3. Watch it try to walk (and probably fall)
4. Pick up the robot, check for damage
5. Fix the code
6. Repeat...

This process is **slow**, **expensive**, and **dangerous**. What if the robot falls and breaks a $10,000 motor? What if it crashes into a wall? What if it hurts someone?

This is why roboticists use **simulation**. A simulation is a virtual copy of your robot—a **digital twin**—that exists inside your computer. You can test your code thousands of times without any risk. The robot can fall, crash, and explode in simulation, and you just click "restart."

In this module, you will learn to create digital twins of your robots using two powerful tools:

- **Gazebo**: The industry-standard open-source robot simulator with accurate physics
- **Unity**: A game engine that provides photo-realistic graphics and complex environments

By the end of this module, you will be able to simulate robots with cameras, lidar, and other sensors, test your ROS 2 code in virtual worlds, and iterate on your designs rapidly and safely.

---

## Why Simulation Matters

Professional robotics companies spend enormous resources on simulation. Here is why:

### The Cost of Real-World Testing

| Problem | Real Hardware | Simulation |
|---------|--------------|------------|
| Robot falls | Potential damage, repair costs | Click restart |
| Testing 1000 scenarios | Weeks of work | Hours of compute |
| Testing dangerous situations | Impossible safely | No risk |
| Changing robot design | Order new parts, wait weeks | Edit a file |
| Running overnight tests | Needs supervision | Runs automatically |

### The Simulation-Reality Gap

There is an important concept you must understand: simulation is **not** reality. A robot that works perfectly in simulation may fail in the real world. This difference is called the **sim-to-real gap**.

Good roboticists use simulation for:
- Rapid prototyping and debugging
- Testing edge cases and dangerous scenarios
- Training machine learning models
- Validating algorithms before hardware tests

But they always validate on real hardware before deployment. Simulation is a powerful tool, not a replacement for real-world testing.

---

## Prerequisites

Before starting this module, ensure you have:

- [ ] **Completed Module 1**: Understand nodes, topics, services, and URDF
- [ ] **ROS 2 Humble installed**: Required for Gazebo integration
- [ ] **GPU with OpenGL 3.3+ support**: Required for Gazebo rendering
- [ ] **At least 8GB RAM**: Simulation is memory-intensive
- [ ] **Ubuntu 22.04**: Recommended for best compatibility

### Environment Verification

Verify your system is ready:

```bash
# Check OpenGL support
glxinfo | grep "OpenGL version"
# Expected: OpenGL version 3.3 or higher

# Check available memory
free -h
# Expected: At least 8GB total

# Check ROS 2 is sourced
echo $ROS_DISTRO
# Expected: humble
```

:::caution GPU Required
Gazebo requires a GPU for rendering. If you are using a virtual machine or remote server, ensure GPU passthrough is configured. Headless simulation is possible but limited.
:::

---

## What You Will Learn

By the end of this module, you will be able to:

1. **Set up Gazebo Fortress** with ROS 2 Humble and understand the simulator architecture
2. **Create simulation worlds** with terrain, objects, lighting, and physics properties
3. **Spawn URDF robots** into Gazebo and control them via ROS 2 topics
4. **Configure physics engines** to balance realism and performance
5. **Simulate sensors** including cameras, lidar, IMU, and depth sensors
6. **Integrate Unity** for photo-realistic rendering and complex environment design
7. **Bridge simulation and ROS 2** for seamless communication

---

## Chapter Overview

### Chapter 1: Gazebo Basics

The first chapter introduces Gazebo Fortress, the simulation tool you will use most frequently. You will learn how Gazebo works internally, how to launch simulations, create world files, and spawn your URDF robots into virtual environments. By the end, you will have a working simulation with your robot driving around.

**Key Topics**: Gazebo architecture, SDF world files, model spawning, ros_gz bridge, simulation control

[Begin Chapter 1 →](/docs/module-2-simulation/gazebo-basics)

---

### Chapter 2: Physics Simulation

This chapter dives into the physics engine that makes simulation realistic. You will learn how Gazebo computes forces, detects collisions, and simulates dynamics. Understanding physics parameters is crucial for creating simulations that transfer well to real hardware.

**Key Topics**: Physics engines (DART, ODE), collision detection, friction, inertia, contact dynamics, performance tuning

[Begin Chapter 2 →](/docs/module-2-simulation/physics-sim)

---

### Chapter 3: Unity Integration

While Gazebo excels at physics, Unity provides superior graphics and environment complexity. This chapter shows you how to use Unity for scenarios requiring photo-realistic rendering, procedural environment generation, or leveraging Unity's vast asset ecosystem.

**Key Topics**: Unity-ROS 2 bridge, scene creation, sensor plugins, hybrid simulation workflows

[Begin Chapter 3 →](/docs/module-2-simulation/unity-integration)

---

### Chapter 4: Sensor Simulation

Robots perceive the world through sensors. This chapter teaches you to simulate cameras, lidar, IMU, depth sensors, and more. You will learn to add realistic noise models so your perception algorithms are tested against imperfect data—just like in the real world.

**Key Topics**: Camera simulation, lidar point clouds, IMU modeling, noise models, sensor fusion preparation

[Begin Chapter 4 →](/docs/module-2-simulation/sensors)

---

## Learning Path

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│   Chapter 1     │     │   Chapter 2     │     │   Chapter 3     │     │   Chapter 4     │
│ Gazebo Basics   │────▶│ Physics Sim     │────▶│ Unity Integration────▶│ Sensor Sim      │
└────────┬────────┘     └────────┬────────┘     └────────┬────────┘     └────────┬────────┘
         │                       │                       │                       │
         ▼                       ▼                       ▼                       ▼
   Launch & Spawn          Configure              Photo-realistic        Complete
   Simulations             Dynamics               Rendering              Digital Twin
```

**Suggested Pace**:
- Chapter 1: 4-5 hours (core foundation, experiment with worlds)
- Chapter 2: 3-4 hours (physics tuning requires iteration)
- Chapter 3: 4-5 hours (Unity setup and ROS bridge)
- Chapter 4: 4-5 hours (multiple sensor types to configure)
- **Total Module Time**: 15-19 hours over 2-3 weeks

---

## Module Outcomes

Upon completing this module, you will have built:

| Project | Description | Skills Demonstrated |
|---------|-------------|---------------------|
| First Gazebo World | Custom environment with ground, walls, objects | SDF, world files, Gazebo UI |
| Mobile Robot Simulation | Your URDF robot driving in Gazebo | Spawning, ros_gz bridge, teleop |
| Physics-Tuned Robot | Robot with realistic friction and contacts | Physics parameters, dynamics |
| Multi-Sensor Robot | Robot with camera, lidar, and IMU | Sensor plugins, noise models |

These projects prepare you for Module 3, where you will add AI-powered perception and navigation to your simulated robots using NVIDIA Isaac.

---

## Getting Started

Ready to build your robot's digital twin? Begin with Chapter 1 to set up Gazebo and launch your first simulation.

[Begin Chapter 1: Gazebo Basics →](/docs/module-2-simulation/gazebo-basics)

---

## Additional Resources

- [Gazebo Fortress Documentation](https://gazebosim.org/docs/fortress) - Official Gazebo documentation
- [ros_gz Repository](https://github.com/gazebosim/ros_gz) - ROS 2 integration packages
- [SDF Specification](http://sdformat.org/spec) - World file format reference
- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub) - Unity-ROS integration
