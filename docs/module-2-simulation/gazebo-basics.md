---
sidebar_position: 2
title: "Chapter 1: Gazebo Basics"
description: "Get started with Gazebo robot simulator. Learn to create worlds, spawn URDF robots, and bridge Gazebo with ROS 2 for complete simulation."
keywords: [Gazebo, simulation, SDF, worlds, robot spawning, ROS 2, ros_gz, digital twin]
---

# Chapter 1: Gazebo Basics

## Chapter Overview

In Module 1, you created a URDF file that describes your robot's physical structure. But a URDF file is just a description—it does not do anything by itself. To see your robot move, interact with objects, and test your code, you need a **simulator**.

**Gazebo** is the most widely used open-source robot simulator. It takes your URDF robot description and brings it to life in a 3D virtual world. Inside Gazebo, your robot can drive, walk, pick up objects, and sense its environment—all without any real hardware.

Think of Gazebo as a **video game for robots**. You are the game designer: you create the world (terrain, objects, lighting), place your robot in it, and then control the robot using ROS 2 exactly as you would with real hardware. The physics engine handles gravity, collisions, and dynamics automatically.

**Prerequisites**: Completed [Module 1](/docs/module-1-ros2) (especially [Chapter 4: URDF](/docs/module-1-ros2/urdf))

**What You Will Learn**: Launch Gazebo, create worlds, spawn robots, and bridge to ROS 2

---

## Learning Objectives

After completing this chapter, you will be able to:

- Explain what Gazebo is and how it integrates with ROS 2
- Install Gazebo Fortress and the ros_gz bridge packages
- Create simulation worlds using SDF (Simulation Description Format)
- Spawn your URDF robot into a Gazebo world
- Control your simulated robot using ROS 2 topics
- Use Gazebo's built-in tools to inspect and debug your simulation

---

## What Is Gazebo?

Gazebo is a 3D robot simulator that provides:

1. **Physics Simulation**: Gravity, collisions, friction, and forces
2. **Sensor Simulation**: Cameras, lidar, IMU, and other sensors
3. **3D Rendering**: Visualize your robot and environment
4. **ROS 2 Integration**: Control robots using the same code you would use on real hardware

### The Gazebo Architecture

Understanding how Gazebo works helps you debug problems and configure simulations effectively.

```
┌─────────────────────────────────────────────────────────────────┐
│                       GAZEBO ARCHITECTURE                        │
│                                                                  │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │                    GAZEBO SIMULATION                       │   │
│  │                                                            │   │
│  │   ┌───────────┐    ┌───────────┐    ┌───────────┐        │   │
│  │   │  Physics  │    │ Rendering │    │  Sensors  │        │   │
│  │   │  Engine   │    │  Engine   │    │  Plugins  │        │   │
│  │   └─────┬─────┘    └─────┬─────┘    └─────┬─────┘        │   │
│  │         │                │                │               │   │
│  │         └────────────────┼────────────────┘               │   │
│  │                          │                                 │   │
│  │                    ┌─────┴─────┐                          │   │
│  │                    │   World   │                          │   │
│  │                    │  (Models, │                          │   │
│  │                    │  Robots)  │                          │   │
│  │                    └───────────┘                          │   │
│  └──────────────────────────────────────────────────────────┘   │
│                              │                                   │
│                              │  ros_gz bridge                    │
│                              ▼                                   │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │                      ROS 2 NETWORK                         │   │
│  │                                                            │   │
│  │     /cmd_vel        /scan         /camera/image            │   │
│  │     (velocity)      (lidar)       (camera)                 │   │
│  │                                                            │   │
│  └──────────────────────────────────────────────────────────┘   │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

The key insight is that Gazebo and ROS 2 are **separate systems** connected by a **bridge**. Gazebo runs the physics and rendering; the ros_gz bridge translates Gazebo messages into ROS 2 topics and vice versa.

### Gazebo Versions: A Quick Note

Gazebo has gone through several naming changes:

| Name | Status | ROS 2 Version |
|------|--------|---------------|
| Gazebo Classic (gazebo11) | Legacy, still works | Foxy, Humble |
| Gazebo Fortress | LTS, recommended | Humble |
| Gazebo Garden/Harmonic | Newer releases | Humble, Iron |

This textbook uses **Gazebo Fortress** because it is the Long-Term Support (LTS) release that pairs with ROS 2 Humble.

---

## Installing Gazebo and ROS 2 Integration

Let us set up Gazebo Fortress with ROS 2 Humble.

### Step 1: Install Gazebo Fortress

```bash
# Add Gazebo repository
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Update and install
sudo apt update
sudo apt install gz-fortress -y
```

### Step 2: Install ROS-Gazebo Bridge Packages

```bash
# Install the bridge packages for ROS 2 Humble
sudo apt install ros-humble-ros-gz -y

# This installs:
# - ros_gz_bridge: Message translation between ROS 2 and Gazebo
# - ros_gz_sim: Launch and control Gazebo from ROS 2
# - ros_gz_image: Bridge for camera images
```

### Step 3: Verify Installation

```bash
# Test Gazebo launches
gz sim --version
# Expected: Gazebo Sim, version 6.x.x

# Test with a demo world
gz sim shapes.sdf
# A window should open showing basic shapes
```

If a window opens showing colored shapes (box, sphere, cylinder), your installation is working. Close the window to continue.

---

## Understanding SDF: The World Description Format

In Module 1, you learned URDF for describing robots. Gazebo uses a different format called **SDF** (Simulation Description Format) for describing entire simulation worlds.

### URDF vs SDF: What Is the Difference?

| Feature | URDF | SDF |
|---------|------|-----|
| Purpose | Describe a single robot | Describe entire worlds |
| Contains | Links, joints | Models, lights, physics, plugins |
| Used by | ROS 2 tools, RViz | Gazebo simulation |
| File extension | `.urdf` or `.xacro` | `.sdf` or `.world` |

Think of it this way: **URDF describes the actor (your robot), SDF describes the stage (the world)**.

The good news: Gazebo can load URDF files directly. You do not need to rewrite your robot in SDF. Gazebo converts URDF to SDF internally.

### Your First SDF World

Let us create a simple world with a ground plane and some lighting:

```xml
<?xml version="1.0" ?>
<!-- File: my_world.sdf -->
<!-- A simple Gazebo world with ground and lighting -->

<sdf version="1.8">
  <world name="my_first_world">

    <!-- Physics configuration -->
    <physics name="1ms" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Plugins for Gazebo functionality -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>

    <!-- Sunlight from above -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

Let me explain the key parts:

**`<physics>`**: Configures how the physics engine runs. `max_step_size` is how often physics updates (1ms = 1000 Hz). `real_time_factor` of 1.0 means simulation runs at real-world speed.

**`<plugin>`**: Gazebo uses plugins for functionality. The three plugins above are essential—physics computation, broadcasting the scene to viewers, and accepting user commands.

**`<light>`**: Without light, your world is black. This creates a sun-like directional light.

**`<model>`**: Everything in Gazebo is a model. Even the ground is a model with a flat plane shape. `<static>true</static>` means this model does not move.

### Launching Your World

```bash
# Save the file as my_world.sdf, then launch
gz sim my_world.sdf
```

You should see an empty world with a gray ground plane. Use your mouse to navigate:
- **Left-click + drag**: Rotate view
- **Right-click + drag**: Pan view
- **Scroll wheel**: Zoom in/out

---

## Adding Objects to Your World

An empty world is boring. Let us add some objects for your robot to interact with.

### Adding a Simple Box

Add this inside your `<world>` tags, after the ground plane:

```xml
<!-- A red box obstacle -->
<model name="red_box">
  <static>false</static>  <!-- This box can move if pushed -->
  <pose>2 0 0.5 0 0 0</pose>  <!-- x y z roll pitch yaw -->
  <link name="link">
    <collision name="collision">
      <geometry>
        <box>
          <size>1 1 1</size>  <!-- 1 meter cube -->
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
      <material>
        <ambient>1 0 0 1</ambient>  <!-- Red color -->
        <diffuse>1 0 0 1</diffuse>
      </material>
    </visual>
    <inertial>
      <mass>5.0</mass>  <!-- 5 kg box -->
      <inertia>
        <ixx>0.833</ixx>
        <iyy>0.833</iyy>
        <izz>0.833</izz>
      </inertia>
    </inertial>
  </link>
</model>
```

The `<pose>` element positions the box at x=2 meters (in front of origin), z=0.5 meters (so the bottom sits on the ground, since the box is 1m tall and centered).

### Adding a Cylinder and Sphere

```xml
<!-- A blue cylinder -->
<model name="blue_cylinder">
  <static>false</static>
  <pose>-2 1 0.5 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry>
        <cylinder>
          <radius>0.3</radius>
          <length>1.0</length>
        </cylinder>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <cylinder>
          <radius>0.3</radius>
          <length>1.0</length>
        </cylinder>
      </geometry>
      <material>
        <ambient>0 0 1 1</ambient>
        <diffuse>0 0 1 1</diffuse>
      </material>
    </visual>
    <inertial>
      <mass>2.0</mass>
    </inertial>
  </link>
</model>

<!-- A green sphere -->
<model name="green_sphere">
  <static>false</static>
  <pose>0 -2 0.5 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry>
        <sphere>
          <radius>0.5</radius>
        </sphere>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <sphere>
          <radius>0.5</radius>
        </sphere>
      </geometry>
      <material>
        <ambient>0 1 0 1</ambient>
        <diffuse>0 1 0 1</diffuse>
      </material>
    </visual>
    <inertial>
      <mass>3.0</mass>
    </inertial>
  </link>
</model>
```

Now when you launch the world, you will see a red box, blue cylinder, and green sphere. Since they are not static, you can push them around (we will add a robot to do that soon).

---

## Spawning Your URDF Robot in Gazebo

Now comes the exciting part: putting your robot from Module 1 into this virtual world.

### The ros_gz Bridge

Remember: Gazebo and ROS 2 are separate systems. To spawn a robot and control it, we need the **ros_gz bridge**. This bridge:

1. Listens to Gazebo topics and republishes them as ROS 2 topics
2. Listens to ROS 2 topics and sends commands to Gazebo
3. Provides services to spawn and delete models

```
┌─────────────────┐          ┌─────────────────┐          ┌─────────────────┐
│                 │          │                 │          │                 │
│     Gazebo      │◄────────▶│  ros_gz_bridge  │◄────────▶│     ROS 2       │
│                 │          │                 │          │                 │
│  /gz/cmd_vel    │          │   Translates    │          │   /cmd_vel      │
│  /gz/scan       │          │    messages     │          │   /scan         │
│                 │          │                 │          │                 │
└─────────────────┘          └─────────────────┘          └─────────────────┘
```

### Launch File for Robot in Gazebo

Here is a complete launch file that:
1. Starts Gazebo with your world
2. Spawns your URDF robot
3. Starts the ros_gz bridge for control

```python
# File: robot_simulation.launch.py
# Prerequisites: ros-humble-ros-gz, your URDF robot
# Place in: your_package/launch/

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    world_file = os.path.join(
        get_package_share_directory('my_robot'),
        'worlds',
        'my_world.sdf'
    )
    urdf_file = os.path.join(
        get_package_share_directory('my_robot'),
        'urdf',
        'robot.urdf'
    )

    # Read URDF
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        # Start Gazebo with the world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': world_file}.items()
        ),

        # Spawn the robot in Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'my_robot',
                '-topic', '/robot_description',
                '-x', '0', '-y', '0', '-z', '0.5'
            ],
            output='screen'
        ),

        # Publish robot description for the spawner
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),

        # Bridge: Gazebo <-> ROS 2 for velocity commands
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'
            ],
            output='screen'
        ),
    ])
```

### Understanding the Bridge Arguments

The bridge uses a special syntax to specify which topics to bridge:

```
/topic_name@ros_msg_type@gz_msg_type
```

For example:
- `/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist` bridges velocity commands
- `/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan` bridges lidar data

The `@` symbols separate the topic name, ROS 2 message type, and Gazebo message type.

---

## Controlling Your Simulated Robot

Once your robot is spawned and the bridge is running, you can control it using standard ROS 2 tools.

### Using Teleop to Drive

```bash
# In a new terminal, source ROS 2 and run teleop
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Use the keyboard to send velocity commands:
- `i`: Forward
- `,`: Backward
- `j`: Turn left
- `l`: Turn right
- `k`: Stop

Your robot should move in Gazebo just like it would in the real world!

### Viewing Topics

```bash
# List all topics (should show both ROS 2 and bridged Gazebo topics)
ros2 topic list

# Echo odometry to see robot position
ros2 topic echo /odom
```

---

## Gazebo GUI Tools

Gazebo provides powerful built-in tools for debugging your simulation.

### The Component Inspector

Right-click any model and select **Inspect** to see:
- Model pose (position and orientation)
- Link properties (mass, inertia)
- Joint states (if applicable)

### The Transform Control

Use the toolbar buttons to:
- **Translate** (arrows icon): Move objects by dragging
- **Rotate** (circular arrows): Rotate objects
- **Scale** (resize icon): Change object size

### The Play/Pause Controls

- **Play** (triangle): Run simulation
- **Pause** (two bars): Freeze simulation
- **Step** (triangle with bar): Advance one physics step

Pausing is useful when you want to carefully position objects or inspect the exact state at a moment in time.

---

## Hands-On Exercise

### Exercise 1: Build a Simple Obstacle Course

**Objective:** Create a Gazebo world with obstacles and drive your robot through it.

**Difficulty:** Beginner

**Instructions:**

1. Create a new SDF world file with:
   - A ground plane
   - At least 5 box obstacles arranged as a maze
   - Proper lighting
2. Create a launch file that loads your world and spawns a robot
3. Use teleop to drive through the obstacle course

**Starter Code:**

```xml
<?xml version="1.0" ?>
<!-- File: obstacle_course.sdf -->
<sdf version="1.8">
  <world name="obstacle_course">

    <!-- TODO: Add physics configuration -->

    <!-- TODO: Add required plugins -->

    <!-- TODO: Add lighting -->

    <!-- TODO: Add ground plane -->

    <!-- TODO: Add 5+ box obstacles to create a maze -->

  </world>
</sdf>
```

<details>
<summary>Click to reveal solution</summary>

```xml
<?xml version="1.0" ?>
<!-- File: obstacle_course.sdf (Solution) -->
<sdf version="1.8">
  <world name="obstacle_course">

    <physics name="1ms" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
          <material><ambient>0.3 0.3 0.3 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- Maze walls -->
    <model name="wall_1">
      <static>true</static>
      <pose>3 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>0.2 4 1</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>0.2 4 1</size></box></geometry>
          <material><ambient>0.6 0.3 0.1 1</ambient></material></visual>
      </link>
    </model>

    <model name="wall_2">
      <static>true</static>
      <pose>0 2 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>4 0.2 1</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>4 0.2 1</size></box></geometry>
          <material><ambient>0.6 0.3 0.1 1</ambient></material></visual>
      </link>
    </model>

    <model name="wall_3">
      <static>true</static>
      <pose>-2 -1 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>0.2 3 1</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>0.2 3 1</size></box></geometry>
          <material><ambient>0.6 0.3 0.1 1</ambient></material></visual>
      </link>
    </model>

    <model name="wall_4">
      <static>true</static>
      <pose>1 -2 0.5 0 0 1.57</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>0.2 2 1</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>0.2 2 1</size></box></geometry>
          <material><ambient>0.6 0.3 0.1 1</ambient></material></visual>
      </link>
    </model>

    <model name="wall_5">
      <static>true</static>
      <pose>-1 -3 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>3 0.2 1</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>3 0.2 1</size></box></geometry>
          <material><ambient>0.6 0.3 0.1 1</ambient></material></visual>
      </link>
    </model>

  </world>
</sdf>
```

</details>

**Verification:**
```bash
# Launch the world
gz sim obstacle_course.sdf

# Verify walls appear as a maze
# Spawn your robot and drive through using teleop
```

---

## Common Issues and Solutions

### Issue 1: Gazebo Window is Black

**Cause:** Missing plugins or no lighting.

**Solution:** Ensure your world includes the three required plugins and at least one light source.

### Issue 2: Robot Falls Through Ground

**Cause:** Robot spawned below ground level or collision geometry missing.

**Solution:** Spawn at z=0.5 or higher. Verify URDF has `<collision>` elements.

### Issue 3: Bridge Not Working

**Cause:** Topic names or message types mismatch.

**Solution:** Check exact topic names in Gazebo vs ROS 2. Use `gz topic -l` to list Gazebo topics.

```bash
# List Gazebo topics
gz topic -l

# List ROS 2 topics
ros2 topic list
```

### Issue 4: Simulation Runs Slowly

**Cause:** Physics step size too small or complex models.

**Solution:** Increase `max_step_size` (e.g., 0.002) or simplify collision geometry.

---

## Summary

### Key Takeaways

1. **Gazebo is a 3D robot simulator** that provides physics, sensors, and rendering. It lets you test robot code without real hardware.

2. **Gazebo and ROS 2 are separate systems** connected by the ros_gz bridge. The bridge translates messages between the two systems.

3. **SDF describes simulation worlds** while URDF describes robots. Gazebo can load URDF files directly.

4. **Worlds contain models** - everything from the ground plane to robots to obstacles is a model in SDF.

5. **The bridge syntax is important**: `/topic@ros_type@gz_type` specifies how to translate messages.

### Quick Reference

```bash
# Launch Gazebo with a world
gz sim my_world.sdf

# List Gazebo topics
gz topic -l

# Echo a Gazebo topic
gz topic -e /topic_name

# Spawn a model
ros2 run ros_gz_sim create -name robot -topic /robot_description
```

---

## What's Next

Now that you can launch simulations and spawn robots, the next chapter dives into **physics simulation**. You will learn how physics engines work, how to configure them for realistic behavior, and how to tune parameters for the best balance of realism and performance.

[Next: Chapter 2 - Physics Simulation →](/docs/module-2-simulation/physics-sim)

---

## Additional Resources

- [Gazebo Fortress Tutorials](https://gazebosim.org/docs/fortress/tutorials) - Official tutorials
- [SDF Specification](http://sdformat.org/spec) - Complete SDF reference
- [ros_gz Documentation](https://github.com/gazebosim/ros_gz) - ROS-Gazebo integration
- [Gazebo Models](https://app.gazebosim.org/fuel/models) - Pre-built models to use
