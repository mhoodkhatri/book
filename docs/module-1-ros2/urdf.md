---
sidebar_position: 5
title: "Chapter 4: Robot Description (URDF)"
description: "Learn URDF (Unified Robot Description Format) to describe your robot's physical structure. Create links, joints, and visualize robots in RViz."
keywords: [URDF, robot description, links, joints, kinematics, xacro, RViz, visualization, robot model, XML]
---

# Chapter 4: Robot Description (URDF)

## Chapter Overview

In the previous chapters, you learned how ROS 2 nodes communicate using topics, services, and actions. But there is something missing: **the robot itself**. How does ROS 2 know what your robot looks like? How many arms does it have? How do the joints bend? Where are the sensors mounted?

This is where **URDF** comes in. URDF stands for **Unified Robot Description Format**. It is an XML file that tells ROS 2 everything about your robot's physical body—its shape, its joints, how the parts connect, and how they can move.

Think of URDF as the robot's **blueprint** or **skeleton definition**. Just like an architect creates blueprints before building a house, roboticists create URDF files before simulating or controlling a robot. Without a URDF, tools like RViz (visualization) and Gazebo (simulation) would not know how to display or simulate your robot.

**Prerequisites**: Completed [Chapter 3: Python Client Library (rclpy)](/docs/module-1-ros2/rclpy), basic understanding of XML syntax

**What You Will Learn**: Describe robot structure for visualization and simulation

---

## Learning Objectives

After completing this chapter, you will be able to:

- Explain why robots need a description file and what URDF provides
- Create links (rigid body parts) with visual and collision geometry
- Define joints that connect links and specify their motion types
- Build a complete URDF for a simple robot arm
- Use Xacro to write cleaner, reusable robot descriptions
- Visualize your robot model in RViz

---

## Why Do Robots Need a Description File?

Imagine you are building a robot arm with three segments. You write code to move the motors, but then you want to:

1. **Visualize the robot** on your screen to see what it is doing
2. **Simulate the robot** in a virtual world before testing on real hardware
3. **Plan motions** that avoid collisions with obstacles
4. **Calculate positions** of the end-effector (the gripper or tool)

All of these tasks require knowing the robot's **geometry**—how long each segment is, where the joints are, and how they rotate. This information must be stored somewhere that all ROS 2 tools can read.

That "somewhere" is the URDF file.

### What Information Does URDF Contain?

A URDF file describes your robot using these key elements:

| Element | What It Describes | Real-World Analogy |
|---------|-------------------|-------------------|
| **Link** | A rigid body part | A bone in your arm |
| **Joint** | Connection between two links | Your elbow or shoulder |
| **Visual** | How the link looks (for display) | The skin and appearance |
| **Collision** | Simplified shape for physics | A safety boundary |
| **Inertial** | Mass and how weight is distributed | How heavy and balanced |

Without a URDF, your robot is invisible to ROS 2 tools. With a URDF, you can see it, simulate it, and plan its motions.

---

## Understanding Links: The Building Blocks

A **link** represents a single rigid body—a part of the robot that does not bend or flex. Think of links as the "bones" of your robot.

### The Human Arm Analogy

Consider your own arm:

```
┌─────────────────────────────────────────────────────────────────┐
│                     YOUR ARM AS LINKS                            │
│                                                                  │
│   ┌──────────┐      ┌──────────┐      ┌──────────┐              │
│   │  Upper   │──────│  Lower   │──────│   Hand   │              │
│   │   Arm    │      │   Arm    │      │          │              │
│   │ (Link 1) │      │ (Link 2) │      │ (Link 3) │              │
│   └──────────┘      └──────────┘      └──────────┘              │
│         ▲                 ▲                 ▲                    │
│         │                 │                 │                    │
│     Shoulder           Elbow             Wrist                  │
│      Joint             Joint             Joint                  │
│                                                                  │
│   Each section between joints is a LINK (rigid, doesn't bend)    │
│   Each connection point is a JOINT (allows movement)             │
└─────────────────────────────────────────────────────────────────┘
```

Your upper arm is one link—it is rigid between your shoulder and elbow. Your lower arm is another link—rigid between your elbow and wrist. The joints (shoulder, elbow, wrist) are what connect these links and allow movement.

### Defining a Link in URDF

Here is the simplest possible link—just a name with no shape:

```xml
<!-- A link is defined with a name -->
<link name="base_link"/>
```

But a link without a shape is invisible. Let us add a **visual** element so we can see it:

```xml
<link name="base_link">
  <visual>
    <geometry>
      <box size="0.5 0.5 0.1"/>  <!-- Width, Depth, Height in meters -->
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>  <!-- Red, Green, Blue, Alpha -->
    </material>
  </visual>
</link>
```

This creates a blue box that is 50cm wide, 50cm deep, and 10cm tall. The `rgba` values go from 0 to 1, where `0 0 1 1` means no red, no green, full blue, and fully opaque.

### Available Geometry Shapes

URDF provides four basic shapes for visual and collision geometry:

| Shape | XML Element | Parameters | Example Use |
|-------|-------------|------------|-------------|
| Box | `<box size="x y z"/>` | Width, depth, height | Robot base, body panels |
| Cylinder | `<cylinder radius="r" length="l"/>` | Radius and length | Arm segments, wheels |
| Sphere | `<sphere radius="r"/>` | Radius only | Joints, end-effectors |
| Mesh | `<mesh filename="path.stl"/>` | Path to 3D model file | Complex shapes, imported CAD |

Here is an example of each:

```xml
<!-- Box: 10cm x 10cm x 5cm -->
<geometry>
  <box size="0.1 0.1 0.05"/>
</geometry>

<!-- Cylinder: 3cm radius, 20cm long -->
<geometry>
  <cylinder radius="0.03" length="0.2"/>
</geometry>

<!-- Sphere: 5cm radius -->
<geometry>
  <sphere radius="0.05"/>
</geometry>

<!-- Mesh: Load a 3D model file -->
<geometry>
  <mesh filename="package://my_robot/meshes/gripper.stl"/>
</geometry>
```

---

## Visual vs Collision Geometry

You might wonder: why do links have both `<visual>` and `<collision>` elements? Are they not the same thing?

### The Difference Explained Simply

**Visual geometry** is what you **see** on screen. It can be detailed and pretty—complex meshes with many polygons, smooth curves, and realistic textures.

**Collision geometry** is what the **physics engine uses** to detect contacts. It should be **simple** so calculations are fast. A detailed mesh with 10,000 triangles would slow down the simulation.

```
┌─────────────────────────────────────────────────────────────────┐
│            VISUAL vs COLLISION GEOMETRY                          │
│                                                                  │
│   VISUAL (what you see):         COLLISION (for physics):        │
│                                                                  │
│      ┌──────────────┐               ┌──────────────┐            │
│      │  Detailed    │               │   Simple     │            │
│      │  robot hand  │               │    box       │            │
│      │  with all    │               │  surrounding │            │
│      │  fingers     │               │  the hand    │            │
│      └──────────────┘               └──────────────┘            │
│                                                                  │
│   Shows realistic appearance     Used for fast collision checks  │
└─────────────────────────────────────────────────────────────────┘
```

### Example: Link with Both Visual and Collision

```xml
<link name="gripper">
  <!-- What you see: detailed mesh -->
  <visual>
    <geometry>
      <mesh filename="package://my_robot/meshes/gripper_detailed.stl"/>
    </geometry>
    <material name="silver">
      <color rgba="0.8 0.8 0.8 1"/>
    </material>
  </visual>

  <!-- What physics uses: simple box -->
  <collision>
    <geometry>
      <box size="0.1 0.08 0.05"/>
    </geometry>
  </collision>
</link>
```

:::tip Best Practice
Always use simple shapes (boxes, cylinders, spheres) for collision geometry, even if your visual geometry uses detailed meshes. This keeps simulations fast and stable.
:::

---

## Understanding Joints: Connecting the Parts

If links are the bones, **joints** are the connections that allow movement. A joint connects exactly two links: a **parent** and a **child**.

### How Joints Work

When a joint moves, it moves the **child link** relative to the **parent link**. The parent stays fixed (from the joint's perspective), and the child moves.

```
┌─────────────────────────────────────────────────────────────────┐
│                    JOINT RELATIONSHIP                            │
│                                                                  │
│    ┌────────────┐                    ┌────────────┐             │
│    │   PARENT   │◄── stays fixed ───│   JOINT    │             │
│    │   LINK     │                    │            │             │
│    └────────────┘                    └─────┬──────┘             │
│                                            │                     │
│                                            │ moves               │
│                                            ▼                     │
│                                      ┌────────────┐             │
│                                      │   CHILD    │             │
│                                      │   LINK     │             │
│                                      └────────────┘             │
│                                                                  │
│   When the joint rotates, only the child link moves              │
└─────────────────────────────────────────────────────────────────┘
```

### Joint Types

Different joints allow different types of motion. URDF supports these joint types:

| Joint Type | Motion | Degrees of Freedom | Real-World Example |
|------------|--------|-------------------|-------------------|
| **fixed** | None (rigid connection) | 0 | Sensor mounted on base |
| **revolute** | Rotation with limits | 1 | Elbow (bends 0° to 150°) |
| **continuous** | Rotation without limits | 1 | Wheel (spins forever) |
| **prismatic** | Linear sliding | 1 | Telescope, drawer |
| **floating** | All movement | 6 | Drone in air |
| **planar** | Sliding on a plane | 2 | Object on a table |

For most robot arms and humanoids, you will use **revolute** (limited rotation) and **continuous** (unlimited rotation) joints.

### Defining a Joint in URDF

Here is a revolute joint that connects a base to an arm:

```xml
<joint name="base_to_arm" type="revolute">
  <!-- Which links this joint connects -->
  <parent link="base_link"/>
  <child link="arm_link"/>

  <!-- Where the joint is located (relative to parent) -->
  <origin xyz="0 0 0.1" rpy="0 0 0"/>

  <!-- The axis of rotation -->
  <axis xyz="0 0 1"/>  <!-- Rotate around Z axis -->

  <!-- Rotation limits (in radians) -->
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
</joint>
```

Let me explain each part in simple terms:

**`<parent>` and `<child>`**: These tell URDF which two links this joint connects. The arm_link will move when this joint rotates; base_link stays still.

**`<origin xyz="0 0 0.1">`**: This is where the joint is located relative to the parent link. Here, the joint is 10cm (0.1m) above the parent's center.

**`<axis xyz="0 0 1">`**: This defines which direction the joint rotates around. `0 0 1` means the Z-axis (up/down), so the arm swings horizontally like a door.

**`<limit>`**: For revolute joints, this sets how far the joint can rotate. `-1.57` to `1.57` radians is about -90° to +90°.

### Visualizing Joint Axes

Understanding which axis does what is important:

```
            Z (up)
            │
            │
            │
            └───────── Y (left)
           /
          /
         X (forward)

Rotation around X: Roll (like a rolling log)
Rotation around Y: Pitch (like nodding your head)
Rotation around Z: Yaw (like shaking your head "no")
```

---

## Building Your First Robot: A Simple Arm

Now let us put everything together and build a complete robot arm with two segments. This will give you a working example you can visualize in RViz.

### The Robot We Are Building

```
┌─────────────────────────────────────────────────────────────────┐
│                    TWO-LINK ROBOT ARM                            │
│                                                                  │
│                              ┌─────────┐                         │
│                              │  Link 2 │                         │
│                              │ (upper  │                         │
│                              │  arm)   │                         │
│                              └────┬────┘                         │
│                                   │ Joint 2 (shoulder)           │
│                              ┌────┴────┐                         │
│                              │  Link 1 │                         │
│                              │ (base   │                         │
│                              │  post)  │                         │
│                              └────┬────┘                         │
│                                   │ Joint 1 (fixed to world)     │
│                              ┌────┴────┐                         │
│     ══════════════════════════ Base ══════════════════════════  │
│                              └─────────┘                         │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### Complete URDF File

```xml
<?xml version="1.0"?>
<!-- File: simple_arm.urdf -->
<!-- A two-link robot arm for learning URDF -->
<!-- Tested with: ROS 2 Humble, RViz2 -->

<robot name="simple_arm">

  <!-- ==================== LINK DEFINITIONS ==================== -->

  <!-- Base: The fixed foundation of our robot -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.3 0.05"/>  <!-- 30cm x 30cm x 5cm platform -->
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.3 0.05"/>
      </geometry>
    </collision>
  </link>

  <!-- Link 1: Vertical post rising from the base -->
  <link name="link_1">
    <visual>
      <origin xyz="0 0 0.15"/>  <!-- Shift visual up so bottom is at joint -->
      <geometry>
        <cylinder radius="0.04" length="0.3"/>  <!-- 4cm radius, 30cm tall -->
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.2 0.8 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.15"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
    </collision>
  </link>

  <!-- Link 2: Horizontal arm segment -->
  <link name="link_2">
    <visual>
      <origin xyz="0.15 0 0"/>  <!-- Shift visual so one end is at joint -->
      <geometry>
        <box size="0.3 0.05 0.05"/>  <!-- 30cm long, 5cm x 5cm cross-section -->
      </geometry>
      <material name="orange">
        <color rgba="0.9 0.5 0.1 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0.15 0 0"/>
      <geometry>
        <box size="0.3 0.05 0.05"/>
      </geometry>
    </collision>
  </link>

  <!-- ==================== JOINT DEFINITIONS ==================== -->

  <!-- Joint 1: Connects base to link_1, allows rotation around Z -->
  <joint name="joint_1" type="revolute">
    <parent link="base_link"/>
    <child link="link_1"/>
    <origin xyz="0 0 0.025"/>  <!-- Just above the base surface -->
    <axis xyz="0 0 1"/>        <!-- Rotate around vertical axis -->
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
  </joint>

  <!-- Joint 2: Connects link_1 to link_2, allows rotation around Y -->
  <joint name="joint_2" type="revolute">
    <parent link="link_1"/>
    <child link="link_2"/>
    <origin xyz="0 0 0.3"/>    <!-- At top of link_1 -->
    <axis xyz="0 1 0"/>        <!-- Rotate around Y axis (pitch) -->
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>

</robot>
```

### Understanding the Visual Origin

Notice the `<origin>` tags inside the `<visual>` elements. This is a common source of confusion, so let me explain it simply.

When you create a cylinder or box, the shape is centered at `(0, 0, 0)` by default. But we often want the joint to be at one end of the link, not in the middle.

```
┌─────────────────────────────────────────────────────────────────┐
│              WITHOUT ORIGIN SHIFT vs WITH ORIGIN SHIFT           │
│                                                                  │
│   WITHOUT:                        WITH:                          │
│   Joint at center                 Joint at bottom                │
│                                                                  │
│        ┌───┐                           ┌───┐                     │
│        │   │                           │   │                     │
│    ──● │   │                           │   │                     │
│  joint │   │                           │   │                     │
│        └───┘                       ──● └───┘                     │
│                                  joint                           │
│                                                                  │
│   Cylinder rotates around         Cylinder rotates around        │
│   its center (weird!)             its base (natural!)            │
└─────────────────────────────────────────────────────────────────┘
```

By adding `<origin xyz="0 0 0.15"/>` to a 30cm cylinder, we shift the visual geometry up by 15cm, putting the joint at the bottom where it belongs.

---

## Visualizing Your Robot in RViz

Once you have a URDF file, you want to see your robot. RViz is the visualization tool for ROS 2.

### Step 1: Create a Launch File

Launch files start multiple nodes and load parameters. Here is a launch file that displays your robot:

```python
# File: display_robot.launch.py
# Prerequisites: ROS 2 Humble, robot_state_publisher, rviz2
# Place in: your_package/launch/

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to your URDF file
    urdf_file = os.path.join(
        get_package_share_directory('my_robot'),
        'urdf',
        'simple_arm.urdf'
    )

    # Read the URDF file
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        # Publish robot description to /robot_description topic
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),

        # GUI to manually move joints
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),

        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory('my_robot'),
                'rviz',
                'display.rviz'
            )]
        )
    ])
```

### Step 2: Run the Visualization

```bash
# Install required packages if needed
sudo apt install ros-humble-joint-state-publisher-gui ros-humble-robot-state-publisher

# Launch the visualization
ros2 launch my_robot display_robot.launch.py
```

### Step 3: Configure RViz

In RViz, you need to add a **RobotModel** display:

1. Click **Add** in the Displays panel
2. Select **RobotModel**
3. Set **Fixed Frame** to `base_link`
4. Your robot should appear!

Use the **Joint State Publisher GUI** window to drag sliders and watch your robot move.

---

## Introduction to Xacro: Cleaner Robot Descriptions

As robots get more complex, URDF files become repetitive and hard to maintain. **Xacro** (XML Macros) solves this problem by adding:

- **Variables** (properties) to avoid repeating numbers
- **Macros** to avoid repeating blocks of XML
- **Math expressions** to calculate values
- **Conditional logic** to include/exclude parts

Think of Xacro as a **template language** that generates URDF. You write a `.xacro` file, and it gets processed into a regular `.urdf` file.

### Why Use Xacro?

Consider a robot with two identical arms. In pure URDF, you would copy-paste all the arm links and joints twice, changing only the names. If you later want to change the arm length, you must edit both copies.

With Xacro, you define the arm once as a macro, then call it twice with different names. Change the arm length in one place, and both arms update.

### Basic Xacro Syntax

**Defining a property (variable):**
```xml
<!-- Define a value once, use it many times -->
<xacro:property name="arm_length" value="0.5"/>
<xacro:property name="arm_radius" value="0.03"/>

<!-- Use the property with ${} -->
<cylinder radius="${arm_radius}" length="${arm_length}"/>
```

**Using math:**
```xml
<!-- Calculate values -->
<xacro:property name="arm_length" value="0.5"/>

<!-- Half the arm length for the visual origin -->
<origin xyz="0 0 ${arm_length/2}"/>
```

**Defining a macro (reusable block):**
```xml
<!-- Define a macro for a wheel -->
<xacro:macro name="wheel" params="name x_pos">
  <link name="${name}_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </visual>
  </link>
  <joint name="${name}_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="${name}_wheel"/>
    <origin xyz="${x_pos} 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
</xacro:macro>

<!-- Use the macro twice -->
<xacro:wheel name="left" x_pos="-0.2"/>
<xacro:wheel name="right" x_pos="0.2"/>
```

### Complete Xacro Example

Here is our simple arm rewritten using Xacro:

```xml
<?xml version="1.0"?>
<!-- File: simple_arm.urdf.xacro -->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="simple_arm">

  <!-- ==================== PROPERTIES ==================== -->
  <xacro:property name="base_size" value="0.3"/>
  <xacro:property name="base_height" value="0.05"/>
  <xacro:property name="link1_radius" value="0.04"/>
  <xacro:property name="link1_length" value="0.3"/>
  <xacro:property name="link2_length" value="0.3"/>
  <xacro:property name="link2_width" value="0.05"/>

  <!-- ==================== MACROS ==================== -->

  <!-- Macro for colored material -->
  <xacro:macro name="material_color" params="name r g b">
    <material name="${name}">
      <color rgba="${r} ${g} ${b} 1"/>
    </material>
  </xacro:macro>

  <!-- ==================== ROBOT DEFINITION ==================== -->

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="${base_size} ${base_size} ${base_height}"/>
      </geometry>
      <xacro:material_color name="gray" r="0.5" g="0.5" b="0.5"/>
    </visual>
    <collision>
      <geometry>
        <box size="${base_size} ${base_size} ${base_height}"/>
      </geometry>
    </collision>
  </link>

  <!-- Link 1 -->
  <link name="link_1">
    <visual>
      <origin xyz="0 0 ${link1_length/2}"/>
      <geometry>
        <cylinder radius="${link1_radius}" length="${link1_length}"/>
      </geometry>
      <xacro:material_color name="blue" r="0.2" g="0.2" b="0.8"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${link1_length/2}"/>
      <geometry>
        <cylinder radius="${link1_radius}" length="${link1_length}"/>
      </geometry>
    </collision>
  </link>

  <!-- Link 2 -->
  <link name="link_2">
    <visual>
      <origin xyz="${link2_length/2} 0 0"/>
      <geometry>
        <box size="${link2_length} ${link2_width} ${link2_width}"/>
      </geometry>
      <xacro:material_color name="orange" r="0.9" g="0.5" b="0.1"/>
    </visual>
    <collision>
      <origin xyz="${link2_length/2} 0 0"/>
      <geometry>
        <box size="${link2_length} ${link2_width} ${link2_width}"/>
      </geometry>
    </collision>
  </link>

  <!-- Joint 1 -->
  <joint name="joint_1" type="revolute">
    <parent link="base_link"/>
    <child link="link_1"/>
    <origin xyz="0 0 ${base_height/2}"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
  </joint>

  <!-- Joint 2 -->
  <joint name="joint_2" type="revolute">
    <parent link="link_1"/>
    <child link="link_2"/>
    <origin xyz="0 0 ${link1_length}"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>

</robot>
```

### Processing Xacro to URDF

To convert a `.xacro` file to `.urdf`:

```bash
# Convert xacro to urdf
xacro simple_arm.urdf.xacro > simple_arm.urdf

# Or process directly in a launch file using Command substitution
robot_description = Command(['xacro ', xacro_file_path])
```

---

## Adding Inertial Properties for Simulation

For visualization in RViz, visual and collision geometry are enough. But for **physics simulation** in Gazebo, you also need **inertial properties**: mass and how that mass is distributed.

### What Are Inertial Properties?

When you push an object, how it moves depends on:
- **Mass**: How heavy it is (in kilograms)
- **Inertia**: How the mass is distributed (affects rotation)

A heavy barbell is hard to spin. A figure skater spins faster when they pull their arms in. This is because of **moment of inertia**—where the mass is located relative to the rotation axis.

### Adding Inertial to a Link

```xml
<link name="link_1">
  <visual>
    <!-- visual geometry here -->
  </visual>

  <collision>
    <!-- collision geometry here -->
  </collision>

  <inertial>
    <mass value="1.0"/>  <!-- 1 kilogram -->
    <origin xyz="0 0 0.15"/>  <!-- Center of mass location -->
    <inertia
      ixx="0.01" ixy="0" ixz="0"
      iyy="0.01" iyz="0"
      izz="0.005"/>
  </inertial>
</link>
```

The inertia matrix describes how the mass is spread out. For simple shapes, you can calculate or look up these values:

| Shape | Ixx, Iyy (around length) | Izz (around axis) |
|-------|-------------------------|-------------------|
| Cylinder (radius r, length l) | (1/12) * m * (3r² + l²) | (1/2) * m * r² |
| Box (w × d × h) | (1/12) * m * (d² + h²) | (1/12) * m * (w² + d²) |
| Sphere (radius r) | (2/5) * m * r² | (2/5) * m * r² |

:::tip For Learning
When first learning URDF, focus on visual and collision geometry. Add inertial properties when you are ready to simulate in Gazebo. RViz visualization does not require inertial data.
:::

---

## Common URDF Mistakes and How to Fix Them

When working with URDF, you will encounter errors. Here are the most common ones:

### Mistake 1: Disconnected Links

Every link except one must have a parent joint. If a link is "floating" with no joint, URDF tools will fail.

```xml
<!-- WRONG: link_2 has no joint connecting it -->
<link name="base_link"/>
<link name="link_2"/>  <!-- Orphan! -->

<!-- CORRECT: All links connected via joints -->
<link name="base_link"/>
<link name="link_2"/>
<joint name="joint_1" type="fixed">
  <parent link="base_link"/>
  <child link="link_2"/>
</joint>
```

### Mistake 2: Wrong Axis Direction

The axis is often set incorrectly, causing unexpected rotation directions.

```xml
<!-- Robot rotates around wrong axis? Check your axis definition -->
<axis xyz="0 0 1"/>  <!-- Z-axis: horizontal rotation (yaw) -->
<axis xyz="0 1 0"/>  <!-- Y-axis: pitch rotation (nodding) -->
<axis xyz="1 0 0"/>  <!-- X-axis: roll rotation -->
```

### Mistake 3: Units Confusion

URDF uses **meters** for length and **radians** for angles. Mixing units causes robots to be tiny or huge.

```xml
<!-- WRONG: Thinking in centimeters -->
<box size="30 30 5"/>  <!-- This is 30 METERS, not centimeters! -->

<!-- CORRECT: Always use meters -->
<box size="0.3 0.3 0.05"/>  <!-- 30cm = 0.3 meters -->
```

### Mistake 4: Missing Fixed Frame in RViz

If your robot does not appear in RViz, check that the **Fixed Frame** is set to a link that exists in your URDF (usually `base_link`).

---

## Hands-On Exercise

### Exercise 1: Build a Mobile Robot Base

**Objective:** Create a URDF for a simple mobile robot with a rectangular base and two wheels.

**Difficulty:** Intermediate

**Instructions:**

1. Create a rectangular base (40cm × 30cm × 10cm)
2. Add two cylindrical wheels (radius 5cm, width 3cm)
3. Use `continuous` joints for the wheels (they spin freely)
4. Position wheels on the left and right sides of the base
5. Visualize in RViz and test that wheels rotate

**Starter Code:**

```xml
<?xml version="1.0"?>
<!-- File: mobile_base.urdf -->
<robot name="mobile_base">

  <!-- TODO: Define base_link (rectangular box) -->

  <!-- TODO: Define left_wheel link (cylinder) -->

  <!-- TODO: Define right_wheel link (cylinder) -->

  <!-- TODO: Define left_wheel_joint (continuous) -->

  <!-- TODO: Define right_wheel_joint (continuous) -->

</robot>
```

<details>
<summary>Click to reveal solution</summary>

```xml
<?xml version="1.0"?>
<!-- File: mobile_base.urdf (Solution) -->
<robot name="mobile_base">

  <!-- Base: Rectangular body of the robot -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.4 0.3 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.2 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.4 0.3 0.1"/>
      </geometry>
    </collision>
  </link>

  <!-- Left Wheel -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.03"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.03"/>
      </geometry>
    </collision>
  </link>

  <!-- Right Wheel -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.03"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.03"/>
      </geometry>
    </collision>
  </link>

  <!-- Left Wheel Joint -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.165 -0.025" rpy="1.5708 0 0"/>  <!-- 90° roll to orient wheel -->
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Right Wheel Joint -->
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.165 -0.025" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

</robot>
```

</details>

**Verification:**
```bash
# Check URDF for errors
check_urdf mobile_base.urdf

# Visualize (requires launch file setup)
ros2 launch my_robot display_robot.launch.py
```

---

## Summary

### Key Takeaways

1. **URDF is the robot's blueprint** - It describes the physical structure that visualization and simulation tools need. Without a URDF, ROS 2 tools cannot see or simulate your robot.

2. **Links are rigid body parts** - Each link has visual geometry (what you see), collision geometry (for physics), and optionally inertial properties (mass distribution).

3. **Joints connect links and define motion** - Use `revolute` for limited rotation, `continuous` for unlimited rotation, `prismatic` for sliding, and `fixed` for rigid connections.

4. **Origin placement matters** - The `<origin>` inside visual/collision shifts the geometry so joints are at the right location (usually at the end of a link, not the center).

5. **Xacro makes URDF maintainable** - Use properties for repeated values and macros for repeated structures. This keeps your robot descriptions clean and easy to modify.

### URDF Element Quick Reference

```xml
<!-- Link structure -->
<link name="my_link">
  <visual>
    <origin xyz="x y z" rpy="roll pitch yaw"/>
    <geometry> ... </geometry>
    <material name="color"> ... </material>
  </visual>
  <collision>
    <geometry> ... </geometry>
  </collision>
  <inertial>
    <mass value="kg"/>
    <inertia ixx="" iyy="" izz="" .../>
  </inertial>
</link>

<!-- Joint structure -->
<joint name="my_joint" type="revolute|continuous|fixed|prismatic">
  <parent link="parent_name"/>
  <child link="child_name"/>
  <origin xyz="x y z" rpy="roll pitch yaw"/>
  <axis xyz="x y z"/>
  <limit lower="" upper="" effort="" velocity=""/>
</joint>
```

---

## Module 1 Complete!

Congratulations! You have completed Module 1: The Robotic Nervous System. You now understand:

- **Nodes and Topics**: How robot components communicate via publish-subscribe
- **Services and Actions**: Request-response and long-running task patterns
- **rclpy**: Writing production-quality Python nodes
- **URDF**: Describing your robot's physical structure

These fundamentals are the foundation for everything that follows. In Module 2, you will bring your URDF robots to life in simulated worlds using Gazebo and Unity.

---

## What's Next

Continue to **Module 2: The Digital Twin** where you will:

- Place your URDF robots in physics-simulated environments
- Add sensors (cameras, lidar, IMU) to your robot models
- Interact with simulated objects using ROS 2 topics and services
- Build virtual testing environments for your robot code

[Next: Module 2 - The Digital Twin →](/docs/module-2-simulation)

---

## Additional Resources

- [URDF Specification](http://wiki.ros.org/urdf/XML) - Complete XML reference
- [URDF Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html) - Official ROS 2 tutorials
- [Xacro Documentation](http://wiki.ros.org/xacro) - Macro language reference
- [RViz User Guide](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide-Main.html) - Visualization tool documentation
