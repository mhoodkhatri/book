---
sidebar_position: 2
title: "Chapter 1: Isaac Sim"
description: "Master NVIDIA Isaac Sim for photorealistic robot simulation. Create advanced environments, generate synthetic training data, and bridge to ROS 2."
keywords: [Isaac Sim, NVIDIA, Omniverse, simulation, synthetic data, robotics, USD, domain randomization, ROS 2]
---

# Chapter 1: Isaac Sim

## Chapter Overview

In Module 2, you learned to simulate robots in Gazebo. Gazebo is excellent for testing control algorithms and basic sensor simulation. But what happens when you need to train a neural network to recognize objects? Or when you need your simulated camera images to look like real camera images?

This is where **NVIDIA Isaac Sim** shines. Built on the Omniverse platform, Isaac Sim uses **ray tracing** and **physically-based rendering** to create images that are nearly indistinguishable from photographs. This matters because AI models trained on realistic images transfer better to the real world.

Think of it this way: Gazebo is like a sketch of reality—useful for understanding structure. Isaac Sim is like a photograph—useful for training vision systems.

In this chapter, you will learn to install Isaac Sim, create photorealistic environments, import your robots, and connect everything to ROS 2.

**Prerequisites**: Completed [Module 2: The Digital Twin](/docs/module-2-simulation) (especially Gazebo basics and sensors)

**What You Will Learn**: Install Isaac Sim, create environments, generate synthetic data, bridge to ROS 2

---

## Learning Objectives

After completing this chapter, you will be able to:

- Explain the difference between Isaac Sim and Gazebo
- Install Isaac Sim via Omniverse Launcher
- Navigate the Isaac Sim interface and understand the USD scene format
- Create simulation environments using built-in assets
- Import URDF robots into Isaac Sim
- Configure the ROS 2 bridge for communication
- Generate synthetic training data with domain randomization

---

## Isaac Sim vs Gazebo: When to Use What

Before diving in, let us understand when to use each simulator:

| Feature | Gazebo | Isaac Sim |
|---------|--------|-----------|
| **Rendering** | OpenGL (basic) | RTX ray tracing (photorealistic) |
| **Physics** | DART, ODE, Bullet | PhysX 5 (GPU-accelerated) |
| **Use Case** | Algorithm testing, control | AI training, perception testing |
| **Hardware** | Any GPU | NVIDIA RTX GPU required |
| **Learning Curve** | Moderate | Steeper |
| **ROS 2 Integration** | Native | Bridge required |
| **Scene Format** | SDF/URDF | USD (Universal Scene Description) |

**When to use Gazebo**:
- Testing control algorithms quickly
- Working with limited hardware
- Simpler environments without visual AI

**When to use Isaac Sim**:
- Training computer vision models
- Testing perception pipelines
- Generating synthetic training data
- High-fidelity sensor simulation (especially cameras)

In practice, many teams use both: Gazebo for rapid iteration and Isaac Sim for final validation and data generation.

---

## Understanding the Omniverse Platform

Isaac Sim is built on **NVIDIA Omniverse**, a platform for 3D simulation and collaboration. Understanding Omniverse helps you work effectively with Isaac Sim.

### The Omniverse Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         OMNIVERSE PLATFORM                                   │
│                                                                              │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                         NUCLEUS SERVER                               │    │
│  │                    (Asset storage & versioning)                      │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                    │                                         │
│                                    │  USD Assets                             │
│                                    ▼                                         │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │   Isaac     │  │    Code     │  │  Create     │  │   Other     │        │
│  │    Sim      │  │   (IDE)     │  │  (Modeling) │  │   Apps      │        │
│  │             │  │             │  │             │  │             │        │
│  │  Robotics   │  │  Scripting  │  │  3D Design  │  │  Extensions │        │
│  │  Simulation │  │  Extension  │  │  Tools      │  │             │        │
│  └─────────────┘  └─────────────┘  └─────────────┘  └─────────────┘        │
│         │                                                                    │
│         │  RTX Rendering + PhysX                                            │
│         ▼                                                                    │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                    SIMULATION OUTPUT                                 │    │
│  │                                                                      │    │
│  │   • Photorealistic images    • Depth maps    • Segmentation masks   │    │
│  │   • Physics simulation       • ROS 2 data    • Sensor outputs       │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

**Key Components**:

**Nucleus Server**: A database that stores 3D assets in USD format. Think of it as "Git for 3D content." Multiple users can collaborate on the same scene.

**USD (Universal Scene Description)**: The file format for 3D scenes, developed by Pixar. More powerful than SDF—supports references, variants, and composition.

**RTX Rendering**: Uses NVIDIA's ray tracing hardware for realistic lighting, reflections, and shadows.

**PhysX 5**: GPU-accelerated physics engine. Faster than CPU-based physics and supports more complex interactions.

---

## Installing Isaac Sim

Isaac Sim installation involves three steps: Omniverse Launcher, Isaac Sim app, and verification.

### Step 1: Download Omniverse Launcher

```bash
# Go to NVIDIA's website and download the Omniverse Launcher
# https://www.nvidia.com/en-us/omniverse/download/

# After downloading the .AppImage file:
chmod +x omniverse-launcher-linux.AppImage
./omniverse-launcher-linux.AppImage
```

The Omniverse Launcher is like an app store for NVIDIA's simulation tools.

### Step 2: Install Isaac Sim

1. Open Omniverse Launcher
2. Go to the **Exchange** tab
3. Search for "Isaac Sim"
4. Click **Install** (this downloads approximately 12GB)
5. Wait for installation to complete

### Step 3: Launch and Verify

```bash
# Isaac Sim can also be launched from terminal
# Navigate to the Isaac Sim installation directory
cd ~/.local/share/ov/pkg/isaac_sim-2023.1.1

# Launch Isaac Sim
./isaac-sim.sh
```

On first launch, Isaac Sim will:
- Compile shaders (takes several minutes)
- Download additional assets
- Set up the local Nucleus cache

**Expected Result**: After a few minutes, the Isaac Sim interface should appear with a default empty stage.

### Verifying GPU Support

In Isaac Sim, check that your GPU is being used:

1. Click **Window > Extensions**
2. Search for "RTX"
3. Verify "omni.rtx.window.settings" is enabled

If your GPU is not detected, check your NVIDIA driver installation:

```bash
# Outside Isaac Sim, verify driver
nvidia-smi

# Check that the RTX GPU is listed
# Should show your RTX card with driver 525+
```

---

## The Isaac Sim Interface

Isaac Sim's interface has several key areas:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│  Menu Bar                                                                    │
├──────────────────────┬─────────────────────────────────┬───────────────────┤
│                      │                                  │                   │
│     Stage Panel      │        Viewport                  │    Property      │
│     (Scene tree)     │        (3D View)                 │    Panel         │
│                      │                                  │                   │
│                      │                                  │                   │
│                      │                                  │                   │
│                      │                                  │                   │
├──────────────────────┴─────────────────────────────────┴───────────────────┤
│                           Content Browser                                    │
│                        (Assets, Scripts, Files)                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

**Stage Panel** (left): Shows all objects in the scene as a hierarchy. Similar to Gazebo's model tree but more detailed.

**Viewport** (center): The 3D view of your simulation. Supports multiple viewports and camera views.

**Property Panel** (right): Shows properties of the selected object. Edit transforms, physics, materials here.

**Content Browser** (bottom): Browse assets from Nucleus, local files, or NVIDIA's asset library.

### Navigation Controls

- **Alt + Left Mouse**: Orbit around focal point
- **Alt + Middle Mouse**: Pan the view
- **Alt + Right Mouse**: Zoom in/out
- **F**: Focus on selected object
- **W/E/R**: Translate/Rotate/Scale tools

---

## Understanding USD: The Scene Format

Unlike Gazebo's SDF, Isaac Sim uses **USD (Universal Scene Description)**. USD is more powerful but also more complex.

### USD File Structure

A USD file describes a scene as a hierarchy of "prims" (primitives):

```
/World                          # Root prim
├── /World/Environment          # Environment settings
│   ├── /World/Environment/Sky  # Lighting
│   └── /World/Environment/Ground
├── /World/Robot                # Your robot
│   ├── /World/Robot/base_link
│   └── /World/Robot/wheel_left
└── /World/Props                # Objects in scene
    ├── /World/Props/Box_01
    └── /World/Props/Table
```

### Creating a Simple USD Scene via Python

Isaac Sim supports Python scripting. Here is how to create a simple scene programmatically:

```python
# File: create_simple_scene.py
# Run this in Isaac Sim's Script Editor (Window > Script Editor)

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid, GroundPlane
from omni.isaac.core.utils.stage import add_reference_to_stage
import numpy as np

# Create the simulation world
world = World(stage_units_in_meters=1.0)

# Add a ground plane
world.scene.add_default_ground_plane()

# Add a red cube that can move (dynamic)
cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/Cube",
        name="red_cube",
        position=np.array([0.0, 0.0, 0.5]),  # 0.5m above ground
        size=np.array([0.5, 0.5, 0.5]),      # 0.5m cube
        color=np.array([1.0, 0.0, 0.0])      # Red color
    )
)

# Reset the world to initialize
world.reset()

print("Scene created successfully!")
```

To run this script:
1. Open Isaac Sim
2. Go to **Window > Script Editor**
3. Paste the code
4. Click **Run**

You should see a red cube appear above the ground plane.

### Loading Assets from Nucleus

Isaac Sim provides a library of pre-built assets through Nucleus:

```python
# File: load_warehouse.py
# Load a pre-built warehouse environment

from omni.isaac.core.utils.stage import add_reference_to_stage

# Path to warehouse asset on Nucleus
warehouse_usd = "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/Environments/Simple_Warehouse/warehouse.usd"

# Add to current stage
add_reference_to_stage(usd_path=warehouse_usd, prim_path="/World/Warehouse")

print("Warehouse loaded!")
```

The `omniverse://localhost/` path refers to your local Nucleus server. NVIDIA provides many free assets including:
- Warehouse and factory environments
- Robot models (manipulators, mobile robots)
- Common objects (boxes, pallets, shelves)

---

## Importing Your URDF Robot

You already have URDF robots from Module 1. Let us import one into Isaac Sim.

### Using the URDF Importer

Isaac Sim includes a URDF importer that converts your robot description:

```python
# File: import_urdf_robot.py
# Import a URDF robot into Isaac Sim

from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.urdf")

from omni.isaac.urdf import _urdf
from omni.isaac.core import World
import omni.kit.commands

# Path to your URDF file
urdf_path = "/home/user/ros2_ws/src/my_robot/urdf/robot.urdf"

# URDF import configuration
import_config = _urdf.ImportConfig()
import_config.merge_fixed_joints = False  # Keep all joints
import_config.fix_base = False            # Robot can move
import_config.import_inertia_tensor = True
import_config.distance_scale = 1.0        # 1 URDF unit = 1 meter
import_config.density = 0.0               # Use URDF-specified mass
import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_VELOCITY

# Import the URDF
result, prim_path = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path=urdf_path,
    import_config=import_config,
    dest_path="/World/Robot"
)

if result:
    print(f"Robot imported successfully at {prim_path}")
else:
    print("URDF import failed!")
```

### Alternative: GUI-Based Import

1. Go to **Isaac Utils > Workflows > URDF Importer**
2. Browse to your URDF file
3. Configure import settings:
   - **Fix Base**: Uncheck for mobile robots
   - **Self Collision**: Enable if needed
   - **Joint Drive Type**: Velocity for wheels, Position for arms
4. Click **Import**

### Verifying the Import

After import, check your robot in the Stage panel:

```
/World
└── /World/Robot
    ├── /World/Robot/base_link
    ├── /World/Robot/wheel_left_link
    ├── /World/Robot/wheel_right_link
    ├── /World/Robot/lidar_link
    └── /World/Robot/camera_link
```

Each link should have:
- **Visual** geometry (for rendering)
- **Collision** geometry (for physics)
- **RigidBody** component (for dynamics)
- **Joint** connecting to parent

---

## Configuring Sensors in Isaac Sim

Isaac Sim provides realistic sensor simulation. Let us add sensors to your robot.

### Adding a Camera

```python
# File: add_camera_sensor.py
# Add a camera to the robot

from omni.isaac.sensor import Camera
import numpy as np

# Create camera at the robot's camera_link position
camera = Camera(
    prim_path="/World/Robot/camera_link/Camera",
    position=np.array([0.0, 0.0, 0.0]),  # Relative to camera_link
    frequency=30,                          # 30 FPS
    resolution=(640, 480),                 # VGA resolution
    orientation=np.array([1.0, 0.0, 0.0, 0.0])  # Quaternion (w, x, y, z)
)

# Initialize the camera
camera.initialize()

# Get camera data (after simulation step)
# rgb_data = camera.get_rgba()
# depth_data = camera.get_depth()

print("Camera added successfully!")
```

### Adding a Lidar Sensor

```python
# File: add_lidar_sensor.py
# Add a 2D lidar to the robot

from omni.isaac.range_sensor import _range_sensor
import omni.kit.commands

# Create lidar sensor
result, lidar = omni.kit.commands.execute(
    "RangeSensorCreateLidar",
    path="/World/Robot/lidar_link/Lidar",
    parent="/World/Robot/lidar_link",
    min_range=0.1,           # Minimum range in meters
    max_range=10.0,          # Maximum range in meters
    draw_points=True,        # Visualize points
    draw_lines=False,
    horizontal_fov=360.0,    # Full rotation
    vertical_fov=30.0,       # Vertical field of view
    horizontal_resolution=0.4,  # Angular resolution in degrees
    vertical_resolution=4.0,
    rotation_rate=10.0,      # Rotations per second
    high_lod=True,
    yaw_offset=0.0,
    enable_semantics=False
)

print(f"Lidar created: {lidar}")
```

### Sensor Data Output

Isaac Sim sensors output data that matches real sensor formats:

| Sensor | Output | ROS 2 Message Type |
|--------|--------|-------------------|
| Camera (RGB) | Color image | `sensor_msgs/Image` |
| Camera (Depth) | Depth image | `sensor_msgs/Image` |
| Lidar | Point cloud | `sensor_msgs/LaserScan` or `PointCloud2` |
| IMU | Acceleration, angular velocity | `sensor_msgs/Imu` |

---

## The ROS 2 Bridge

To use your Isaac Sim simulation with ROS 2, you need the **ROS 2 Bridge**. This extension publishes Isaac Sim data to ROS 2 topics.

### Enabling the ROS 2 Bridge

```python
# File: enable_ros2_bridge.py
# Enable ROS 2 Bridge extension

from omni.isaac.core.utils.extensions import enable_extension

# Enable the ROS 2 Bridge
enable_extension("omni.isaac.ros2_bridge")

print("ROS 2 Bridge enabled!")
```

Or via the GUI:
1. **Window > Extensions**
2. Search for "ros2_bridge"
3. Enable "omni.isaac.ros2_bridge"

### Creating ROS 2 Publishers

After enabling the bridge, add publishers for your sensors:

```python
# File: setup_ros2_publishers.py
# Set up ROS 2 publishers for robot sensors

import omni.graph.core as og

# Create an OmniGraph for ROS 2 communication
keys = og.Controller.Keys
(graph_handle, nodes, _, _) = og.Controller.edit(
    {"graph_path": "/World/Robot/ROS2_Graph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("CameraHelper", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
            ("LidarHelper", "omni.isaac.ros2_bridge.ROS2PublishLaserScan"),
            ("OdometryPublisher", "omni.isaac.ros2_bridge.ROS2PublishOdometry"),
            ("TFPublisher", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
            ("ClockPublisher", "omni.isaac.ros2_bridge.ROS2PublishClock"),
        ],
        keys.SET_VALUES: [
            # Camera settings
            ("CameraHelper.inputs:topicName", "/camera/image_raw"),
            ("CameraHelper.inputs:frameId", "camera_link"),
            ("CameraHelper.inputs:type", "rgb"),

            # Lidar settings
            ("LidarHelper.inputs:topicName", "/scan"),
            ("LidarHelper.inputs:frameId", "lidar_link"),

            # Odometry settings
            ("OdometryPublisher.inputs:topicName", "/odom"),
            ("OdometryPublisher.inputs:chassisFrameId", "base_link"),
            ("OdometryPublisher.inputs:odomFrameId", "odom"),

            # TF settings
            ("TFPublisher.inputs:topicName", "/tf"),

            # Clock settings
            ("ClockPublisher.inputs:topicName", "/clock"),
        ],
        keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "CameraHelper.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "LidarHelper.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "OdometryPublisher.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "TFPublisher.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "ClockPublisher.inputs:execIn"),
        ],
    }
)

print("ROS 2 publishers configured!")
```

### Creating ROS 2 Subscribers (for Robot Control)

To control your robot from ROS 2, add a velocity subscriber:

```python
# File: setup_ros2_subscribers.py
# Set up ROS 2 subscriber for velocity commands

import omni.graph.core as og

# Add to existing graph or create new one
keys = og.Controller.Keys
og.Controller.edit(
    {"graph_path": "/World/Robot/ROS2_Graph"},
    {
        keys.CREATE_NODES: [
            ("SubscribeTwist", "omni.isaac.ros2_bridge.ROS2SubscribeTwist"),
            ("DifferentialController", "omni.isaac.wheeled_robots.DifferentialController"),
            ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
        ],
        keys.SET_VALUES: [
            ("SubscribeTwist.inputs:topicName", "/cmd_vel"),
            ("DifferentialController.inputs:wheelRadius", 0.05),  # meters
            ("DifferentialController.inputs:wheelDistance", 0.3), # meters
            ("ArticulationController.inputs:robotPath", "/World/Robot"),
        ],
        keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
            ("SubscribeTwist.outputs:linearVelocity", "DifferentialController.inputs:linearVelocity"),
            ("SubscribeTwist.outputs:angularVelocity", "DifferentialController.inputs:angularVelocity"),
            ("DifferentialController.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
        ],
    }
)

print("ROS 2 velocity subscriber configured!")
```

### Testing the ROS 2 Connection

With Isaac Sim running and ROS 2 bridge configured:

```bash
# In a terminal with ROS 2 sourced
source /opt/ros/humble/setup.bash

# List topics - should see Isaac Sim topics
ros2 topic list

# Expected output:
# /camera/image_raw
# /scan
# /odom
# /tf
# /clock
# /cmd_vel

# View camera image
ros2 run rqt_image_view rqt_image_view

# Send velocity command
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

---

## Synthetic Data Generation

One of Isaac Sim's most powerful features is **synthetic data generation**—creating labeled training data for AI models.

### What is Domain Randomization?

Real-world data is expensive to collect and label. Synthetic data solves this, but models trained only on "perfect" synthetic images do not generalize well.

**Domain randomization** adds controlled variation to synthetic data:
- Lighting changes (color, intensity, direction)
- Texture variations (material properties)
- Object positions and orientations
- Camera noise and distortion
- Background clutter

This forces the model to learn robust features rather than overfitting to specific visual patterns.

### Replicator: The Synthetic Data Pipeline

Isaac Sim uses **Replicator** for synthetic data generation:

```python
# File: generate_synthetic_data.py
# Generate training data with domain randomization

import omni.replicator.core as rep
from omni.isaac.core import World

# Initialize world
world = World()
world.scene.add_default_ground_plane()

# Define the scene with randomization
with rep.new_layer():
    # Create randomized lighting
    light = rep.create.light(
        light_type="distant",
        intensity=rep.distribution.uniform(500, 2000),
        color=rep.distribution.uniform((0.8, 0.8, 0.8), (1.0, 1.0, 1.0))
    )

    # Create objects with random positions
    cubes = rep.create.cube(
        count=5,
        position=rep.distribution.uniform((-2, -2, 0.5), (2, 2, 0.5)),
        rotation=rep.distribution.uniform((0, 0, 0), (0, 0, 360)),
        scale=rep.distribution.uniform(0.1, 0.5)
    )

    # Randomize textures
    with cubes:
        rep.randomizer.materials(
            materials=rep.create.material_omnipbr(
                diffuse=rep.distribution.uniform((0, 0, 0), (1, 1, 1))
            )
        )

    # Create camera
    camera = rep.create.camera(
        position=(0, -5, 2),
        look_at=(0, 0, 0)
    )

    # Set up data output
    render_product = rep.create.render_product(camera, (640, 480))

    # Configure output formats
    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(
        output_dir="/tmp/synthetic_data",
        rgb=True,
        bounding_box_2d_tight=True,
        semantic_segmentation=True,
        instance_segmentation=True,
        distance_to_camera=True
    )
    writer.attach([render_product])

# Generate 100 frames of data
rep.orchestrator.run_until_complete(num_frames=100)

print("Synthetic data generation complete!")
```

### Output Formats

Replicator can generate:

| Output Type | Description | Use Case |
|-------------|-------------|----------|
| RGB Image | Color image | General training |
| Depth | Distance per pixel | 3D perception |
| Bounding Boxes | Object locations | Object detection |
| Semantic Segmentation | Class per pixel | Scene understanding |
| Instance Segmentation | Object ID per pixel | Instance detection |
| Normals | Surface orientation | Surface analysis |

---

## Hands-On Exercise

### Exercise 1: Create a Warehouse Robot Simulation

**Objective:** Set up a complete Isaac Sim simulation with your robot in a warehouse environment, connected to ROS 2.

**Difficulty:** Intermediate

**Instructions:**

1. Create a new Isaac Sim stage
2. Load the warehouse environment from Nucleus
3. Import your URDF robot
4. Add camera and lidar sensors
5. Configure ROS 2 bridge
6. Verify data flows to ROS 2 topics

**Starter Code:**

```python
# File: warehouse_robot_setup.py
# Complete warehouse robot simulation setup

from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.sensor import Camera
import numpy as np

# TODO: Create the simulation world
world = None  # Create World instance

# TODO: Load warehouse environment from Nucleus
# Hint: Use add_reference_to_stage with warehouse asset path

# TODO: Import your URDF robot
# Hint: Enable urdf extension and use import commands

# TODO: Add camera sensor to robot
# Hint: Create Camera at /World/Robot/camera_link/Camera

# TODO: Add lidar sensor to robot
# Hint: Use RangeSensorCreateLidar command

# TODO: Enable ROS 2 bridge and configure publishers
# Hint: Enable omni.isaac.ros2_bridge extension

# Initialize simulation
if world:
    world.reset()
    print("Simulation ready!")
```

<details>
<summary>Click to reveal solution</summary>

```python
# File: warehouse_robot_setup_solution.py
# Complete warehouse robot simulation setup

from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.sensor import Camera
from omni.isaac.urdf import _urdf
import omni.kit.commands
import omni.graph.core as og
import numpy as np

# Enable required extensions
enable_extension("omni.isaac.urdf")
enable_extension("omni.isaac.ros2_bridge")

# Create the simulation world
world = World(stage_units_in_meters=1.0)

# Load warehouse environment
warehouse_path = "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/Environments/Simple_Warehouse/warehouse.usd"
add_reference_to_stage(usd_path=warehouse_path, prim_path="/World/Warehouse")

# Import URDF robot
urdf_path = "/home/user/ros2_ws/src/my_robot/urdf/robot.urdf"
import_config = _urdf.ImportConfig()
import_config.merge_fixed_joints = False
import_config.fix_base = False
import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_VELOCITY

omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path=urdf_path,
    import_config=import_config,
    dest_path="/World/Robot"
)

# Position robot in warehouse
from pxr import UsdGeom
stage = omni.usd.get_context().get_stage()
robot_prim = stage.GetPrimAtPath("/World/Robot")
xform = UsdGeom.Xformable(robot_prim)
xform.ClearXformOpOrder()
xform.AddTranslateOp().Set((0.0, 0.0, 0.1))

# Add camera sensor
camera = Camera(
    prim_path="/World/Robot/camera_link/RGBCamera",
    position=np.array([0.0, 0.0, 0.0]),
    frequency=30,
    resolution=(640, 480)
)
camera.initialize()

# Add lidar sensor
omni.kit.commands.execute(
    "RangeSensorCreateLidar",
    path="/World/Robot/lidar_link/Lidar",
    parent="/World/Robot/lidar_link",
    min_range=0.1,
    max_range=10.0,
    draw_points=True,
    horizontal_fov=360.0,
    horizontal_resolution=0.4,
    rotation_rate=10.0
)

# Configure ROS 2 publishers
keys = og.Controller.Keys
(graph_handle, nodes, _, _) = og.Controller.edit(
    {"graph_path": "/World/Robot/ROS2Graph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("CameraHelper", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
            ("LaserScan", "omni.isaac.ros2_bridge.ROS2PublishLaserScan"),
            ("ClockPub", "omni.isaac.ros2_bridge.ROS2PublishClock"),
            ("TFPub", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
            ("TwistSub", "omni.isaac.ros2_bridge.ROS2SubscribeTwist"),
        ],
        keys.SET_VALUES: [
            ("CameraHelper.inputs:topicName", "/camera/image_raw"),
            ("CameraHelper.inputs:frameId", "camera_link"),
            ("CameraHelper.inputs:type", "rgb"),
            ("LaserScan.inputs:topicName", "/scan"),
            ("LaserScan.inputs:frameId", "lidar_link"),
            ("TwistSub.inputs:topicName", "/cmd_vel"),
        ],
        keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "CameraHelper.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "LaserScan.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "ClockPub.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "TFPub.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "TwistSub.inputs:execIn"),
        ],
    }
)

# Reset and start
world.reset()
print("Warehouse robot simulation ready!")
print("ROS 2 topics available:")
print("  /camera/image_raw - RGB camera")
print("  /scan - Lidar scan")
print("  /cmd_vel - Velocity commands (subscriber)")
```

</details>

**Verification:**

```bash
# With Isaac Sim playing, run in terminal:
source /opt/ros/humble/setup.bash

# Check topics are available
ros2 topic list | grep -E "camera|scan|cmd_vel"

# Verify camera data is flowing
ros2 topic hz /camera/image_raw
# Expected: average rate: ~30 Hz

# Verify lidar data
ros2 topic hz /scan
# Expected: average rate: ~10 Hz

# Test robot control
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}}"
# Robot should move forward in Isaac Sim
```

---

## Common Issues and Solutions

### Issue 1: Isaac Sim Crashes on Startup

**Cause:** Insufficient GPU memory or incompatible driver.

**Solution:**
```bash
# Check GPU memory
nvidia-smi

# If VRAM is low, close other GPU applications
# Ensure driver is 525.60 or newer
# Try launching with reduced settings
./isaac-sim.sh --/renderer/active=rtx --/renderer/rtx/raytracing/taa/enabled=false
```

### Issue 2: ROS 2 Topics Not Appearing

**Cause:** ROS 2 bridge not enabled or ROS_DOMAIN_ID mismatch.

**Solution:**
```bash
# Ensure ROS_DOMAIN_ID matches
export ROS_DOMAIN_ID=0  # Default

# In Isaac Sim, verify bridge is enabled:
# Window > Extensions > search "ros2_bridge" > should be enabled

# Check Isaac Sim console for ROS 2 errors
```

### Issue 3: URDF Import Fails

**Cause:** Missing mesh files or invalid URDF syntax.

**Solution:**
```bash
# Verify URDF is valid
check_urdf /path/to/robot.urdf

# Ensure mesh paths are absolute or correctly referenced
# Try simplifying URDF (remove unused links/joints)
```

### Issue 4: Simulation Runs Very Slowly

**Cause:** Complex scene or physics settings.

**Solution:**
```python
# Reduce physics substeps
# In Isaac Sim settings or via Python:
from omni.isaac.core import World
world = World(physics_dt=1.0/60.0, rendering_dt=1.0/30.0)

# Use simpler collision meshes
# Reduce number of objects in scene
```

---

## Summary

### Key Takeaways

1. **Isaac Sim provides photorealistic simulation** using ray tracing and physically-based rendering. This is essential for training vision-based AI systems.

2. **USD is the scene format**—more powerful than SDF with support for references, variants, and composition. Scenes are stored in Nucleus server.

3. **The ROS 2 bridge connects Isaac Sim to your robot stack**. Configure publishers and subscribers using OmniGraph.

4. **URDF import is straightforward**—your existing robot descriptions work in Isaac Sim with the built-in importer.

5. **Synthetic data generation with Replicator** enables creating large labeled datasets with domain randomization for AI training.

### Quick Reference

```bash
# Launch Isaac Sim
cd ~/.local/share/ov/pkg/isaac_sim-2023.1.1
./isaac-sim.sh

# Useful keyboard shortcuts
Alt + LMB   - Orbit view
Alt + MMB   - Pan view
Alt + RMB   - Zoom
F           - Focus on selection
Space       - Play/Pause simulation

# Verify ROS 2 connection
ros2 topic list | grep isaac
```

```python
# Essential imports
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.sensor import Camera

# Enable ROS 2
enable_extension("omni.isaac.ros2_bridge")

# Create world
world = World(stage_units_in_meters=1.0)
```

---

## What's Next

Now that you can create photorealistic simulations in Isaac Sim, the next chapter focuses on **Isaac ROS**—GPU-accelerated perception packages that run in real-time. You will learn to process camera images, detect objects, and estimate depth using NVIDIA's optimized implementations.

[Next: Chapter 2 - Isaac ROS →](/docs/module-3-nvidia-isaac/isaac-ros)

---

## Additional Resources

- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/) - Official documentation
- [USD Tutorials](https://openusd.org/docs/Tutorials.html) - Learn USD format
- [Replicator Documentation](https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator.html) - Synthetic data generation
- [Isaac Sim ROS 2 Examples](https://github.com/NVIDIA-Omniverse/IsaacSim-ros_workspaces) - Sample workspaces
