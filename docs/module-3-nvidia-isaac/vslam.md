---
sidebar_position: 4
title: "Chapter 3: Visual SLAM"
description: "Implement visual SLAM for robot localization without GPS. Build maps and track robot position using camera input with Isaac ROS Visual SLAM."
keywords: [VSLAM, visual SLAM, localization, mapping, NVIDIA, stereo vision, odometry, cuVSLAM, ROS 2]
---

# Chapter 3: Visual SLAM

## Chapter Overview

Your robot can now see and detect objects. But there is a fundamental question it cannot answer: **"Where am I?"**

In outdoor environments, GPS provides location. But indoors—in warehouses, hospitals, homes—GPS does not work. The robot must figure out its position using only its sensors.

**SLAM** (Simultaneous Localization and Mapping) solves this. The robot builds a map of its environment while simultaneously tracking its position within that map. It is a chicken-and-egg problem: you need a map to know where you are, but you need to know where you are to build a map. SLAM solves both simultaneously.

**Visual SLAM** uses cameras as the primary sensor. This is particularly powerful because cameras are cheap, provide rich information, and are already on most robots for other perception tasks.

In this chapter, you will learn SLAM fundamentals and implement Isaac ROS Visual SLAM (cuVSLAM) for real-time robot localization.

**Prerequisites**: Completed [Chapter 2: Isaac ROS](/docs/module-3-nvidia-isaac/isaac-ros), understanding of coordinate frames and transforms

**What You Will Learn**: SLAM concepts, visual odometry, map building, localization, and integration with ROS 2 transforms

---

## Learning Objectives

After completing this chapter, you will be able to:

- Explain the SLAM problem and why it is challenging
- Distinguish between visual odometry and full SLAM
- Configure Isaac ROS Visual SLAM for your robot
- Build maps using stereo cameras
- Localize in pre-built maps
- Integrate VSLAM with the ROS 2 transform system

---

## Understanding SLAM

Before implementing SLAM, let us understand what problem we are solving.

### The Localization Problem

Imagine you wake up in an unfamiliar building. How do you figure out where you are?

1. **Look around** - observe distinctive features (doors, windows, signs)
2. **Start walking** - track how far you have moved
3. **Recognize places** - "I have been here before!"
4. **Build mental map** - understand the building's layout

Robots do exactly the same thing, but mathematically.

### What SLAM Does

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           THE SLAM PROBLEM                                   │
│                                                                              │
│   Robot has:                           Robot needs:                          │
│   • Camera images                      • Its position (x, y, θ)              │
│   • Wheel odometry (noisy)            • A map of the environment             │
│   • No GPS                             • Both at the same time!              │
│                                                                              │
│   ┌─────────┐          ┌─────────┐          ┌─────────┐                     │
│   │ Camera  │ ───────▶ │  SLAM   │ ───────▶ │  Map +  │                     │
│   │ Images  │          │ System  │          │  Pose   │                     │
│   └─────────┘          └─────────┘          └─────────┘                     │
│        │                    │                    │                           │
│        │                    │                    │                           │
│        ▼                    ▼                    ▼                           │
│   [Observe         [Extract features,    [Know where robot                   │
│    environment]     match & track,        is in the world]                   │
│                     optimize positions]                                      │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

SLAM maintains two things:
1. **Map**: A representation of the environment (feature locations, point cloud, etc.)
2. **Pose**: The robot's position and orientation in that map

### Why SLAM is Hard

SLAM seems straightforward, but several factors make it challenging:

**Sensor Noise**: Cameras have measurement errors. Small errors accumulate over time (drift).

**Dynamic Environments**: People walk by, doors open and close. The "map" is not static.

**Perceptual Aliasing**: Different places can look similar (all hallways look alike).

**Loop Closure**: When the robot returns to a previously visited location, it must recognize this and correct accumulated drift.

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           DRIFT AND LOOP CLOSURE                             │
│                                                                              │
│   Without loop closure:              With loop closure:                      │
│                                                                              │
│        Start                              Start                              │
│          │                                  │                                │
│          ▼                                  ▼                                │
│      ┌──────┐                           ┌──────┐                            │
│      │      │                           │      │                            │
│      │  ?───┼───X  ← Drift             │      │                            │
│      │      │        accumulated        │      │                            │
│      └──────┘                           └──┬───┘                            │
│                                            │                                │
│   Robot thinks it's at X,               Loop detected!                      │
│   but actually back at start            Position corrected                  │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Visual Odometry vs Full SLAM

It is important to understand the difference between visual odometry (VO) and full SLAM.

### Visual Odometry

Visual odometry tracks motion **frame-to-frame**. It answers: "How did I move since the last image?"

```
Frame N-1          Frame N            Motion Estimate
┌─────────┐       ┌─────────┐
│  * *    │       │   * *   │        Δx = 0.1m
│    *    │  ───▶ │     *   │  ───▶  Δy = 0.02m
│  *   *  │       │   *   * │        Δθ = 2°
└─────────┘       └─────────┘
```

**Pros**: Fast, simple, works immediately
**Cons**: Drift accumulates, no map building, no loop closure

### Full SLAM

Full SLAM maintains a **global map** and performs **loop closure**. It answers: "Where am I in the world?"

```
Full SLAM maintains:
• Feature database (for recognition)
• Graph of poses and constraints
• Optimization to minimize error

When loop detected → Correct entire trajectory
```

**Pros**: Globally consistent, corrects drift, builds reusable map
**Cons**: More complex, higher compute requirements

### Isaac ROS Visual SLAM

Isaac ROS Visual SLAM (cuVSLAM) provides:
- GPU-accelerated visual odometry
- Optional loop closure and map building
- Integration with ROS 2 transforms
- Support for stereo and depth cameras

---

## Setting Up Isaac ROS Visual SLAM

Let us configure cuVSLAM for your robot.

### Prerequisites

```bash
# Inside Isaac ROS container
cd ~/isaac_ros_ws/src

# Clone the Visual SLAM repository
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git

# Build
cd /workspaces/isaac_ros-dev
colcon build --packages-select isaac_ros_visual_slam
source install/setup.bash
```

### Camera Requirements

cuVSLAM works best with:
- **Stereo cameras**: Two cameras with known baseline (recommended)
- **Depth cameras**: RGB-D cameras like RealSense
- **Monocular**: Single camera (limited, no scale)

For this chapter, we will use **stereo cameras** as they provide the best results.

### Camera Calibration

Visual SLAM requires accurate camera calibration. If not calibrated, results will be poor.

```yaml
# File: camera_info.yaml
# Stereo camera calibration parameters

image_width: 640
image_height: 480

# Left camera intrinsics
left:
  camera_matrix:
    rows: 3
    cols: 3
    data: [615.0, 0.0, 320.0,
           0.0, 615.0, 240.0,
           0.0, 0.0, 1.0]
  distortion_coefficients:
    rows: 1
    cols: 5
    data: [0.0, 0.0, 0.0, 0.0, 0.0]

# Right camera intrinsics (same as left if identical cameras)
right:
  camera_matrix:
    rows: 3
    cols: 3
    data: [615.0, 0.0, 320.0,
           0.0, 615.0, 240.0,
           0.0, 0.0, 1.0]
  distortion_coefficients:
    rows: 1
    cols: 5
    data: [0.0, 0.0, 0.0, 0.0, 0.0]

# Stereo baseline (distance between cameras in meters)
baseline: 0.12  # 12 cm
```

---

## Configuring cuVSLAM

### Launch File

```python
# File: vslam.launch.py
# Isaac ROS Visual SLAM launch configuration

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Launch Visual SLAM for stereo cameras."""

    # Launch arguments
    enable_localization = LaunchConfiguration('enable_localization')
    enable_mapping = LaunchConfiguration('enable_mapping')
    map_path = LaunchConfiguration('map_path')

    vslam_container = ComposableNodeContainer(
        name='vslam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_visual_slam',
                plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
                name='visual_slam',
                remappings=[
                    # Stereo camera inputs
                    ('stereo_camera/left/image', '/stereo/left/image_rect'),
                    ('stereo_camera/left/camera_info', '/stereo/left/camera_info'),
                    ('stereo_camera/right/image', '/stereo/right/image_rect'),
                    ('stereo_camera/right/camera_info', '/stereo/right/camera_info'),
                ],
                parameters=[{
                    # Enable GPU acceleration
                    'enable_gpu': True,
                    'enable_debug_mode': False,

                    # Frame configuration
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    'base_frame': 'base_link',
                    'input_left_camera_frame': 'camera_left',
                    'input_right_camera_frame': 'camera_right',

                    # SLAM configuration
                    'enable_localization_n_mapping': True,
                    'enable_slam_visualization': True,

                    # Visual odometry settings
                    'rectified_images': True,
                    'enable_observations_view': False,
                    'enable_landmarks_view': True,

                    # Loop closure settings
                    'enable_loop_closure': True,
                    'loop_closure_method': 'dbow2',

                    # Performance tuning
                    'image_jitter_threshold_ms': 35.0,
                    'sync_matching_threshold_ms': 5.0,
                }]
            )
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_localization',
            default_value='true',
            description='Enable localization mode'
        ),
        DeclareLaunchArgument(
            'enable_mapping',
            default_value='true',
            description='Enable mapping mode'
        ),
        DeclareLaunchArgument(
            'map_path',
            default_value='',
            description='Path to load/save map'
        ),
        vslam_container
    ])
```

### Key Parameters Explained

| Parameter | Description | Typical Value |
|-----------|-------------|---------------|
| `enable_gpu` | Use GPU acceleration | `true` |
| `map_frame` | TF frame for the map origin | `"map"` |
| `odom_frame` | TF frame for odometry | `"odom"` |
| `base_frame` | Robot's base frame | `"base_link"` |
| `enable_loop_closure` | Detect revisited places | `true` |
| `rectified_images` | Input images are rectified | `true` |

---

## Running Visual SLAM

### Building a Map

Let us build a map of your environment:

```bash
# Terminal 1: Launch your robot simulation (Isaac Sim or Gazebo)
# Ensure stereo cameras are publishing

# Terminal 2: Launch Visual SLAM
ros2 launch my_robot vslam.launch.py enable_mapping:=true

# Terminal 3: Teleoperate the robot
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Drive slowly around the environment
# Return to starting position to trigger loop closure
```

### Monitoring SLAM Status

```bash
# View SLAM status
ros2 topic echo /visual_slam/status

# Example output:
# ---
# vo_state: 1        # 0=lost, 1=tracking
# node_callback_id: 1234
# tracking_quality: 0.95
# loop_closure_detected: true

# View current pose
ros2 topic echo /visual_slam/tracking/odometry
```

### Visualizing in RViz2

Create an RViz2 configuration to visualize SLAM:

```yaml
# File: vslam.rviz (partial configuration)
Displays:
  - Class: rviz_default_plugins/TF
    Name: TF
    Enabled: true
    Frame Timeout: 15
    Show Arrows: true
    Show Axes: true
    Show Names: true

  - Class: rviz_default_plugins/PointCloud2
    Name: SLAM Landmarks
    Topic: /visual_slam/vis/landmarks_cloud
    Size (m): 0.05
    Color Transformer: FlatColor
    Color: 0; 255; 0  # Green

  - Class: rviz_default_plugins/Path
    Name: Trajectory
    Topic: /visual_slam/tracking/slam_path
    Color: 255; 0; 0  # Red

  - Class: rviz_default_plugins/Marker
    Name: Loop Closures
    Topic: /visual_slam/vis/loop_closure_markers
```

```bash
# Launch RViz2 with configuration
rviz2 -d vslam.rviz
```

---

## Understanding Visual SLAM Output

### TF Transforms

cuVSLAM publishes transforms that integrate with the ROS 2 transform system:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        VSLAM TRANSFORM TREE                                  │
│                                                                              │
│                         ┌─────────┐                                         │
│                         │   map   │  (Fixed world frame)                    │
│                         └────┬────┘                                         │
│                              │                                              │
│                         ┌────┴────┐                                         │
│                         │  odom   │  (Continuous, may drift)                │
│                         └────┬────┘                                         │
│                              │                                              │
│                         ┌────┴────┐                                         │
│                         │base_link│  (Robot body)                           │
│                         └────┬────┘                                         │
│                   ┌──────────┼──────────┐                                   │
│              ┌────┴────┐ ┌───┴───┐ ┌────┴────┐                              │
│              │camera_L │ │ lidar │ │camera_R │                              │
│              └─────────┘ └───────┘ └─────────┘                              │
│                                                                              │
│   map → odom: Published by VSLAM (corrects drift via loop closure)          │
│   odom → base_link: Published by VSLAM (visual odometry)                    │
│   base_link → sensors: Static transforms from URDF                          │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

**Important**: cuVSLAM publishes both:
- `odom → base_link`: The visual odometry (smooth, may drift)
- `map → odom`: The correction from loop closure (jumps when loop detected)

The combination `map → base_link` gives the globally corrected position.

### Odometry Output

```bash
# View odometry (continuous pose estimate)
ros2 topic echo /visual_slam/tracking/odometry

# nav_msgs/Odometry message:
# header:
#   stamp: {sec: 1234, nanosec: 567890}
#   frame_id: "odom"
# child_frame_id: "base_link"
# pose:
#   pose:
#     position: {x: 1.234, y: 0.567, z: 0.0}
#     orientation: {x: 0.0, y: 0.0, z: 0.1, w: 0.995}
#   covariance: [...]  # 6x6 uncertainty
# twist:
#   twist:
#     linear: {x: 0.1, y: 0.0, z: 0.0}
#     angular: {x: 0.0, y: 0.0, z: 0.05}
```

---

## Saving and Loading Maps

### Saving a Map

After exploring the environment, save the map for later use:

```bash
# Call the save map service
ros2 service call /visual_slam/save_map isaac_ros_visual_slam_interfaces/srv/FilePath \
    "{file_path: '/tmp/my_map'}"

# This creates:
# /tmp/my_map.db  - Landmark database
# /tmp/my_map.pb  - Map metadata
```

### Loading a Map for Localization

Start SLAM in localization-only mode with a pre-built map:

```python
# File: vslam_localization.launch.py
# Launch VSLAM in localization mode

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Launch Visual SLAM in localization mode."""

    vslam_container = ComposableNodeContainer(
        name='vslam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_visual_slam',
                plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
                name='visual_slam',
                remappings=[
                    ('stereo_camera/left/image', '/stereo/left/image_rect'),
                    ('stereo_camera/left/camera_info', '/stereo/left/camera_info'),
                    ('stereo_camera/right/image', '/stereo/right/image_rect'),
                    ('stereo_camera/right/camera_info', '/stereo/right/camera_info'),
                ],
                parameters=[{
                    'enable_gpu': True,

                    # Frame configuration
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    'base_frame': 'base_link',

                    # Localization mode - load map, don't modify
                    'enable_localization_n_mapping': False,
                    'localize_in_map': True,
                    'map_file_path': '/tmp/my_map',

                    # Still enable loop closure for relocalization
                    'enable_loop_closure': True,
                }]
            )
        ],
        output='screen'
    )

    return LaunchDescription([vslam_container])
```

### Initial Localization

When starting in a known map, the robot needs to find its initial position:

```bash
# If the robot starts near a known location, it will localize automatically
# If not, you may need to provide an initial pose hint:

ros2 service call /visual_slam/set_pose geometry_msgs/srv/Pose \
    "{position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"
```

---

## Integration with Robot Navigation

Visual SLAM provides localization for navigation. Here is how to integrate it:

### Transform Configuration

Ensure your robot's URDF publishes the correct static transforms:

```xml
<!-- In robot.urdf -->
<!-- Stereo camera transforms -->
<joint name="camera_left_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_left"/>
  <origin xyz="0.2 0.06 0.1" rpy="0 0 0"/>
</joint>

<joint name="camera_right_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_right"/>
  <origin xyz="0.2 -0.06 0.1" rpy="0 0 0"/>
</joint>
```

### Using VSLAM Odometry with Nav2

Nav2 expects odometry on `/odom` topic. Configure the topic remapping:

```python
# In your vslam launch file, add remapping
remappings=[
    # ... existing remappings ...
    ('/visual_slam/tracking/odometry', '/odom'),
]
```

Or use a relay node:

```bash
ros2 run topic_tools relay /visual_slam/tracking/odometry /odom
```

### Complete Localization Pipeline

```python
# File: robot_localization.launch.py
# Complete robot localization with VSLAM

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """Launch complete localization pipeline."""

    pkg_share = get_package_share_directory('my_robot')

    return LaunchDescription([
        # Launch Visual SLAM
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'vslam.launch.py')
            ),
            launch_arguments={
                'enable_localization': 'true',
                'enable_mapping': 'false',
                'map_path': '/tmp/my_map'
            }.items()
        ),

        # Robot state publisher (for URDF transforms)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': open(
                    os.path.join(pkg_share, 'urdf', 'robot.urdf')
                ).read()
            }]
        ),

        # Relay odometry to standard topic
        Node(
            package='topic_tools',
            executable='relay',
            arguments=['/visual_slam/tracking/odometry', '/odom'],
            output='screen'
        ),
    ])
```

---

## Handling SLAM Failures

Visual SLAM can fail in certain conditions. Here is how to handle them:

### Tracking Loss

Tracking loss occurs when SLAM cannot find enough features:

```python
# File: vslam_monitor.py
# Monitor VSLAM status and handle failures

import rclpy
from rclpy.node import Node
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from geometry_msgs.msg import Twist

class VSLAMMonitor(Node):
    def __init__(self):
        super().__init__('vslam_monitor')

        self.status_sub = self.create_subscription(
            VisualSlamStatus,
            '/visual_slam/status',
            self.status_callback,
            10
        )

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.tracking_lost = False
        self.get_logger().info('VSLAM Monitor started')

    def status_callback(self, msg):
        # vo_state: 0 = lost, 1 = tracking
        if msg.vo_state == 0 and not self.tracking_lost:
            self.tracking_lost = True
            self.get_logger().warn('VSLAM tracking lost! Stopping robot.')
            self.stop_robot()

        elif msg.vo_state == 1 and self.tracking_lost:
            self.tracking_lost = False
            self.get_logger().info('VSLAM tracking recovered.')

    def stop_robot(self):
        """Stop the robot when tracking is lost."""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)


def main():
    rclpy.init()
    node = VSLAMMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Recovery Strategies

| Failure | Cause | Recovery |
|---------|-------|----------|
| Tracking lost | Rapid motion, blur | Stop, wait, move slowly |
| Poor quality | Textureless scene | Move to feature-rich area |
| Drift | No loop closure | Return to known location |
| Cannot localize | Changed environment | Rebuild map |

---

## Hands-On Exercise

### Exercise: Build and Use a Navigation Map

**Objective:** Build a VSLAM map, save it, then use it for localization.

**Difficulty:** Advanced

**Instructions:**

1. Launch Isaac Sim with your robot in a warehouse
2. Start VSLAM in mapping mode
3. Teleoperate to explore the environment (complete a loop)
4. Save the map
5. Restart VSLAM in localization mode
6. Verify the robot localizes correctly

**Starter Code:**

```python
# File: mapping_exercise.py
# Guide for the mapping exercise

"""
Exercise Steps:

1. Launch simulation and VSLAM
   Terminal 1: Launch Isaac Sim with robot
   Terminal 2: ros2 launch my_robot vslam.launch.py enable_mapping:=true
   Terminal 3: ros2 run teleop_twist_keyboard teleop_twist_keyboard
   Terminal 4: rviz2 -d vslam.rviz

2. Explore the environment
   - Drive slowly (max 0.3 m/s)
   - Cover all areas you want mapped
   - Return to starting position for loop closure
   - Watch for loop closure in RViz (trajectory should "snap")

3. Save the map
   ros2 service call /visual_slam/save_map \
       isaac_ros_visual_slam_interfaces/srv/FilePath \
       "{file_path: '/tmp/warehouse_map'}"

4. Verify save
   ls -la /tmp/warehouse_map*
   # Should see .db and .pb files

5. Restart in localization mode
   # Stop VSLAM (Ctrl+C)
   ros2 launch my_robot vslam.launch.py \
       enable_mapping:=false \
       map_path:=/tmp/warehouse_map

6. Test localization
   - Move the robot around
   - Verify pose updates correctly
   - Check TF in RViz
"""

import subprocess
import time

def run_mapping_workflow():
    """Automated workflow guide."""
    print("=" * 60)
    print("VISUAL SLAM MAPPING EXERCISE")
    print("=" * 60)

    steps = [
        "1. Launch Isaac Sim with robot and stereo cameras",
        "2. Launch VSLAM: ros2 launch my_robot vslam.launch.py enable_mapping:=true",
        "3. Launch RViz2: rviz2 -d vslam.rviz",
        "4. Launch teleop: ros2 run teleop_twist_keyboard teleop_twist_keyboard",
        "5. Explore environment slowly, covering all areas",
        "6. Return to start to trigger loop closure",
        "7. Save map: ros2 service call /visual_slam/save_map ...",
        "8. Stop VSLAM and restart in localization mode",
        "9. Verify localization works correctly"
    ]

    for step in steps:
        print(f"\n{step}")
        input("Press Enter when complete...")

    print("\n" + "=" * 60)
    print("Exercise complete!")
    print("=" * 60)


if __name__ == '__main__':
    run_mapping_workflow()
```

**Verification Checklist:**

```bash
# Check map files exist
ls -la /tmp/warehouse_map*

# In localization mode, verify transforms
ros2 topic echo /tf --once | grep -A5 "map"
# Should show map → odom transform

# Check tracking quality
ros2 topic echo /visual_slam/status --once
# vo_state should be 1 (tracking)

# Verify pose changes when robot moves
ros2 topic echo /visual_slam/tracking/odometry
# Position values should update as robot moves
```

<details>
<summary>Click to reveal verification commands</summary>

```bash
# Complete verification sequence

# 1. Check VSLAM is running
ros2 node list | grep visual_slam
# Should show: /visual_slam

# 2. Check topics are being published
ros2 topic list | grep visual_slam
# Should include:
# /visual_slam/tracking/odometry
# /visual_slam/vis/landmarks_cloud
# /visual_slam/status

# 3. Verify camera input
ros2 topic hz /stereo/left/image_rect
# Should show ~30 Hz

# 4. Check odometry rate
ros2 topic hz /visual_slam/tracking/odometry
# Should match camera rate (~30 Hz)

# 5. Test localization after moving
ros2 topic echo /visual_slam/tracking/odometry --once
# Note the position
# Move robot
ros2 topic echo /visual_slam/tracking/odometry --once
# Position should have changed

# 6. Verify TF tree is complete
ros2 run tf2_tools view_frames
# Should create frames.pdf showing complete tree
```

</details>

---

## Common Issues and Solutions

### Issue 1: No Odometry Output

**Cause:** Camera frames not matching, or images not synchronized.

**Solution:**
```bash
# Check camera topics are publishing
ros2 topic hz /stereo/left/image_rect
ros2 topic hz /stereo/right/image_rect

# Verify synchronization (timestamps should be close)
ros2 topic echo /stereo/left/camera_info --once
ros2 topic echo /stereo/right/camera_info --once

# Check VSLAM is receiving images
ros2 topic echo /visual_slam/status
# frame_id should increment
```

### Issue 2: High Drift

**Cause:** Poor calibration or insufficient features.

**Solution:**
```bash
# Recalibrate cameras
ros2 run camera_calibration cameracalibrator \
    --size 8x6 --square 0.025 \
    --approximate=0.1 \
    image:=/stereo/left/image_raw \
    camera:=/stereo/left

# Ensure environment has texture
# Add patterns/posters to textureless walls
```

### Issue 3: Loop Closure Not Triggering

**Cause:** Not returning close enough to original position, or too few shared features.

**Solution:**
```python
# In VSLAM parameters, adjust loop closure sensitivity:
parameters=[{
    'enable_loop_closure': True,
    'loop_closure_method': 'dbow2',
    # Lower threshold = more sensitive (may get false positives)
    # Higher threshold = less sensitive
}]
```

### Issue 4: TF Tree Broken

**Cause:** Multiple sources publishing same transform, or missing transforms.

**Solution:**
```bash
# View TF tree
ros2 run tf2_tools view_frames

# Check for duplicate publishers
ros2 topic echo /tf | grep -A3 "child_frame_id"

# Ensure robot_state_publisher is not conflicting
# with VSLAM-published transforms
```

---

## Summary

### Key Takeaways

1. **SLAM solves two problems simultaneously**: building a map and localizing within it. Visual SLAM uses cameras as the primary sensor.

2. **Visual odometry tracks frame-to-frame motion** but drifts over time. Full SLAM with loop closure corrects this drift.

3. **cuVSLAM is GPU-accelerated** and integrates with ROS 2 transforms. It publishes both visual odometry and loop-closure corrected poses.

4. **Maps can be saved and reused** for localization-only mode. This is essential for production deployments.

5. **Handle SLAM failures gracefully** by monitoring status and stopping the robot when tracking is lost.

### Quick Reference

```bash
# Launch VSLAM in mapping mode
ros2 launch my_robot vslam.launch.py enable_mapping:=true

# Save map
ros2 service call /visual_slam/save_map \
    isaac_ros_visual_slam_interfaces/srv/FilePath \
    "{file_path: '/tmp/my_map'}"

# Launch in localization mode
ros2 launch my_robot vslam.launch.py \
    enable_mapping:=false \
    map_path:=/tmp/my_map

# Monitor status
ros2 topic echo /visual_slam/status

# View trajectory in RViz
# Topic: /visual_slam/tracking/slam_path
```

```python
# Essential VSLAM parameters
parameters=[{
    'enable_gpu': True,
    'map_frame': 'map',
    'odom_frame': 'odom',
    'base_frame': 'base_link',
    'enable_loop_closure': True,
    'rectified_images': True,
}]
```

---

## What's Next

Your robot can now perceive objects and know where it is. The final piece is **autonomous navigation**—planning paths and executing them while avoiding obstacles. The next chapter covers **Nav2**, the ROS 2 navigation stack, and how to integrate it with your Isaac perception and localization.

[Next: Chapter 4 - Nav2 Integration →](/docs/module-3-nvidia-isaac/nav2)

---

## Additional Resources

- [Isaac ROS Visual SLAM Documentation](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/index.html) - Official documentation
- [SLAM for Dummies](https://dspace.mit.edu/handle/1721.1/36832) - Introductory tutorial
- [ORB-SLAM Paper](https://arxiv.org/abs/1502.00956) - Foundational visual SLAM work
- [TF2 Documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html) - ROS 2 transform system
