---
sidebar_position: 5
title: "Chapter 4: Sensor Simulation"
description: "Simulate robot sensors including cameras, LiDAR, IMU, and depth sensors. Add realistic noise models for accurate perception algorithm testing."
keywords: [sensors, camera, LiDAR, IMU, depth sensor, simulation, perception, noise models, Gazebo]
---

# Chapter 4: Sensor Simulation

## Chapter Overview

In the real world, robots do not have magical knowledge of their environment. They must **perceive** the world through sensors—cameras that see, lidars that measure distances, and IMUs that feel acceleration and rotation. A robot without sensors is blind, deaf, and unaware of its surroundings.

In simulation, we must recreate these sensors virtually. This chapter teaches you to add simulated cameras, lidars, IMUs, and depth sensors to your robot models. But here is the crucial part: **real sensors are not perfect**. They have noise, distortion, and limitations. To properly test perception algorithms, your simulated sensors must behave like real ones—imperfections included.

Think of sensor simulation as giving your robot its **senses**. Just like humans have eyes, ears, and a sense of balance, robots have cameras, microphones, and IMUs. The quality of your sensor simulation directly affects how well your algorithms will work on real hardware.

**Prerequisites**: Completed [Chapter 1: Gazebo Basics](/docs/module-2-simulation/gazebo-basics)

**What You Will Learn**: Simulate all major robot sensors with realistic noise models

---

## Learning Objectives

After completing this chapter, you will be able to:

- Add camera sensors to your robot and publish images to ROS 2 topics
- Simulate 2D and 3D lidar sensors with configurable parameters
- Model IMU sensors for robot orientation and acceleration
- Configure depth cameras and RGB-D sensors
- Add realistic noise to sensor outputs
- Understand sensor update rates and their impact on system design

---

## Why Sensor Simulation Matters

Before diving into implementation, let us understand why accurate sensor simulation is critical.

### The Perception Pipeline

Every autonomous robot follows a similar pattern:

```
┌─────────────────────────────────────────────────────────────────┐
│                    ROBOT PERCEPTION PIPELINE                     │
│                                                                  │
│   ┌───────────┐     ┌───────────┐     ┌───────────┐            │
│   │  SENSORS  │────▶│ PERCEPTION │────▶│ PLANNING  │            │
│   │           │     │ ALGORITHMS │     │           │            │
│   │ • Camera  │     │            │     │ "Go to    │            │
│   │ • Lidar   │     │ • Object   │     │  kitchen" │            │
│   │ • IMU     │     │   detection│     │           │            │
│   │           │     │ • SLAM     │     │           │            │
│   └───────────┘     └───────────┘     └─────┬─────┘            │
│                                              │                   │
│                                              ▼                   │
│                                        ┌───────────┐            │
│                                        │  CONTROL  │            │
│                                        │           │            │
│                                        │ Move      │            │
│                                        │ motors    │            │
│                                        └───────────┘            │
│                                                                  │
│   If sensors in simulation are perfect but real sensors are not, │
│   your perception algorithms will fail on real hardware!         │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### The Problem with Perfect Sensors

Imagine training an object detection system with perfect simulated camera images—no blur, no noise, perfect lighting. When you deploy to a real robot:

- The real camera has motion blur
- The lighting is different than simulation
- There is sensor noise in every pixel

Your algorithm may fail completely because it was never trained on realistic data.

**Bottom line**: Good sensor simulation includes realistic imperfections.

---

## Camera Sensors

Cameras are the most common robot sensors. They capture images that perception algorithms use for object detection, navigation, and more.

### Adding a Camera in Gazebo

To add a camera to your robot, add a sensor to a link in your URDF or SDF:

```xml
<!-- Add to your robot URDF inside a <link> element -->
<gazebo reference="camera_link">
  <sensor name="camera_sensor" type="camera">
    <!-- Update rate in Hz -->
    <update_rate>30.0</update_rate>

    <!-- Camera properties -->
    <camera>
      <!-- Image size -->
      <horizontal_fov>1.047</horizontal_fov>  <!-- ~60 degrees -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>  <!-- RGB 8-bit per channel -->
      </image>

      <!-- Clipping planes (min/max render distance) -->
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>

      <!-- Add noise for realism -->
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>  <!-- Small amount of noise -->
      </noise>
    </camera>

    <!-- Plugin to publish to ROS 2 -->
    <plugin name="camera_plugin" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>image_raw:=/robot/camera/image_raw</remapping>
        <remapping>camera_info:=/robot/camera/camera_info</remapping>
      </ros>
      <camera_name>camera</camera_name>
      <frame_name>camera_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

Let me explain the key parameters:

**`<horizontal_fov>`**: Field of view in radians. 1.047 radians = 60 degrees. Wider FOV sees more but with more distortion.

**`<image>`**: Resolution and format. Higher resolution = more detail but more processing required.

**`<clip>`**: Objects closer than `near` or farther than `far` are not rendered. Set `near` small enough to see close objects.

**`<noise>`**: Adds Gaussian noise to each pixel. Essential for realistic simulation.

### Viewing Camera Output

```bash
# View the camera image
ros2 run rqt_image_view rqt_image_view

# Or use image_view
ros2 run image_view image_view --ros-args -r image:=/robot/camera/image_raw

# Check camera info
ros2 topic echo /robot/camera/camera_info
```

### Camera Parameters Explained

| Parameter | Typical Value | Effect |
|-----------|---------------|--------|
| Width x Height | 640x480, 1280x720 | Resolution (higher = more detail) |
| FOV | 60°-90° | Field of view (wider = more visible area) |
| Update rate | 15-60 Hz | Frames per second |
| Noise stddev | 0.005-0.02 | Amount of pixel noise |

---

## Depth Cameras and RGB-D Sensors

Depth cameras provide both color images AND distance measurements for each pixel. Common examples include Intel RealSense and Microsoft Kinect.

### How Depth Cameras Work

```
┌─────────────────────────────────────────────────────────────────┐
│                    DEPTH CAMERA OUTPUT                           │
│                                                                  │
│   RGB Image:                      Depth Image:                   │
│   ┌─────────────────────┐        ┌─────────────────────┐        │
│   │                     │        │ █████████████████   │        │
│   │    [Color image     │        │ ███  Dark = close   │        │
│   │     of the scene]   │        │ ░░░  Light = far    │        │
│   │                     │        │ ████████░░░░░░░░░   │        │
│   │                     │        │ ████░░░░░░░░░░░░░   │        │
│   └─────────────────────┘        └─────────────────────┘        │
│                                                                  │
│   Combined: For each pixel, you know the COLOR and the DISTANCE │
│   This enables 3D reconstruction of the environment             │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### Adding a Depth Camera in Gazebo

```xml
<!-- Depth camera sensor (like Intel RealSense) -->
<gazebo reference="depth_camera_link">
  <sensor name="depth_camera" type="depth">
    <update_rate>30.0</update_rate>

    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.3</near>   <!-- RealSense min range ~0.3m -->
        <far>10.0</far>    <!-- Max useful range ~10m -->
      </clip>

      <!-- Depth-specific noise -->
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>
      </noise>
    </camera>

    <plugin name="depth_camera_plugin" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/robot</namespace>
      </ros>
      <camera_name>depth_camera</camera_name>
      <frame_name>depth_camera_link</frame_name>
      <hack_baseline>0.07</hack_baseline>  <!-- Stereo baseline for depth -->
    </plugin>
  </sensor>
</gazebo>
```

### Depth Camera Topics

A depth camera publishes multiple topics:

| Topic | Content | Message Type |
|-------|---------|--------------|
| `/depth_camera/image_raw` | RGB image | sensor_msgs/Image |
| `/depth_camera/depth/image_raw` | Depth image | sensor_msgs/Image |
| `/depth_camera/points` | 3D point cloud | sensor_msgs/PointCloud2 |
| `/depth_camera/camera_info` | Calibration | sensor_msgs/CameraInfo |

---

## LiDAR Sensors

LiDAR (Light Detection and Ranging) measures distances by shooting laser beams and timing how long they take to return. Lidars are essential for autonomous navigation because they provide accurate distance measurements in all directions.

### 2D LiDAR (Laser Scanner)

A 2D lidar scans in a single plane, producing a ring of distance measurements.

```xml
<!-- 2D Lidar (like RPLIDAR, Hokuyo) -->
<gazebo reference="lidar_link">
  <sensor name="lidar" type="ray">
    <update_rate>10.0</update_rate>  <!-- 10 Hz scan rate -->

    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>      <!-- 360 rays = 1 degree resolution -->
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>  <!-- -180 degrees -->
          <max_angle>3.14159</max_angle>   <!-- +180 degrees -->
        </horizontal>
      </scan>
      <range>
        <min>0.10</min>   <!-- Minimum range (meters) -->
        <max>12.0</max>   <!-- Maximum range (meters) -->
        <resolution>0.01</resolution>  <!-- Distance resolution -->
      </range>

      <!-- Noise model -->
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>  <!-- 1cm standard deviation -->
      </noise>
    </ray>

    <plugin name="lidar_plugin" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/out:=/robot/scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### Visualizing LiDAR Data

```bash
# View in RViz
ros2 run rviz2 rviz2

# In RViz:
# 1. Set Fixed Frame to "lidar_link" or "base_link"
# 2. Add "LaserScan" display
# 3. Set topic to /robot/scan

# Or view raw data
ros2 topic echo /robot/scan
```

### 3D LiDAR (Point Cloud)

3D lidars scan in multiple planes, producing a full 3D point cloud. Examples include Velodyne and Ouster sensors.

```xml
<!-- 3D Lidar (like Velodyne VLP-16) -->
<gazebo reference="lidar_3d_link">
  <sensor name="lidar_3d" type="gpu_ray">  <!-- gpu_ray for better performance -->
    <update_rate>10.0</update_rate>

    <ray>
      <scan>
        <horizontal>
          <samples>1800</samples>     <!-- 0.2 degree horizontal resolution -->
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
        <vertical>
          <samples>16</samples>       <!-- 16 vertical channels -->
          <resolution>1</resolution>
          <min_angle>-0.2618</min_angle>  <!-- -15 degrees -->
          <max_angle>0.2618</max_angle>   <!-- +15 degrees -->
        </vertical>
      </scan>
      <range>
        <min>0.5</min>
        <max>100.0</max>
        <resolution>0.01</resolution>
      </range>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.02</stddev>
      </noise>
    </ray>

    <plugin name="lidar_3d_plugin" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/out:=/robot/points</remapping>
      </ros>
      <output_type>sensor_msgs/PointCloud2</output_type>
      <frame_name>lidar_3d_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### LiDAR Parameter Guide

| Sensor Type | Samples | Range | Update Rate | Use Case |
|-------------|---------|-------|-------------|----------|
| RPLIDAR A1 | 360 | 0.15-12m | 5.5 Hz | Budget indoor |
| Hokuyo UTM-30LX | 1081 | 0.1-30m | 40 Hz | High-speed outdoor |
| Velodyne VLP-16 | 1800×16 | 0.5-100m | 10-20 Hz | Autonomous vehicles |

---

## IMU Sensors

An IMU (Inertial Measurement Unit) measures:

- **Linear acceleration** (how fast is the robot speeding up or slowing down?)
- **Angular velocity** (how fast is the robot rotating?)

IMUs are essential for estimating robot orientation and are often fused with other sensors for localization.

### Understanding IMU Data

```
┌─────────────────────────────────────────────────────────────────┐
│                        IMU MEASUREMENTS                          │
│                                                                  │
│   LINEAR ACCELERATION (m/s²):       ANGULAR VELOCITY (rad/s):   │
│                                                                  │
│         Z (up)                            Z (yaw)                │
│         │                                 │ ↻                    │
│         │ Gravity!                        │                      │
│         │ (9.81 m/s²                      │                      │
│         │  when stationary)               │                      │
│         │                                 │                      │
│   Y ────┼──── X                     Y ────┼──── X                │
│        /│                                /│                      │
│       / │                               / │                      │
│                                                                  │
│   When robot is still:              When robot is still:         │
│   - accel_x ≈ 0                     - angular_x ≈ 0              │
│   - accel_y ≈ 0                     - angular_y ≈ 0              │
│   - accel_z ≈ 9.81 (gravity!)       - angular_z ≈ 0              │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### Adding an IMU in Gazebo

```xml
<!-- IMU sensor -->
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <update_rate>100.0</update_rate>  <!-- IMUs typically run at high rates -->

    <imu>
      <!-- Accelerometer noise -->
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0002</stddev>  <!-- rad/s -->
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0002</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0002</stddev>
          </noise>
        </z>
      </angular_velocity>

      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>  <!-- m/s² -->
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>

    <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/out:=/robot/imu</remapping>
      </ros>
      <initial_orientation_as_reference>false</initial_orientation_as_reference>
      <frame_name>imu_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### Reading IMU Data

```bash
# View IMU data
ros2 topic echo /robot/imu

# Example output:
# orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}  # Quaternion
# angular_velocity: {x: 0.001, y: -0.002, z: 0.0}  # rad/s
# linear_acceleration: {x: 0.05, y: 0.02, z: 9.78}  # m/s² (gravity in z)
```

---

## Noise Models: Making Sensors Realistic

Real sensors have noise. Without noise in simulation, your algorithms may fail on real hardware.

### Types of Sensor Noise

| Noise Type | Description | Example |
|------------|-------------|---------|
| **Gaussian** | Random variation around true value | Most common |
| **Bias** | Constant offset from true value | IMU drift |
| **Quantization** | Limited resolution | Cheap sensors |
| **Dropout** | Missing readings | LiDAR on reflective surfaces |

### Configuring Gaussian Noise

Gaussian noise adds a random value from a normal distribution:

```
actual_reading = true_value + noise
where noise ~ Normal(mean, stddev)
```

```xml
<noise>
  <type>gaussian</type>
  <mean>0.0</mean>       <!-- Usually 0 (no bias) -->
  <stddev>0.01</stddev>  <!-- Higher = more noise -->
</noise>
```

### Typical Noise Values

| Sensor | Parameter | Typical Noise |
|--------|-----------|---------------|
| Camera | Pixel intensity | stddev 0.005-0.02 |
| 2D LiDAR | Range | stddev 0.01-0.03 m |
| 3D LiDAR | Range | stddev 0.02-0.05 m |
| IMU gyroscope | Angular velocity | stddev 0.0001-0.001 rad/s |
| IMU accelerometer | Linear acceleration | stddev 0.01-0.05 m/s² |

:::tip Finding Real Noise Values
Check your sensor's datasheet for noise specifications. Look for "RMS noise" or "noise density" values.
:::

---

## Complete Multi-Sensor Robot Example

Here is a complete example showing a robot with camera, lidar, and IMU sensors:

```xml
<?xml version="1.0"?>
<!-- File: multi_sensor_robot.urdf.xacro -->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="sensor_robot">

  <!-- Robot base -->
  <link name="base_link">
    <visual>
      <geometry><box size="0.4 0.3 0.15"/></geometry>
      <material name="blue"><color rgba="0.2 0.2 0.8 1"/></material>
    </visual>
    <collision>
      <geometry><box size="0.4 0.3 0.15"/></geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.05" iyy="0.05" izz="0.05" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <!-- Camera mount -->
  <link name="camera_link">
    <visual>
      <geometry><box size="0.02 0.04 0.02"/></geometry>
      <material name="black"><color rgba="0.1 0.1 0.1 1"/></material>
    </visual>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
  </joint>

  <!-- Lidar mount -->
  <link name="lidar_link">
    <visual>
      <geometry><cylinder radius="0.03" length="0.04"/></geometry>
      <material name="gray"><color rgba="0.5 0.5 0.5 1"/></material>
    </visual>
  </link>

  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
  </joint>

  <!-- IMU (inside base) -->
  <link name="imu_link"/>

  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

  <!-- Gazebo sensor plugins (add to separate .gazebo file or inline) -->

</robot>
```

---

## Hands-On Exercise

### Exercise 1: Build a Multi-Sensor Robot

**Objective:** Create a robot with camera, lidar, and IMU sensors and verify all sensors publish data correctly.

**Difficulty:** Intermediate

**Instructions:**

1. Create a URDF with links for camera, lidar, and IMU
2. Add Gazebo sensor plugins for each sensor
3. Launch in Gazebo and verify topics are published
4. Visualize camera and lidar in RViz
5. Record sensor data using `ros2 bag`

**Verification Checklist:**

```bash
# Check all sensor topics exist
ros2 topic list | grep -E "(camera|scan|imu)"
# Expected:
# /robot/camera/image_raw
# /robot/scan
# /robot/imu

# Verify camera publishes images
ros2 topic hz /robot/camera/image_raw
# Expected: ~30 Hz

# Verify lidar publishes scans
ros2 topic hz /robot/scan
# Expected: ~10 Hz

# Verify IMU publishes data
ros2 topic hz /robot/imu
# Expected: ~100 Hz

# View in RViz
ros2 run rviz2 rviz2
# Add LaserScan, Image, and TF displays
```

---

## Common Sensor Simulation Issues

### Issue 1: Sensor Data Not Publishing

**Cause:** Plugin not loaded or wrong topic name.

**Solution:**
```bash
# Check if plugin is loaded
ros2 node list  # Should show sensor nodes

# Check actual topic names
ros2 topic list

# Verify URDF loaded correctly
ros2 param get /robot_state_publisher robot_description
```

### Issue 2: LiDAR Shows No Obstacles

**Cause:** Obstacles lack collision geometry or are outside range.

**Solution:** Ensure all obstacles have `<collision>` elements in their URDF/SDF.

### Issue 3: Camera Image is Black

**Cause:** No lighting in the world or camera clipping planes wrong.

**Solution:**
1. Add a light to your world
2. Check `<near>` and `<far>` clip planes include the scene

### Issue 4: IMU Shows Zero Acceleration

**Cause:** IMU not properly attached to moving body.

**Solution:** Verify IMU link is connected via a fixed joint to a moving link.

---

## Summary

### Key Takeaways

1. **Sensors are how robots perceive the world**. Without sensors, a robot cannot see, measure distances, or know its orientation.

2. **Camera sensors provide images** for object detection and visual navigation. Configure resolution, FOV, and add noise for realism.

3. **LiDAR sensors measure distances** in 2D (laser scanner) or 3D (point cloud). Essential for SLAM and obstacle avoidance.

4. **IMU sensors measure acceleration and rotation**. Critical for orientation estimation and sensor fusion.

5. **Noise models are essential**. Algorithms trained on perfect data often fail on real, noisy sensors. Always add realistic noise to simulated sensors.

### Sensor Quick Reference

```xml
<!-- Camera -->
<sensor type="camera">
  <camera>
    <image><width>640</width><height>480</height></image>
    <noise type="gaussian"><stddev>0.007</stddev></noise>
  </camera>
</sensor>

<!-- 2D LiDAR -->
<sensor type="ray">
  <ray>
    <scan><horizontal><samples>360</samples></horizontal></scan>
    <range><min>0.1</min><max>12</max></range>
    <noise type="gaussian"><stddev>0.01</stddev></noise>
  </ray>
</sensor>

<!-- IMU -->
<sensor type="imu">
  <imu>
    <angular_velocity><noise><stddev>0.0002</stddev></noise></angular_velocity>
    <linear_acceleration><noise><stddev>0.017</stddev></noise></linear_acceleration>
  </imu>
</sensor>
```

---

## Module 2 Complete!

Congratulations! You have completed **Module 2: The Digital Twin**. You now have the skills to:

- **Launch Gazebo simulations** with custom worlds and robots
- **Configure physics engines** for realistic dynamics
- **Use Unity** for photo-realistic environments
- **Simulate sensors** with realistic noise models

Your robots now have virtual bodies, physics, and senses. In Module 3, you will give them **intelligence** using NVIDIA Isaac for GPU-accelerated perception and autonomous navigation.

---

## What's Next

Continue to **Module 3: The AI-Robot Brain** where you will:

- Set up NVIDIA Isaac Sim for advanced simulation
- Integrate Isaac ROS packages for GPU-accelerated perception
- Implement Visual SLAM for mapping and localization
- Deploy the Nav2 stack for autonomous navigation

[Next: Module 3 - The AI-Robot Brain →](/docs/module-3-nvidia-isaac)

---

## Additional Resources

- [Gazebo Sensor Plugins](https://gazebosim.org/docs/fortress/sensors) - Official sensor documentation
- [ROS 2 Sensor Messages](https://docs.ros.org/en/humble/p/sensor_msgs/) - Message type reference
- [Sensor Noise Modeling](http://wiki.ros.org/hector_gazebo_plugins) - Advanced noise models
- [Camera Calibration](http://wiki.ros.org/camera_calibration) - Calibrating real and simulated cameras
