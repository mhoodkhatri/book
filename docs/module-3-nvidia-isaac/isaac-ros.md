---
sidebar_position: 3
title: "Chapter 2: Isaac ROS"
description: "Integrate Isaac ROS packages for GPU-accelerated perception in ROS 2. Leverage NVIDIA's optimized robotics algorithms for real-time performance."
keywords: [Isaac ROS, GPU, perception, acceleration, NVIDIA, ROS 2, NITROS, DNN, object detection, depth estimation]
---

# Chapter 2: Isaac ROS

## Chapter Overview

In Chapter 1, you set up photorealistic simulation with Isaac Sim. Now you need to process that visual data—detect objects, estimate depth, and understand scenes. The problem? Standard ROS 2 perception runs on CPU, which is often too slow for real-time robotics.

**Isaac ROS** solves this. It is a collection of GPU-accelerated ROS 2 packages that provide 10x or more speedup over CPU implementations. The same perception code that struggles at 5 FPS on CPU can run at 30+ FPS on an NVIDIA GPU.

Think of Isaac ROS as "drop-in replacements" for common perception tasks. Your existing ROS 2 code still works—you just get faster results.

In this chapter, you will learn to set up Isaac ROS, run GPU-accelerated perception pipelines, and integrate them with your robot system.

**Prerequisites**: Completed [Chapter 1: Isaac Sim](/docs/module-3-nvidia-isaac/isaac-sim), familiarity with Docker

**What You Will Learn**: Install Isaac ROS, accelerate perception with GPU, run object detection and depth estimation

---

## Learning Objectives

After completing this chapter, you will be able to:

- Explain what NITROS is and why it accelerates ROS 2
- Set up Isaac ROS using Docker containers
- Run GPU-accelerated image processing pipelines
- Implement real-time object detection using DNN inference
- Configure stereo depth estimation
- Integrate Isaac ROS packages with your robot stack

---

## What is Isaac ROS?

Isaac ROS is NVIDIA's collection of hardware-accelerated ROS 2 packages. Here is what makes it special:

### The Performance Problem

Traditional ROS 2 perception has a bottleneck:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                     TRADITIONAL ROS 2 PERCEPTION                             │
│                                                                              │
│   Camera ──▶ CPU Copy ──▶ ROS 2 Message ──▶ CPU Processing ──▶ Result       │
│                 │                                   │                        │
│                 │         Memory copies            │                        │
│                 │         at every step            │                        │
│                 ▼                                   ▼                        │
│            [GPU Memory]                       [GPU Memory]                   │
│             (unused)                           (unused)                      │
│                                                                              │
│   Typical throughput: 5-10 FPS for object detection                         │
└─────────────────────────────────────────────────────────────────────────────┘
```

Even with a powerful GPU, data must be copied from GPU to CPU for ROS 2 message passing, then back to GPU for processing. These memory copies dominate processing time.

### NITROS: The Solution

Isaac ROS uses **NITROS** (NVIDIA Isaac Transport for ROS) to eliminate these copies:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        ISAAC ROS WITH NITROS                                 │
│                                                                              │
│   Camera ──▶ GPU Memory ──▶ NITROS Message ──▶ GPU Processing ──▶ Result    │
│                  │              │                    │                       │
│                  └──────────────┴────────────────────┘                       │
│                       Data stays in GPU memory                               │
│                       Zero copies between nodes                              │
│                                                                              │
│   Typical throughput: 30+ FPS for object detection                          │
└─────────────────────────────────────────────────────────────────────────────┘
```

NITROS keeps data in GPU memory throughout the pipeline. When NITROS nodes connect, they share GPU memory pointers instead of copying data. This is transparent to your code—standard ROS 2 interfaces still work.

### Isaac ROS Package Categories

Isaac ROS includes packages for:

| Category | Packages | Description |
|----------|----------|-------------|
| **DNN Inference** | `isaac_ros_dnn_inference` | Run neural networks on GPU |
| **Object Detection** | `isaac_ros_detectnet`, `isaac_ros_yolov8` | Detect and classify objects |
| **Segmentation** | `isaac_ros_unet`, `isaac_ros_segformer` | Semantic/instance segmentation |
| **Depth** | `isaac_ros_ess`, `isaac_ros_dnn_stereo_depth` | Stereo depth estimation |
| **Pose Estimation** | `isaac_ros_dope`, `isaac_ros_centerpose` | 6-DOF object poses |
| **SLAM** | `isaac_ros_visual_slam` | Visual localization |
| **Navigation** | `isaac_ros_nvblox` | 3D reconstruction for navigation |
| **Image Processing** | `isaac_ros_image_pipeline` | GPU-accelerated image ops |
| **AprilTag** | `isaac_ros_apriltag` | Fiducial marker detection |

---

## Setting Up Isaac ROS

Isaac ROS uses Docker containers for easy deployment and reproducibility.

### Why Docker?

Isaac ROS has complex dependencies (CUDA, TensorRT, specific library versions). Docker ensures you get exactly the right environment without conflicting with your system.

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          YOUR HOST SYSTEM                                    │
│                                                                              │
│   ┌─────────────────────────────────────────────────────────────────────┐   │
│   │                    ISAAC ROS DOCKER CONTAINER                        │   │
│   │                                                                      │   │
│   │   Ubuntu 22.04 + ROS 2 Humble + CUDA 12 + TensorRT + Isaac ROS      │   │
│   │                                                                      │   │
│   │   • Consistent environment                                          │   │
│   │   • Pre-built packages                                              │   │
│   │   • GPU passthrough with nvidia-docker                              │   │
│   │                                                                      │   │
│   └─────────────────────────────────────────────────────────────────────┘   │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Step 1: Install Prerequisites

```bash
# Install Docker if not already installed
sudo apt update
sudo apt install docker.io -y
sudo usermod -aG docker $USER
# Log out and back in for group change to take effect

# Install NVIDIA Container Toolkit
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/libnvidia-container/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt update
sudo apt install nvidia-container-toolkit -y
sudo systemctl restart docker

# Verify GPU access in Docker
docker run --rm --gpus all nvidia/cuda:12.0-base nvidia-smi
```

### Step 2: Clone Isaac ROS Repositories

```bash
# Create workspace
mkdir -p ~/isaac_ros_ws/src
cd ~/isaac_ros_ws/src

# Clone Isaac ROS common (required by all packages)
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git

# Clone packages you need
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_object_detection.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_inference.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_depth_segmentation.git
```

### Step 3: Build and Launch the Container

Isaac ROS provides a script to build and run the development container:

```bash
cd ~/isaac_ros_ws/src/isaac_ros_common

# Run the development container
# This builds the container on first run (takes 10-20 minutes)
./scripts/run_dev.sh
```

Inside the container, build the workspace:

```bash
# Inside Docker container
cd /workspaces/isaac_ros-dev

# Build Isaac ROS packages
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

### Step 4: Verify Installation

```bash
# Inside container, check packages are available
ros2 pkg list | grep isaac_ros

# Expected output includes:
# isaac_ros_apriltag
# isaac_ros_common
# isaac_ros_dnn_inference
# isaac_ros_image_pipeline
# ...
```

---

## GPU-Accelerated Image Processing

Let us start with basic image processing—the foundation of any perception pipeline.

### The Image Pipeline

Isaac ROS `image_pipeline` provides GPU-accelerated versions of common image operations:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    ISAAC ROS IMAGE PIPELINE                                  │
│                                                                              │
│   Raw Image ──▶ Debayer ──▶ Rectify ──▶ Resize ──▶ Color Convert ──▶ Output │
│       │            │           │           │             │                   │
│       └────────────┴───────────┴───────────┴─────────────┘                   │
│                      All operations on GPU                                   │
│                      NITROS zero-copy between stages                         │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Running the Rectify Node

Image rectification removes lens distortion. Here is how to run it with GPU acceleration:

```bash
# Launch rectify node
ros2 launch isaac_ros_image_pipeline isaac_ros_image_rectify.launch.py \
    image_input:=/camera/image_raw \
    camera_info_input:=/camera/camera_info
```

### Python Launch File for Image Pipeline

```python
# File: image_pipeline.launch.py
# GPU-accelerated image processing pipeline

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Launch GPU-accelerated image pipeline."""

    # Container for composable nodes (enables zero-copy)
    image_pipeline_container = ComposableNodeContainer(
        name='image_pipeline_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',  # Multi-threaded
        composable_node_descriptions=[
            # Rectification node
            ComposableNode(
                package='isaac_ros_image_pipeline',
                plugin='nvidia::isaac_ros::image_pipeline::RectifyNode',
                name='rectify_node',
                remappings=[
                    ('image_raw', '/camera/image_raw'),
                    ('camera_info', '/camera/camera_info'),
                    ('image_rect', '/camera/image_rect')
                ],
                parameters=[{
                    'output_width': 640,
                    'output_height': 480
                }]
            ),
            # Resize node
            ComposableNode(
                package='isaac_ros_image_pipeline',
                plugin='nvidia::isaac_ros::image_pipeline::ResizeNode',
                name='resize_node',
                remappings=[
                    ('image', '/camera/image_rect'),
                    ('camera_info', '/camera/camera_info'),
                    ('resize/image', '/camera/image_resized'),
                    ('resize/camera_info', '/camera/camera_info_resized')
                ],
                parameters=[{
                    'output_width': 320,
                    'output_height': 240,
                    'keep_aspect_ratio': True
                }]
            )
        ],
        output='screen'
    )

    return LaunchDescription([image_pipeline_container])
```

### Verifying GPU Acceleration

```bash
# Check that GPU is being used
nvidia-smi

# Look for isaac_ros processes using GPU memory
# You should see memory allocated

# Compare frame rates
# CPU rectification: ~30 FPS for 640x480
# GPU rectification: ~200+ FPS for 640x480
ros2 topic hz /camera/image_rect
```

---

## Object Detection with Isaac ROS

Object detection is a core perception task. Isaac ROS provides GPU-accelerated detection using various neural networks.

### Detection Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    OBJECT DETECTION PIPELINE                                 │
│                                                                              │
│   Camera ──▶ Preprocessing ──▶ DNN Inference ──▶ Postprocessing ──▶ Detections
│      │             │                │                  │               │     │
│      │         GPU-based        TensorRT            NMS/Decode     2D BBs    │
│      │         resize,          optimized           on GPU         + labels  │
│      │         normalize        inference                                    │
│      │                                                                       │
│   Input: sensor_msgs/Image                  Output: vision_msgs/Detection2DArray
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Setting Up DetectNet

DetectNet is NVIDIA's object detection network. Let us run it:

```bash
# Download pre-trained model (inside container)
cd /workspaces/isaac_ros-dev

# Create models directory
mkdir -p /tmp/models

# Download PeopleNet model (detects people, faces, bags)
wget -O /tmp/models/peoplenet.etlt \
    https://api.ngc.nvidia.com/v2/models/nvidia/tao/peoplenet/versions/pruned_quantized_decrypted_v2.3.3/files/resnet34_peoplenet_int8.etlt
```

### DetectNet Launch File

```python
# File: detectnet.launch.py
# GPU-accelerated object detection with DetectNet

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Launch DetectNet for object detection."""

    # Launch arguments
    model_file_path = LaunchConfiguration('model_file_path')
    input_topic = LaunchConfiguration('input_topic')

    detectnet_container = ComposableNodeContainer(
        name='detectnet_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # Encoder node (preprocessing)
            ComposableNode(
                package='isaac_ros_dnn_image_encoder',
                plugin='nvidia::isaac_ros::dnn_inference::DnnImageEncoderNode',
                name='dnn_image_encoder',
                remappings=[
                    ('image', '/camera/image_rect'),
                    ('encoded_tensor', '/tensor_pub')
                ],
                parameters=[{
                    'input_image_width': 640,
                    'input_image_height': 480,
                    'network_image_width': 960,
                    'network_image_height': 544,
                    'image_mean': [0.0, 0.0, 0.0],
                    'image_stddev': [1.0, 1.0, 1.0]
                }]
            ),
            # TensorRT inference node
            ComposableNode(
                package='isaac_ros_tensor_rt',
                plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
                name='tensor_rt',
                parameters=[{
                    'model_file_path': '/tmp/models/peoplenet.etlt',
                    'engine_file_path': '/tmp/models/peoplenet.engine',
                    'input_tensor_names': ['input_1'],
                    'input_binding_names': ['input_1'],
                    'output_tensor_names': ['output_cov/Sigmoid', 'output_bbox/BiasAdd'],
                    'output_binding_names': ['output_cov/Sigmoid', 'output_bbox/BiasAdd'],
                    'verbose': False,
                    'force_engine_update': False
                }]
            ),
            # DetectNet decoder (postprocessing)
            ComposableNode(
                package='isaac_ros_detectnet',
                plugin='nvidia::isaac_ros::detectnet::DetectNetDecoderNode',
                name='detectnet_decoder',
                parameters=[{
                    'label_list': ['person', 'bag', 'face'],
                    'confidence_threshold': 0.5,
                    'nms_threshold': 0.45,
                    'enable_tf_publishing': False
                }]
            )
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'model_file_path',
            default_value='/tmp/models/peoplenet.etlt',
            description='Path to detection model'
        ),
        DeclareLaunchArgument(
            'input_topic',
            default_value='/camera/image_rect',
            description='Input image topic'
        ),
        detectnet_container
    ])
```

### Running Detection

```bash
# Launch the detection pipeline
ros2 launch my_robot detectnet.launch.py

# In another terminal, check detections
ros2 topic echo /detectnet/detections

# Visualize detections
ros2 run rqt_image_view rqt_image_view --ros-args -r image:=/detectnet/image_detections
```

### Detection Output Format

The detections are published as `vision_msgs/Detection2DArray`:

```python
# Example detection message structure
# vision_msgs/Detection2DArray

header:
  stamp: {sec: 1234567, nanosec: 890}
  frame_id: "camera_link"
detections:
  - results:
      - hypothesis:
          class_id: "person"
          score: 0.95
    bbox:
      center:
        position: {x: 320.0, y: 240.0}
      size_x: 100.0
      size_y: 200.0
  - results:
      - hypothesis:
          class_id: "bag"
          score: 0.87
    bbox:
      center:
        position: {x: 150.0, y: 300.0}
      size_x: 50.0
      size_y: 60.0
```

---

## Stereo Depth Estimation

Depth perception is critical for obstacle avoidance and manipulation. Isaac ROS provides GPU-accelerated stereo depth estimation.

### ESS (Efficient Stereo Sisparity)

ESS is NVIDIA's deep learning stereo depth network:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    ESS STEREO DEPTH PIPELINE                                 │
│                                                                              │
│   Left Camera ──┐                                                            │
│                 ├──▶ ESS Network ──▶ Disparity ──▶ Point Cloud              │
│   Right Camera ─┘      (GPU)           Map                                   │
│                                                                              │
│   • Deep learning-based (more accurate than traditional stereo)             │
│   • Handles textureless regions                                             │
│   • Real-time on RTX GPUs                                                   │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### ESS Launch Configuration

```python
# File: ess_depth.launch.py
# Stereo depth estimation with ESS

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Launch ESS stereo depth estimation."""

    ess_container = ComposableNodeContainer(
        name='ess_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # ESS disparity node
            ComposableNode(
                package='isaac_ros_ess',
                plugin='nvidia::isaac_ros::dnn_stereo_depth::ESSDisparityNode',
                name='ess_disparity',
                remappings=[
                    ('left/image_rect', '/stereo/left/image_rect'),
                    ('right/image_rect', '/stereo/right/image_rect'),
                    ('left/camera_info', '/stereo/left/camera_info'),
                    ('right/camera_info', '/stereo/right/camera_info')
                ],
                parameters=[{
                    'engine_file_path': '/tmp/models/ess.engine',
                    'threshold': 0.35
                }]
            ),
            # Disparity to depth conversion
            ComposableNode(
                package='isaac_ros_stereo_image_proc',
                plugin='nvidia::isaac_ros::stereo_image_proc::DisparityToDepthNode',
                name='disparity_to_depth',
                remappings=[
                    ('disparity', '/ess/disparity'),
                    ('depth', '/stereo/depth')
                ]
            ),
            # Point cloud generation
            ComposableNode(
                package='isaac_ros_stereo_image_proc',
                plugin='nvidia::isaac_ros::stereo_image_proc::PointCloudNode',
                name='point_cloud',
                remappings=[
                    ('left/image_rect_color', '/stereo/left/image_rect'),
                    ('left/camera_info', '/stereo/left/camera_info'),
                    ('right/camera_info', '/stereo/right/camera_info'),
                    ('disparity', '/ess/disparity'),
                    ('points2', '/stereo/points')
                ],
                parameters=[{
                    'use_color': True,
                    'queue_size': 10
                }]
            )
        ],
        output='screen'
    )

    return LaunchDescription([ess_container])
```

### Visualizing Depth

```bash
# View depth image
ros2 run rqt_image_view rqt_image_view --ros-args -r image:=/stereo/depth

# View point cloud in RViz2
rviz2

# In RViz2:
# 1. Add PointCloud2 display
# 2. Set topic to /stereo/points
# 3. Set Fixed Frame to "camera_link"
```

---

## AprilTag Detection

AprilTags are fiducial markers used for localization and object identification. Isaac ROS provides GPU-accelerated detection.

### AprilTag Pipeline

```python
# File: apriltag.launch.py
# GPU-accelerated AprilTag detection

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Launch AprilTag detection."""

    apriltag_container = ComposableNodeContainer(
        name='apriltag_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_apriltag',
                plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
                name='apriltag',
                remappings=[
                    ('image', '/camera/image_rect'),
                    ('camera_info', '/camera/camera_info')
                ],
                parameters=[{
                    'size': 0.162,  # Tag size in meters
                    'max_tags': 64,
                    'tile_size': 4
                }]
            )
        ],
        output='screen'
    )

    return LaunchDescription([apriltag_container])
```

### AprilTag Output

```bash
# Run detection
ros2 launch my_robot apriltag.launch.py

# View detections (includes 3D pose)
ros2 topic echo /tag_detections

# Example output:
# detections:
#   - id: [5]
#     size: [0.162]
#     pose:
#       pose:
#         position: {x: 1.2, y: 0.1, z: 0.5}
#         orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
```

AprilTag detection is useful for:
- Robot localization (known tag positions)
- Object identification (tags on boxes)
- Calibration (camera-to-robot calibration)

---

## Integrating Isaac ROS with Your Robot

Let us put it all together in a complete perception pipeline.

### Complete Perception Launch File

```python
# File: perception_pipeline.launch.py
# Complete Isaac ROS perception pipeline

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """Launch complete perception pipeline."""

    perception_container = ComposableNodeContainer(
        name='perception_container',
        namespace='perception',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # Image rectification
            ComposableNode(
                package='isaac_ros_image_pipeline',
                plugin='nvidia::isaac_ros::image_pipeline::RectifyNode',
                name='rectify',
                namespace='perception',
                remappings=[
                    ('image_raw', '/camera/image_raw'),
                    ('camera_info', '/camera/camera_info'),
                    ('image_rect', '/perception/image_rect')
                ]
            ),
            # Object detection
            ComposableNode(
                package='isaac_ros_dnn_image_encoder',
                plugin='nvidia::isaac_ros::dnn_inference::DnnImageEncoderNode',
                name='encoder',
                namespace='perception',
                remappings=[
                    ('image', '/perception/image_rect'),
                    ('encoded_tensor', '/perception/tensor')
                ],
                parameters=[{
                    'network_image_width': 960,
                    'network_image_height': 544
                }]
            ),
            ComposableNode(
                package='isaac_ros_tensor_rt',
                plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
                name='tensorrt',
                namespace='perception',
                parameters=[{
                    'model_file_path': '/tmp/models/detectnet.onnx',
                    'engine_file_path': '/tmp/models/detectnet.engine'
                }]
            ),
            ComposableNode(
                package='isaac_ros_detectnet',
                plugin='nvidia::isaac_ros::detectnet::DetectNetDecoderNode',
                name='detector',
                namespace='perception',
                parameters=[{
                    'label_list': ['person', 'vehicle', 'obstacle'],
                    'confidence_threshold': 0.6
                }]
            ),
            # AprilTag detection
            ComposableNode(
                package='isaac_ros_apriltag',
                plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
                name='apriltag',
                namespace='perception',
                remappings=[
                    ('image', '/perception/image_rect'),
                    ('camera_info', '/camera/camera_info')
                ],
                parameters=[{
                    'size': 0.162,
                    'max_tags': 16
                }]
            )
        ],
        output='screen'
    )

    return LaunchDescription([perception_container])
```

### Processing Detections in Your Node

```python
# File: perception_processor.py
# Process detections from Isaac ROS

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist

class PerceptionProcessor(Node):
    """Process perception data and generate robot behaviors."""

    def __init__(self):
        super().__init__('perception_processor')

        # Subscribers
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/perception/detections',
            self.detection_callback,
            10
        )

        self.apriltag_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/perception/tag_detections',
            self.apriltag_callback,
            10
        )

        # Publisher for robot control
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('Perception processor initialized')

    def detection_callback(self, msg: Detection2DArray):
        """Handle object detections."""
        for detection in msg.detections:
            for result in detection.results:
                class_id = result.hypothesis.class_id
                score = result.hypothesis.score
                bbox = detection.bbox

                self.get_logger().info(
                    f'Detected {class_id} (conf: {score:.2f}) '
                    f'at ({bbox.center.position.x:.0f}, {bbox.center.position.y:.0f})'
                )

                # React to detections
                if class_id == 'person' and score > 0.8:
                    self.avoid_person(bbox)

    def apriltag_callback(self, msg: AprilTagDetectionArray):
        """Handle AprilTag detections."""
        for detection in msg.detections:
            tag_id = detection.id[0]
            pose = detection.pose.pose.pose

            self.get_logger().info(
                f'AprilTag {tag_id} at position '
                f'({pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f})'
            )

            # Navigate to tag
            if tag_id == 5:  # Target tag
                self.navigate_to_tag(pose)

    def avoid_person(self, bbox):
        """Stop or navigate around detected person."""
        cmd = Twist()

        # Calculate avoidance based on person position
        image_center_x = 320  # Assuming 640px width
        person_x = bbox.center.position.x

        if abs(person_x - image_center_x) < 50:
            # Person is centered - stop
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().warn('Person detected ahead - stopping')
        else:
            # Turn away from person
            cmd.linear.x = 0.1
            cmd.angular.z = 0.5 if person_x > image_center_x else -0.5

        self.cmd_vel_pub.publish(cmd)

    def navigate_to_tag(self, pose):
        """Navigate toward detected AprilTag."""
        cmd = Twist()

        # Simple proportional control to tag
        cmd.linear.x = min(0.3, pose.position.z * 0.2)  # Forward based on distance
        cmd.angular.z = -pose.position.x * 0.5  # Turn based on lateral offset

        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Hands-On Exercise

### Exercise: Build a Person-Following Robot

**Objective:** Use Isaac ROS object detection to make your robot follow a person.

**Difficulty:** Intermediate-Advanced

**Instructions:**

1. Set up Isaac ROS DetectNet with the PeopleNet model
2. Create a node that processes detections
3. Implement following behavior: move toward detected person, maintain distance
4. Test in Isaac Sim

**Starter Code:**

```python
# File: person_follower.py
# Person-following robot using Isaac ROS

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Twist

class PersonFollower(Node):
    def __init__(self):
        super().__init__('person_follower')

        # TODO: Create subscriber for detections
        self.detection_sub = None

        # TODO: Create publisher for velocity commands
        self.cmd_vel_pub = None

        # Parameters
        self.target_distance = 1.5  # meters (approximate based on bbox size)
        self.max_linear_vel = 0.5
        self.max_angular_vel = 1.0

    def detection_callback(self, msg: Detection2DArray):
        # TODO: Find person with highest confidence
        best_person = None

        # TODO: Calculate control commands
        # - Angular: Turn to center person in frame
        # - Linear: Move to maintain target distance

        # TODO: Publish velocity command
        pass


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

<details>
<summary>Click to reveal solution</summary>

```python
# File: person_follower_solution.py
# Person-following robot using Isaac ROS

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Twist

class PersonFollower(Node):
    def __init__(self):
        super().__init__('person_follower')

        # Subscriber for detections
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/detectnet/detections',
            self.detection_callback,
            10
        )

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Parameters
        self.image_width = 640
        self.image_height = 480
        self.target_bbox_height = 200  # Target person height in pixels
        self.max_linear_vel = 0.5
        self.max_angular_vel = 1.0

        # PID-like gains
        self.kp_angular = 0.005  # Proportional gain for angular
        self.kp_linear = 0.003   # Proportional gain for linear

        self.get_logger().info('Person follower initialized')

    def detection_callback(self, msg: Detection2DArray):
        cmd = Twist()

        # Find person with highest confidence
        best_person = None
        best_score = 0.0

        for detection in msg.detections:
            for result in detection.results:
                if result.hypothesis.class_id == 'person':
                    if result.hypothesis.score > best_score:
                        best_score = result.hypothesis.score
                        best_person = detection

        if best_person is None:
            # No person detected - rotate to search
            cmd.linear.x = 0.0
            cmd.angular.z = 0.3  # Slow rotation to search
            self.get_logger().info('Searching for person...')
        else:
            bbox = best_person.bbox

            # Calculate error from image center (for turning)
            center_x = bbox.center.position.x
            error_x = center_x - (self.image_width / 2)

            # Calculate error from target size (for distance)
            bbox_height = bbox.size_y
            error_distance = self.target_bbox_height - bbox_height

            # Angular control (turn to center person)
            cmd.angular.z = -self.kp_angular * error_x
            cmd.angular.z = max(-self.max_angular_vel,
                               min(self.max_angular_vel, cmd.angular.z))

            # Linear control (move to maintain distance)
            if abs(error_x) < 100:  # Only move forward if roughly centered
                cmd.linear.x = self.kp_linear * error_distance
                cmd.linear.x = max(0, min(self.max_linear_vel, cmd.linear.x))
            else:
                cmd.linear.x = 0.0  # Turn first, then move

            self.get_logger().info(
                f'Following person: error_x={error_x:.0f}, '
                f'bbox_h={bbox_height:.0f}, '
                f'cmd=({cmd.linear.x:.2f}, {cmd.angular.z:.2f})'
            )

        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

</details>

**Verification:**

```bash
# Terminal 1: Launch Isaac Sim with robot (from Chapter 1)

# Terminal 2: Launch DetectNet
ros2 launch isaac_ros_detectnet detectnet.launch.py

# Terminal 3: Run person follower
ros2 run my_robot person_follower

# Terminal 4: Monitor behavior
ros2 topic echo /cmd_vel

# In Isaac Sim, add a person model to the scene
# The robot should turn to face and follow the person
```

---

## Common Issues and Solutions

### Issue 1: "No GPU Memory Available"

**Cause:** GPU memory exhausted by other processes or models.

**Solution:**
```bash
# Check GPU memory usage
nvidia-smi

# Kill other GPU processes if needed
# Or reduce model complexity

# Use smaller batch sizes in parameters
parameters=[{
    'max_batch_size': 1  # Reduce from default
}]
```

### Issue 2: TensorRT Engine Build Fails

**Cause:** Incompatible model format or CUDA version.

**Solution:**
```bash
# Inside container, check TensorRT version
dpkg -l | grep tensorrt

# Ensure model is compatible
# Re-export model with correct target if needed

# Delete old engine files to force rebuild
rm /tmp/models/*.engine
```

### Issue 3: NITROS Topics Not Connecting

**Cause:** Type negotiation failure between nodes.

**Solution:**
```bash
# Ensure all nodes are in same composable container
# Check that topics match exactly

# View NITROS status
ros2 topic info /your_topic --verbose
# Should show "negotiated" type
```

### Issue 4: Low Frame Rate Despite GPU

**Cause:** CPU bottleneck or memory copies.

**Solution:**
```bash
# Profile the pipeline
ros2 run isaac_ros_benchmark isaac_ros_benchmark

# Ensure all nodes use NITROS types
# Check for non-NITROS nodes in the pipeline
# Use composable nodes, not standalone
```

---

## Summary

### Key Takeaways

1. **Isaac ROS provides GPU-accelerated ROS 2 packages** for perception tasks. Performance improvements of 10x or more are common compared to CPU implementations.

2. **NITROS eliminates memory copies** by keeping data in GPU memory throughout the pipeline. Use composable nodes in containers to enable this.

3. **Docker containers simplify deployment**. Use the Isaac ROS development container for consistent environments.

4. **DetectNet provides real-time object detection**. Pre-trained models like PeopleNet work out of the box.

5. **ESS provides deep learning stereo depth**. More robust than traditional stereo matching, especially in textureless regions.

### Quick Reference

```bash
# Start Isaac ROS container
cd ~/isaac_ros_ws/src/isaac_ros_common
./scripts/run_dev.sh

# Build workspace (inside container)
cd /workspaces/isaac_ros-dev
colcon build --symlink-install
source install/setup.bash

# Check NITROS acceleration
ros2 topic info /topic --verbose
# Look for "negotiated" type

# Monitor GPU usage
watch -n 1 nvidia-smi
```

```python
# Essential imports for Isaac ROS launch files
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# Use composable container for NITROS acceleration
container = ComposableNodeContainer(
    name='my_container',
    package='rclcpp_components',
    executable='component_container_mt',  # Multi-threaded
    composable_node_descriptions=[
        # Add ComposableNode entries here
    ]
)
```

---

## What's Next

You can now process visual data in real-time using GPU acceleration. The next chapter introduces **Visual SLAM**—the ability to simultaneously build a map and track your robot's position using camera input alone. This is essential for autonomous navigation in environments without GPS.

[Next: Chapter 3 - Visual SLAM →](/docs/module-3-nvidia-isaac/vslam)

---

## Additional Resources

- [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/) - Official documentation
- [NITROS Overview](https://nvidia-isaac-ros.github.io/concepts/nitros/index.html) - Technical details on NITROS
- [NGC Model Catalog](https://catalog.ngc.nvidia.com/) - Pre-trained models
- [Isaac ROS Examples](https://github.com/NVIDIA-ISAAC-ROS) - GitHub repositories
