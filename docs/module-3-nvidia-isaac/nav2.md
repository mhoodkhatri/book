---
sidebar_position: 5
title: "Chapter 4: Nav2 Integration"
description: "Deploy Nav2 for autonomous robot navigation. Configure path planning, obstacle avoidance, and goal-seeking behavior with ROS 2 Navigation stack."
keywords: [Nav2, navigation, path planning, obstacle avoidance, ROS 2, autonomous, costmap, behavior tree, controller]
---

# Chapter 4: Nav2 Integration

## Chapter Overview

Throughout this module, you have given your robot eyes (Isaac ROS perception) and a sense of location (Visual SLAM). Now it is time for the final piece: **autonomous navigation**.

Navigation is the ability to move from point A to point B while avoiding obstacles. It sounds simple, but involves complex decisions:
- What path should the robot take?
- How should it follow that path?
- What happens when an obstacle appears?
- How does it recover from unexpected situations?

**Nav2** (Navigation 2) is the ROS 2 navigation stack that handles all of this. It is a modular framework where you configure planners, controllers, and behaviors to match your robot's needs.

In this chapter, you will configure Nav2 to work with your Isaac perception and VSLAM localization, creating a fully autonomous robot.

**Prerequisites**: Completed [Chapter 3: Visual SLAM](/docs/module-3-nvidia-isaac/vslam), understanding of ROS 2 lifecycle nodes

**What You Will Learn**: Nav2 architecture, costmaps, planners, controllers, behavior trees, and complete integration

---

## Learning Objectives

After completing this chapter, you will be able to:

- Explain the Nav2 architecture and component interactions
- Configure global and local costmaps for obstacle representation
- Select and tune path planners for your environment
- Configure controllers for smooth path following
- Create behavior trees for complex navigation tasks
- Integrate Nav2 with Isaac perception and VSLAM
- Handle navigation failures with recovery behaviors

---

## Nav2 Architecture

Nav2 is built on a modular architecture where each component has a specific role:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                            NAV2 ARCHITECTURE                                 │
│                                                                              │
│   ┌─────────────────────────────────────────────────────────────────────┐   │
│   │                      BEHAVIOR TREE (BT)                              │   │
│   │              Orchestrates the navigation process                     │   │
│   └───────────────────────────┬─────────────────────────────────────────┘   │
│                               │                                              │
│           ┌───────────────────┼───────────────────┐                         │
│           │                   │                   │                         │
│           ▼                   ▼                   ▼                         │
│   ┌───────────────┐   ┌───────────────┐   ┌───────────────┐                │
│   │    Planner    │   │  Controller   │   │   Recoveries  │                │
│   │    Server     │   │    Server     │   │    Server     │                │
│   │               │   │               │   │               │                │
│   │ Global path   │   │ Follow path   │   │ Handle stuck  │                │
│   │ planning      │   │ locally       │   │ situations    │                │
│   └───────┬───────┘   └───────┬───────┘   └───────────────┘                │
│           │                   │                                              │
│           └─────────┬─────────┘                                              │
│                     │                                                        │
│                     ▼                                                        │
│   ┌─────────────────────────────────────────────────────────────────────┐   │
│   │                         COSTMAPS                                     │   │
│   │                                                                      │   │
│   │   ┌─────────────────┐         ┌─────────────────┐                   │   │
│   │   │  Global Costmap │         │  Local Costmap  │                   │   │
│   │   │  (entire map)   │         │  (around robot) │                   │   │
│   │   └─────────────────┘         └─────────────────┘                   │   │
│   │                                                                      │   │
│   └─────────────────────────────────────────────────────────────────────┘   │
│                     │                                                        │
│                     ▼                                                        │
│   ┌─────────────────────────────────────────────────────────────────────┐   │
│   │                      SENSOR INPUT                                    │   │
│   │         Lidar, Camera, Depth sensors, Odometry                       │   │
│   └─────────────────────────────────────────────────────────────────────┘   │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Key Components

| Component | Role | Input | Output |
|-----------|------|-------|--------|
| **Planner Server** | Find global path | Goal pose, costmap | Path (list of poses) |
| **Controller Server** | Follow path | Path, local costmap | Velocity commands |
| **Costmap** | Represent obstacles | Sensor data | Grid of costs |
| **Behavior Tree** | Orchestrate | Events | Actions |
| **Recovery Server** | Handle failures | Stuck state | Recovery actions |

---

## Costmaps: Representing the World

Costmaps are grid-based representations of the environment where each cell has a "cost" indicating how dangerous it is to occupy.

### Cost Values

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           COSTMAP VALUES                                     │
│                                                                              │
│   Value    Meaning                    Visual                                 │
│   ─────────────────────────────────────────────                              │
│   0        Free space                 ░░░░░░░░░                              │
│   1-252    Increasing cost            ▒▒▒▒▒▒▒▒▒                              │
│   253      Inscribed (robot touches)  ▓▓▓▓▓▓▓▓▓                              │
│   254      Lethal (occupied)          █████████                              │
│   255      Unknown                    ?????????                              │
│                                                                              │
│   Example costmap around obstacle:                                           │
│                                                                              │
│   ░░░░░▒▒▓█████▓▒▒░░░░░                                                     │
│   ░░░░▒▒▓█████████▓▒▒░░░                                                    │
│   ░░░▒▓███████████████▓▒░                                                   │
│   ░░▒▓█████[OBSTACLE]████▓░                                                 │
│   ░░░▒▓███████████████▓▒░                                                   │
│   ░░░░▒▒▓█████████▓▒▒░░░                                                    │
│   ░░░░░▒▒▓█████▓▒▒░░░░░                                                     │
│                                                                              │
│   The "inflation" around obstacles keeps robot from getting too close        │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Costmap Layers

Nav2 costmaps are built from multiple layers:

```yaml
# File: global_costmap.yaml
# Global costmap configuration

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      resolution: 0.05  # 5cm per cell

      # Layers stack on top of each other
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

      # Static layer: loads map from SLAM
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True

      # Obstacle layer: adds dynamic obstacles from sensors
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"

      # Inflation layer: adds cost gradient around obstacles
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55  # Robot radius + safety margin
```

### Local Costmap Configuration

```yaml
# File: local_costmap.yaml
# Local costmap for immediate obstacle avoidance

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0  # More frequent for reactive behavior
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05

      plugins: ["voxel_layer", "inflation_layer"]

      # Voxel layer: 3D obstacle representation
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: pointcloud
        pointcloud:
          topic: /stereo/points
          max_obstacle_height: 2.0
          min_obstacle_height: 0.0
          clearing: True
          marking: True
          data_type: "PointCloud2"

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
```

---

## Path Planning

The planner finds a path from current position to the goal. Nav2 includes several planners:

### Available Planners

| Planner | Type | Best For |
|---------|------|----------|
| **NavFn** | Dijkstra/A* | Simple environments |
| **Smac 2D** | State lattice | Ackermann vehicles |
| **Smac Hybrid-A*** | Hybrid A* | Cars, non-holonomic |
| **Smac Lattice** | State lattice | Custom motion primitives |
| **Theta*** | Any-angle | Smooth paths |

### Configuring NavFn Planner

```yaml
# File: planner_server.yaml
# Path planner configuration

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]

    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false  # Use Dijkstra (more thorough)
      allow_unknown: true
```

### Configuring Smac Hybrid-A* (for better paths)

```yaml
# File: planner_server.yaml
# Smac Hybrid-A* for smoother paths

planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]

    GridBased:
      plugin: "nav2_smac_planner/SmacPlannerHybrid"
      tolerance: 0.25
      downsample_costmap: false
      downsampling_factor: 1
      allow_unknown: true
      max_iterations: 1000000
      max_on_approach_iterations: 1000
      max_planning_time: 5.0

      # Motion model
      motion_model_for_search: "DUBIN"  # or "REEDS_SHEPP" for reversing
      minimum_turning_radius: 0.40  # Based on robot kinematics

      # Analytic expansion
      analytic_expansion_ratio: 3.5
      analytic_expansion_max_length: 3.0

      # Cost penalties
      cost_penalty: 2.0
      change_penalty: 0.0
      non_straight_penalty: 1.2
      reverse_penalty: 2.0
```

---

## Controllers: Following the Path

The controller converts a path into velocity commands that the robot can execute.

### Available Controllers

| Controller | Type | Best For |
|------------|------|----------|
| **DWB** | Dynamic Window | General mobile robots |
| **MPPI** | Model Predictive | Smooth, optimal control |
| **RPP** | Regulated Pure Pursuit | Simple, stable |
| **Rotation Shim** | Pre-rotation | Align before moving |

### DWB Controller Configuration

```yaml
# File: controller_server.yaml
# DWB controller configuration

controller_server:
  ros__parameters:
    controller_frequency: 20.0
    controller_plugins: ["FollowPath"]

    # Goal checker
    goal_checker_plugins: ["general_goal_checker"]
    general_goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True

    # Progress checker (detect if stuck)
    progress_checker_plugins: ["progress_checker"]
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    # DWB Controller
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: False
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.5
      max_vel_y: 0.0  # Differential drive: no lateral movement
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.5
      min_speed_theta: 0.0
      acc_lim_x: 2.5
      acc_lim_y: 0.0
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_y: 0.0
      decel_lim_theta: -3.2

      # Velocity samples
      vx_samples: 20
      vy_samples: 1  # 1 for differential drive
      vtheta_samples: 20
      sim_time: 1.7
      linear_granularity: 0.05
      angular_granularity: 0.025

      # Critics (score trajectories)
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]

      BaseObstacle.scale: 0.02
      PathAlign.scale: 32.0
      PathAlign.forward_point_distance: 0.1
      GoalAlign.scale: 24.0
      GoalAlign.forward_point_distance: 0.1
      PathDist.scale: 32.0
      GoalDist.scale: 24.0
      RotateToGoal.scale: 32.0
      RotateToGoal.slowing_factor: 5.0
      RotateToGoal.lookahead_time: -1.0
```

### MPPI Controller (Advanced)

```yaml
# File: controller_server.yaml
# MPPI controller for smoother motion

controller_server:
  ros__parameters:
    controller_frequency: 30.0
    controller_plugins: ["FollowPath"]

    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      time_steps: 56
      model_dt: 0.05
      batch_size: 2000
      vx_std: 0.2
      vy_std: 0.0
      wz_std: 0.4
      vx_max: 0.5
      vx_min: -0.35
      vy_max: 0.0
      wz_max: 1.9
      iteration_count: 1
      prune_distance: 1.7
      transform_tolerance: 0.1
      temperature: 0.3
      gamma: 0.015
      motion_model: "DiffDrive"
      visualize: true

      # Critics
      critics: ["ConstraintCritic", "ObstacleCritic", "GoalCritic", "GoalAngleCritic", "PathFollowCritic", "PathAngleCritic", "PreferForwardCritic"]

      ConstraintCritic:
        enabled: true
        cost_weight: 4.0
        cost_power: 1

      ObstacleCritic:
        enabled: true
        cost_weight: 1.0
        collision_cost: 10000.0
        collision_margin_distance: 0.1
        near_goal_distance: 0.5

      GoalCritic:
        enabled: true
        cost_weight: 5.0
        cost_power: 1
        threshold_to_consider: 1.0

      PathFollowCritic:
        enabled: true
        cost_weight: 5.0
        offset_from_furthest: 5
        threshold_to_consider: 0.6
```

---

## Behavior Trees

Nav2 uses **Behavior Trees** to orchestrate navigation. A behavior tree is a hierarchical structure that determines what action to take based on conditions.

### Default Navigation Behavior Tree

```xml
<!-- File: navigate_to_pose_w_replanning_and_recovery.xml -->
<!-- Default Nav2 behavior tree -->

<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <!-- Compute path to goal -->
        <RateController hz="1.0">
          <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
        </RateController>
        <!-- Follow the path -->
        <FollowPath path="{path}" controller_id="FollowPath"/>
      </PipelineSequence>
      <!-- Recovery actions if navigation fails -->
      <SequenceStar name="RecoveryActions">
        <ClearEntireCostmap name="ClearGlobalCostmap" service_name="global_costmap/clear_entirely_global_costmap"/>
        <ClearEntireCostmap name="ClearLocalCostmap" service_name="local_costmap/clear_entirely_local_costmap"/>
        <Spin spin_dist="1.57"/>  <!-- Spin 90 degrees -->
        <Wait wait_duration="5"/>
        <BackUp backup_dist="0.3" backup_speed="0.05"/>
      </SequenceStar>
    </RecoveryNode>
  </BehaviorTree>
</root>
```

### Understanding Behavior Tree Nodes

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                     BEHAVIOR TREE NODE TYPES                                 │
│                                                                              │
│   CONTROL NODES (how to execute children):                                  │
│   ─────────────────────────────────────────                                 │
│   Sequence      : Execute children in order, fail if any fails             │
│   Selector      : Try children until one succeeds                          │
│   Parallel      : Execute children simultaneously                          │
│   RecoveryNode  : Try main child, if fails try recovery                    │
│   PipelineSequence: Continue until all children succeed or one fails       │
│                                                                              │
│   ACTION NODES (do something):                                              │
│   ─────────────────────────────────────────                                 │
│   ComputePathToPose : Plan a path                                          │
│   FollowPath        : Execute path with controller                         │
│   Spin              : Rotate in place                                       │
│   BackUp            : Drive backward                                        │
│   Wait              : Pause execution                                       │
│   ClearCostmap      : Reset costmap                                         │
│                                                                              │
│   DECORATOR NODES (modify child behavior):                                  │
│   ─────────────────────────────────────────                                 │
│   RateController    : Limit execution rate                                 │
│   SpeedController   : Adjust based on conditions                           │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Custom Behavior Tree

```xml
<!-- File: custom_navigate.xml -->
<!-- Custom behavior tree with Isaac integration -->

<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="3" name="NavigateRecovery">
      <PipelineSequence name="NavigateSequence">

        <!-- First, ensure VSLAM is tracking -->
        <Condition ID="IsVSLAMTracking" topic="/visual_slam/status"/>

        <!-- Compute path with replanning -->
        <RateController hz="2.0">
          <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
        </RateController>

        <!-- Follow path with dynamic obstacle avoidance -->
        <FollowPath path="{path}" controller_id="FollowPath"/>

      </PipelineSequence>

      <!-- Recovery sequence -->
      <SequenceStar name="RecoverySequence">
        <!-- Clear costmaps -->
        <ClearEntireCostmap name="ClearGlobal"
            service_name="global_costmap/clear_entirely_global_costmap"/>
        <ClearEntireCostmap name="ClearLocal"
            service_name="local_costmap/clear_entirely_local_costmap"/>

        <!-- Wait for VSLAM to recover if lost -->
        <Wait wait_duration="2"/>

        <!-- Spin to gather new observations -->
        <Spin spin_dist="1.57"/>

        <!-- If still stuck, back up -->
        <BackUp backup_dist="0.5" backup_speed="0.1"/>
      </SequenceStar>
    </RecoveryNode>
  </BehaviorTree>
</root>
```

---

## Complete Nav2 Launch Configuration

### Main Nav2 Parameters File

```yaml
# File: nav2_params.yaml
# Complete Nav2 parameter configuration

amcl:  # Not used with VSLAM, but included for reference
  ros__parameters:
    use_sim_time: True

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    default_nav_to_pose_bt_xml: "navigate_to_pose_w_replanning_and_recovery.xml"
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_is_stuck_condition_bt_node
      - nav2_goal_reached_condition_bt_node
      - nav2_goal_updated_condition_bt_node
      - nav2_rate_controller_bt_node
      - nav2_recovery_node_bt_node
      - nav2_pipeline_sequence_bt_node

planner_server:
  ros__parameters:
    use_sim_time: True
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    controller_plugins: ["FollowPath"]

    progress_checker_plugins: ["progress_checker"]
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    goal_checker_plugins: ["goal_checker"]
    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True

    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.5
      max_vel_y: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.5
      min_speed_theta: 0.0
      acc_lim_x: 2.5
      acc_lim_y: 0.0
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_y: 0.0
      decel_lim_theta: -3.2
      vx_samples: 20
      vy_samples: 1
      vtheta_samples: 20
      sim_time: 1.7
      linear_granularity: 0.05
      angular_granularity: 0.025
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
      BaseObstacle.scale: 0.02
      PathAlign.scale: 32.0
      GoalAlign.scale: 24.0
      PathDist.scale: 32.0
      GoalDist.scale: 24.0
      RotateToGoal.scale: 32.0

recoveries_server:
  ros__parameters:
    use_sim_time: True
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    recovery_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_recoveries/Spin"
    backup:
      plugin: "nav2_recoveries/BackUp"
    wait:
      plugin: "nav2_recoveries/Wait"

waypoint_follower:
  ros__parameters:
    use_sim_time: True
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: True
      waypoint_pause_duration: 200
```

### Nav2 Launch File

```python
# File: nav2_bringup.launch.py
# Launch Nav2 with Isaac ROS integration

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """Launch Nav2 navigation stack."""

    pkg_share = get_package_share_directory('my_robot')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
            description='Nav2 parameters file'
        ),
        DeclareLaunchArgument(
            'map_subscribe_transient_local',
            default_value='true',
            description='Transient local map subscription'
        ),

        # Nav2 Bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': 'true',
                'use_composition': 'True',
                'use_respawn': 'True',
            }.items()
        ),
    ])
```

---

## Integration with Isaac ROS and VSLAM

### Complete System Launch

```python
# File: autonomous_robot.launch.py
# Complete autonomous robot with Isaac ROS + VSLAM + Nav2

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """Launch complete autonomous navigation system."""

    pkg_share = get_package_share_directory('my_robot')

    # 1. Robot description and state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': open(
                os.path.join(pkg_share, 'urdf', 'robot.urdf')
            ).read(),
            'use_sim_time': True
        }]
    )

    # 2. Visual SLAM for localization
    vslam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'vslam.launch.py')
        ),
        launch_arguments={
            'enable_localization': 'true',
            'enable_mapping': 'false',
            'map_path': '/tmp/my_map'
        }.items()
    )

    # 3. Isaac ROS perception
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'perception_pipeline.launch.py')
        )
    )

    # 4. Nav2 navigation (delayed start to let VSLAM initialize)
    nav2_launch = TimerAction(
        period=5.0,  # Wait 5 seconds for VSLAM
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, 'launch', 'nav2_bringup.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'params_file': os.path.join(pkg_share, 'config', 'nav2_params.yaml')
                }.items()
            )
        ]
    )

    # 5. Odometry relay (VSLAM to Nav2)
    odom_relay = Node(
        package='topic_tools',
        executable='relay',
        arguments=['/visual_slam/tracking/odometry', '/odom'],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        vslam_launch,
        perception_launch,
        odom_relay,
        nav2_launch,
    ])
```

---

## Sending Navigation Goals

### Using the Command Line

```bash
# Send a navigation goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}"

# Send waypoints (multiple goals)
ros2 action send_goal /follow_waypoints nav2_msgs/action/FollowWaypoints \
    "{poses: [{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}, {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}]}"
```

### Using RViz2

1. Open RViz2 with Nav2 displays
2. Click **2D Goal Pose** button
3. Click on the map where you want the robot to go
4. The robot will plan and execute the path

### Programmatic Navigation

```python
# File: navigation_client.py
# Send navigation goals programmatically

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math

class NavigationClient(Node):
    """Client for sending navigation goals."""

    def __init__(self):
        super().__init__('navigation_client')

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Wait for Nav2 to be ready
        self.get_logger().info('Waiting for Nav2...')
        self.nav_client.wait_for_server()
        self.get_logger().info('Nav2 is ready!')

    def send_goal(self, x: float, y: float, yaw: float = 0.0):
        """Send a navigation goal."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Orientation (yaw to quaternion)
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f'Sending goal: ({x}, {y}, {yaw})')

        # Send goal
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected!')
            return

        self.get_logger().info('Goal accepted!')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback."""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Distance remaining: {feedback.distance_remaining:.2f}m'
        )

    def get_result_callback(self, future):
        """Handle navigation result."""
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')


def main():
    rclpy.init()
    client = NavigationClient()

    # Send a goal
    client.send_goal(x=3.0, y=2.0, yaw=1.57)  # Go to (3, 2) facing +Y

    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Hands-On Exercise

### Exercise: Autonomous Warehouse Navigation

**Objective:** Configure Nav2 to navigate your robot through a warehouse, visiting multiple waypoints.

**Difficulty:** Advanced

**Instructions:**

1. Launch the complete system (Isaac Sim + VSLAM + Isaac ROS + Nav2)
2. Configure Nav2 parameters for your robot
3. Create a waypoint patrol that visits 4 locations
4. Handle navigation failures gracefully

**Starter Code:**

```python
# File: warehouse_patrol.py
# Warehouse patrol with multiple waypoints

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math

class WarehousePatrol(Node):
    def __init__(self):
        super().__init__('warehouse_patrol')

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Define patrol waypoints (x, y, yaw)
        self.waypoints = [
            # TODO: Add your waypoints here
            # (1.0, 0.0, 0.0),
            # (2.0, 1.0, 1.57),
            # ...
        ]

        self.current_waypoint = 0

        # Wait for Nav2
        self.get_logger().info('Waiting for Nav2...')
        self.nav_client.wait_for_server()
        self.get_logger().info('Starting patrol!')

        # Start patrol
        self.send_next_waypoint()

    def send_next_waypoint(self):
        # TODO: Implement waypoint sending
        # Cycle through self.waypoints
        # Call self.send_goal(x, y, yaw)
        pass

    def send_goal(self, x, y, yaw):
        # TODO: Send navigation goal
        pass

    def result_callback(self, future):
        # TODO: Handle result and send next waypoint
        pass


def main():
    rclpy.init()
    patrol = WarehousePatrol()
    rclpy.spin(patrol)
    patrol.destroy_node()
    rclpy.shutdown()
```

<details>
<summary>Click to reveal solution</summary>

```python
# File: warehouse_patrol_solution.py
# Warehouse patrol with multiple waypoints - Solution

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math

class WarehousePatrol(Node):
    def __init__(self):
        super().__init__('warehouse_patrol')

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Define patrol waypoints (x, y, yaw)
        self.waypoints = [
            (1.0, 0.0, 0.0),      # Waypoint 1
            (3.0, 0.0, 1.57),     # Waypoint 2, facing +Y
            (3.0, 2.0, 3.14),     # Waypoint 3, facing -X
            (1.0, 2.0, -1.57),    # Waypoint 4, facing -Y
        ]

        self.current_waypoint = 0
        self.patrol_count = 0

        # Wait for Nav2
        self.get_logger().info('Waiting for Nav2...')
        self.nav_client.wait_for_server()
        self.get_logger().info('Starting warehouse patrol!')

        # Start patrol
        self.send_next_waypoint()

    def send_next_waypoint(self):
        if self.current_waypoint >= len(self.waypoints):
            self.current_waypoint = 0
            self.patrol_count += 1
            self.get_logger().info(f'Patrol {self.patrol_count} complete!')

        wp = self.waypoints[self.current_waypoint]
        self.get_logger().info(
            f'Navigating to waypoint {self.current_waypoint + 1}: '
            f'({wp[0]}, {wp[1]})'
        )
        self.send_goal(wp[0], wp[1], wp[2])

    def send_goal(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected! Retrying...')
            self.send_next_waypoint()
            return

        self.get_logger().info('Goal accepted!')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Distance remaining: {feedback.distance_remaining:.2f}m',
            throttle_duration_sec=2.0
        )

    def result_callback(self, future):
        result = future.result()
        status = result.status

        if status == 4:  # SUCCEEDED
            self.get_logger().info('Reached waypoint!')
            self.current_waypoint += 1
            self.send_next_waypoint()
        else:
            self.get_logger().warn(f'Navigation failed with status: {status}')
            # Retry same waypoint
            self.get_logger().info('Retrying waypoint...')
            self.send_next_waypoint()


def main():
    rclpy.init()
    patrol = WarehousePatrol()
    try:
        rclpy.spin(patrol)
    except KeyboardInterrupt:
        pass
    patrol.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

</details>

---

## Common Issues and Solutions

### Issue 1: Robot Does Not Move

**Cause:** Transform tree incomplete or costmap not updating.

**Solution:**
```bash
# Check TF tree
ros2 run tf2_tools view_frames

# Verify odom → base_link transform exists
ros2 topic echo /tf --once | grep base_link

# Check costmap is publishing
ros2 topic hz /global_costmap/costmap
ros2 topic hz /local_costmap/costmap
```

### Issue 2: Path Planning Fails

**Cause:** Start or goal in obstacle, or planner timeout.

**Solution:**
```bash
# Check if positions are in free space
ros2 run rviz2 rviz2
# Visualize costmap and robot position

# Increase planner timeout
# In nav2_params.yaml:
# planner_server:
#   ros__parameters:
#     expected_planner_frequency: 10.0  # Lower = more time
```

### Issue 3: Robot Oscillates

**Cause:** Controller gains too high or goal tolerance too tight.

**Solution:**
```yaml
# In controller_server parameters:
goal_checker:
  xy_goal_tolerance: 0.3  # Increase tolerance
  yaw_goal_tolerance: 0.3

FollowPath:
  # Reduce oscillation by lowering gains
  PathAlign.scale: 24.0  # Lower from 32
  GoalAlign.scale: 18.0  # Lower from 24
```

### Issue 4: Recovery Behaviors Not Working

**Cause:** Recovery server not started or wrong costmap topic.

**Solution:**
```bash
# Check recovery server is running
ros2 node list | grep recoveries

# Verify costmap topic in params matches actual topic
ros2 topic list | grep costmap
```

---

## Summary

### Key Takeaways

1. **Nav2 is modular**: Planners, controllers, and behaviors can be swapped independently. Choose components that match your robot and environment.

2. **Costmaps represent obstacles**: Global costmap for long-term planning, local costmap for reactive avoidance. Configure layers based on your sensors.

3. **Behavior trees orchestrate navigation**: Understand the tree structure to customize recovery behaviors and navigation logic.

4. **Integration requires correct transforms**: Ensure VSLAM publishes map → odom → base_link and that Nav2 can read them.

5. **Tune parameters for your robot**: Default parameters are starting points. Adjust velocities, tolerances, and gains based on testing.

### Quick Reference

```bash
# Launch Nav2
ros2 launch nav2_bringup navigation_launch.py params_file:=nav2_params.yaml

# Send navigation goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}}}"

# Check Nav2 status
ros2 lifecycle list /planner_server
ros2 lifecycle list /controller_server

# Clear costmaps
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap
```

```yaml
# Essential Nav2 parameters
planner_server:
  GridBased:
    plugin: "nav2_navfn_planner/NavfnPlanner"
    tolerance: 0.5

controller_server:
  FollowPath:
    plugin: "dwb_core::DWBLocalPlanner"
    max_vel_x: 0.5
    max_vel_theta: 1.0
```

---

## Module 3 Complete!

Congratulations! You have completed Module 3: The AI-Robot Brain. Your robot now has:

- **Photorealistic simulation** with Isaac Sim
- **GPU-accelerated perception** with Isaac ROS
- **Visual localization** with VSLAM
- **Autonomous navigation** with Nav2

These capabilities form the foundation of modern autonomous robots used in warehouses, hospitals, and homes around the world.

---

## What's Next

In **Module 4: Vision-Language-Action**, you will explore the frontier of robotics AI:
- Vision-Language Models for understanding scenes
- Language-conditioned behavior
- Imitation learning
- Foundation models for robotics

This represents the cutting edge of embodied AI research.

[Continue to Module 4 →](/docs/module-4-vla)

---

## Additional Resources

- [Nav2 Documentation](https://navigation.ros.org/) - Official Nav2 documentation
- [Nav2 Tuning Guide](https://navigation.ros.org/tuning/index.html) - Parameter tuning
- [Behavior Tree CPP](https://www.behaviortree.dev/) - BT framework documentation
- [MPPI Controller Paper](https://arxiv.org/abs/2209.04359) - MPPI algorithm details
- [ROS 2 Navigation Examples](https://github.com/ros-planning/navigation2/tree/main/nav2_bringup) - Example configurations
