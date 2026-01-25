---
sidebar_position: 4
title: "Chapter 3: Capstone Project"
description: "Complete capstone project integrating ROS 2, simulation, NVIDIA Isaac, and VLA models into a fully functional voice-controlled humanoid robot system."
keywords: [capstone, project, integration, humanoid robot, full stack robotics, VLA, voice control]
---

# Chapter 3: Capstone Project

**Building Your Voice-Controlled Robot**

---

## Congratulations

You have made it to the final chapter of this textbook. This is not just an ending—it is a beginning. Everything you have learned converges here into a single, complete system.

Over the past modules, you have built:
- A communication system (ROS 2 nodes, topics, services, actions)
- A digital twin (Gazebo simulation, physics, sensors)
- An AI brain (Isaac Sim, perception, VSLAM, Nav2)
- Ears and a mind (Whisper speech recognition, LLM planning)

Now you will combine them all.

---

## Project Overview

The capstone project is a **voice-controlled service robot** that can:

1. **Listen** for voice commands using Whisper
2. **Understand** commands using LLM-based planning
3. **Navigate** autonomously using Nav2
4. **Perceive** its environment using GPU-accelerated vision
5. **Execute** tasks and report back to the user

```
┌─────────────────────────────────────────────────────────────────────────┐
│                     CAPSTONE PROJECT ARCHITECTURE                        │
│                                                                          │
│                           User speaks                                    │
│                         "Go to the kitchen                               │
│                          and find my keys"                               │
│                               │                                          │
│                               ▼                                          │
│   ┌─────────────────────────────────────────────────────────────────┐   │
│   │                        WHISPER NODE                              │   │
│   │                    (Speech Recognition)                          │   │
│   │                        Module 4.1                                │   │
│   └─────────────────────────────┬───────────────────────────────────┘   │
│                                 │                                        │
│                                 ▼                                        │
│   ┌─────────────────────────────────────────────────────────────────┐   │
│   │                      COGNITIVE PLANNER                           │   │
│   │                     (LLM Task Planning)                          │   │
│   │                        Module 4.2                                │   │
│   └─────────────────────────────┬───────────────────────────────────┘   │
│                                 │                                        │
│                                 ▼                                        │
│   ┌───────────────┐   ┌───────────────┐   ┌───────────────────────┐    │
│   │   NAVIGATION  │   │  PERCEPTION   │   │     MANIPULATION      │    │
│   │     (Nav2)    │   │  (Isaac ROS)  │   │   (Future Work)       │    │
│   │   Module 3.4  │   │   Module 3.2  │   │                       │    │
│   └───────┬───────┘   └───────┬───────┘   └───────────────────────┘    │
│           │                   │                                         │
│           └─────────┬─────────┘                                         │
│                     │                                                    │
│                     ▼                                                    │
│   ┌─────────────────────────────────────────────────────────────────┐   │
│   │                        SIMULATION                                │   │
│   │              (Isaac Sim / Gazebo with Robot)                     │   │
│   │                    Modules 2 & 3.1                               │   │
│   └─────────────────────────────────────────────────────────────────┘   │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### Learning Objectives

By completing this project, you will demonstrate mastery of:

- [ ] Integrating multiple ROS 2 nodes into a cohesive system
- [ ] Configuring Nav2 for autonomous navigation
- [ ] Processing speech input in real-time
- [ ] Using LLMs for robot task planning
- [ ] Debugging complex multi-component systems
- [ ] Creating a demonstrable robotics project

---

## Project Requirements

### Minimum Requirements (Must Complete)

1. **Voice Input**: Robot responds to at least 5 distinct voice commands
2. **Navigation**: Robot can navigate to at least 3 named locations
3. **Planning**: LLM generates valid action plans for commands
4. **Feedback**: Robot speaks status updates to the user
5. **Simulation**: System runs in Isaac Sim or Gazebo

### Extended Requirements (Challenge Yourself)

6. **Object Detection**: Robot can identify and locate objects
7. **Error Recovery**: System replans when actions fail
8. **Wake Word**: Robot only responds after hearing "Hey Robot"
9. **Multi-Step Tasks**: Robot completes tasks with 5+ steps
10. **Real Hardware**: Deploy to a physical robot (if available)

---

## System Architecture

### Node Graph

Your system will consist of the following ROS 2 nodes:

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         ROS 2 NODE GRAPH                                 │
│                                                                          │
│   ┌─────────────┐     /speech/transcription     ┌──────────────────┐   │
│   │             │ ─────────────────────────────▶│                  │   │
│   │  whisper_   │                               │   cognitive_     │   │
│   │   node      │     /speech/command           │   planner_node   │   │
│   │             │ ─────────────────────────────▶│                  │   │
│   └─────────────┘                               └────────┬─────────┘   │
│         ▲                                                │              │
│         │ audio                                          │              │
│         │                                                ▼              │
│   ┌─────────────┐                               ┌──────────────────┐   │
│   │ Microphone  │                               │  plan_executor_  │   │
│   │   Input     │                               │     node         │   │
│   └─────────────┘                               └────────┬─────────┘   │
│                                                          │              │
│                           /navigate_to_pose/goal         │              │
│                    ┌─────────────────────────────────────┘              │
│                    │                                                    │
│                    ▼                                                    │
│   ┌─────────────────────────────────────────────────────────────────┐  │
│   │                          NAV2 STACK                              │  │
│   │  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────────┐    │  │
│   │  │ Planner  │  │Controller│  │ Costmap  │  │   BT Nav     │    │  │
│   │  │  Server  │  │  Server  │  │  Server  │  │   Server     │    │  │
│   │  └──────────┘  └──────────┘  └──────────┘  └──────────────┘    │  │
│   └─────────────────────────────────────────────────────────────────┘  │
│                              │                                          │
│                              │ /cmd_vel                                 │
│                              ▼                                          │
│   ┌─────────────────────────────────────────────────────────────────┐  │
│   │                      SIMULATION / ROBOT                          │  │
│   │              (Isaac Sim, Gazebo, or Real Hardware)               │  │
│   └─────────────────────────────────────────────────────────────────┘  │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### Topics and Services

| Topic/Service | Type | Description |
|---------------|------|-------------|
| `/speech/transcription` | `std_msgs/String` | Raw transcribed text |
| `/speech/command` | `std_msgs/String` | Filtered robot commands |
| `/speech/output` | `std_msgs/String` | Text-to-speech output |
| `/planner/status` | `std_msgs/String` | Planning status updates |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Navigation goals |
| `/perception/objects` | Custom | Detected objects |
| `/cmd_vel` | `geometry_msgs/Twist` | Robot velocity commands |

---

## Step-by-Step Implementation

### Phase 1: Environment Setup

First, set up your development environment and simulation.

#### 1.1 Create Workspace

```bash
# Create capstone workspace
mkdir -p ~/capstone_ws/src
cd ~/capstone_ws/src

# Create main package
ros2 pkg create --build-type ament_python capstone_robot \
    --dependencies rclpy std_msgs geometry_msgs nav2_msgs

# Create launch package
ros2 pkg create --build-type ament_cmake capstone_bringup \
    --dependencies launch launch_ros
```

#### 1.2 Set Up Simulation World

Create a simulation world with named locations. For Gazebo:

```xml
<!-- worlds/home.sdf -->
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="home">
    <!-- Ground plane -->
    <model name="ground">
      <static>true</static>
      <link name="ground_link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>20 20</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>20 20</size>
            </plane>
          </geometry>
        </visual>
      </link>
    </model>

    <!-- Kitchen area (5, 2) -->
    <model name="kitchen_marker">
      <static>true</static>
      <pose>5 2 0.01 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box><size>2 2 0.01</size></box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.2 1</ambient>
          </material>
        </visual>
      </link>
    </model>

    <!-- Living room area (0, 0) -->
    <model name="living_room_marker">
      <static>true</static>
      <pose>0 0 0.01 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box><size>3 3 0.01</size></box>
          </geometry>
          <material>
            <ambient>0.2 0.8 0.2 1</ambient>
          </material>
        </visual>
      </link>
    </model>

    <!-- Bedroom area (-3, 4) -->
    <model name="bedroom_marker">
      <static>true</static>
      <pose>-3 4 0.01 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box><size>2.5 2.5 0.01</size></box>
          </geometry>
          <material>
            <ambient>0.2 0.2 0.8 1</ambient>
          </material>
        </visual>
      </link>
    </model>

    <!-- Walls -->
    <model name="walls">
      <static>true</static>
      <!-- Add wall definitions here -->
    </model>

    <!-- Furniture and objects -->
    <!-- Add tables, chairs, etc. -->

  </world>
</sdf>
```

#### 1.3 Location Configuration

Create a configuration file for named locations:

```yaml
# config/locations.yaml
locations:
  living_room:
    x: 0.0
    y: 0.0
    theta: 0.0
    description: "Main living area"

  kitchen:
    x: 5.0
    y: 2.0
    theta: 0.0
    description: "Kitchen area with fridge and counter"

  bedroom:
    x: -3.0
    y: 4.0
    theta: 1.57
    description: "Bedroom with bed and dresser"

  bathroom:
    x: -3.0
    y: -2.0
    theta: 3.14
    description: "Bathroom"

  entrance:
    x: 0.0
    y: -5.0
    theta: 1.57
    description: "Main entrance"
```

---

### Phase 2: Core Integration Node

The capstone node connects all the pieces together. Let us build it step by step.

#### Understanding the Integration Node

This node is the "brain" that coordinates everything:

```
┌─────────────────────────────────────────────────────────────────────┐
│                    CAPSTONE INTEGRATION NODE                         │
│                                                                      │
│   Whisper ──────▶ [Transcription] ──▶ Wake word check               │
│                          │                                           │
│                          ▼                                           │
│                   [Command Parser] ──▶ Pattern matching              │
│                          │                                           │
│                          ▼                                           │
│                   [Action Handler] ──▶ Nav2 / Speech / etc.         │
│                          │                                           │
│                          ▼                                           │
│                   [Feedback] ──────▶ Speech output to user          │
└─────────────────────────────────────────────────────────────────────┘
```

#### Step 1: Imports and Data Structures

```python
#!/usr/bin/env python3
"""
capstone_node.py
Main integration node for the voice-controlled robot.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

import yaml
import os
from typing import Dict, Optional
from dataclasses import dataclass
from enum import Enum


class RobotState(Enum):
    """What the robot is currently doing."""
    IDLE = "idle"           # Waiting for commands
    LISTENING = "listening"  # Actively listening
    PLANNING = "planning"    # Figuring out what to do
    EXECUTING = "executing"  # Doing something
    ERROR = "error"         # Something went wrong


@dataclass
class Location:
    """A named location in the environment."""
    name: str        # e.g., "kitchen"
    x: float         # X coordinate on map
    y: float         # Y coordinate on map
    theta: float     # Orientation (radians)
    description: str # e.g., "Kitchen area with fridge"
```

#### Step 2: Initialize the Node

```python
class CapstoneRobot(Node):
    """
    Main capstone integration node.
    Connects speech recognition, planning, and execution.
    """

    def __init__(self):
        super().__init__('capstone_robot')

        # === PARAMETERS ===
        # These can be changed at launch time
        self.declare_parameter('locations_file', '')
        self.declare_parameter('wake_word', 'hey robot')
        self.declare_parameter('use_wake_word', True)

        # === STATE ===
        self.state = RobotState.IDLE
        self.current_location = "living_room"
        self.wake_word_active = False

        # Load known locations
        self._load_locations()

        # Set up ROS communication
        self._setup_publishers()
        self._setup_subscribers()
        self._setup_action_clients()

        # Initialize command patterns
        self._init_command_patterns()

        # Periodic status check
        self.create_timer(5.0, self._status_timer)

        # Announce we're ready
        self.get_logger().info('Capstone robot initialized')
        self._say("Hello! I am ready to help. Say 'Hey Robot' to get my attention.")
```

#### Step 3: Set Up ROS Communication

```python
    def _setup_publishers(self):
        """Create publishers for output."""
        # For text-to-speech
        self.speech_pub = self.create_publisher(
            String, 'speech/output', 10
        )
        # For status monitoring
        self.status_pub = self.create_publisher(
            String, 'planner/status', 10
        )

    def _setup_subscribers(self):
        """Subscribe to input topics."""
        # Raw transcriptions (for wake word detection)
        self.create_subscription(
            String,
            'speech/transcription',
            self._on_transcription,
            10
        )
        # Filtered commands (from Whisper node)
        self.create_subscription(
            String,
            'speech/command',
            self._on_command,
            10
        )

    def _setup_action_clients(self):
        """Set up action clients for robot capabilities."""
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
```

#### Step 4: Load Location Configuration

```python
    def _load_locations(self):
        """Load named locations from file or use defaults."""
        locations_file = self.get_parameter('locations_file').value

        if locations_file and os.path.exists(locations_file):
            # Load from YAML file
            with open(locations_file, 'r') as f:
                config = yaml.safe_load(f)

            self.locations: Dict[str, Location] = {}
            for name, data in config.get('locations', {}).items():
                self.locations[name] = Location(
                    name=name,
                    x=data['x'],
                    y=data['y'],
                    theta=data['theta'],
                    description=data.get('description', '')
                )
            self.get_logger().info(f'Loaded {len(self.locations)} locations')
        else:
            # Use default locations
            self.locations = {
                "living_room": Location("living_room", 0.0, 0.0, 0.0, "Living room"),
                "kitchen": Location("kitchen", 5.0, 2.0, 0.0, "Kitchen"),
                "bedroom": Location("bedroom", -3.0, 4.0, 1.57, "Bedroom"),
            }
            self.get_logger().info('Using default locations')
```

#### Step 5: Initialize Command Patterns

```python
    def _init_command_patterns(self):
        """
        Define what commands the robot understands.
        Maps keywords to handler functions.
        """
        self.command_patterns = {
            # Navigation commands
            "go to": self._handle_navigation,
            "navigate to": self._handle_navigation,
            "move to": self._handle_navigation,

            # Search commands
            "find": self._handle_find,

            # Information commands
            "where am i": self._handle_where_am_i,
            "what can you do": self._handle_capabilities,
            "help": self._handle_help,

            # Control commands
            "stop": self._handle_stop,
            "cancel": self._handle_stop,
        }
```

#### Step 6: Handle Incoming Transcriptions

```python
    def _on_transcription(self, msg: String):
        """
        Called when Whisper transcribes speech.
        Check for wake word.
        """
        text = msg.data.lower()
        wake_word = self.get_parameter('wake_word').value
        use_wake_word = self.get_parameter('use_wake_word').value

        if use_wake_word:
            if wake_word in text:
                self.wake_word_active = True
                self._say("Yes? I'm listening.")
                self.get_logger().info('Wake word detected')
        else:
            # Always active if wake word disabled
            self.wake_word_active = True

    def _on_command(self, msg: String):
        """
        Called when a command is received.
        Only process if wake word was heard (if enabled).
        """
        command = msg.data

        # Check if we should respond
        if self.get_parameter('use_wake_word').value and not self.wake_word_active:
            return  # Ignore - wake word not active

        self.get_logger().info(f'Processing: {command}')
        self._set_state(RobotState.PLANNING)

        # Find matching command pattern
        self._process_command(command)

        # Reset wake word
        self.wake_word_active = False
```

#### Step 7: Process Commands

```python
    def _process_command(self, command: str):
        """Match command to a handler and execute it."""
        command_lower = command.lower()

        # Try each pattern
        for pattern, handler in self.command_patterns.items():
            if pattern in command_lower:
                handler(command)
                return

        # No match found
        self._say("I didn't understand. Say 'help' to see what I can do.")
        self._set_state(RobotState.IDLE)
```

#### Step 8: Navigation Handler

```python
    def _handle_navigation(self, command: str):
        """Handle 'go to <location>' commands."""
        # Find which location was mentioned
        location_name = None
        for name in self.locations.keys():
            # Handle "living_room" matching "living room"
            if name.replace("_", " ") in command.lower():
                location_name = name
                break

        if not location_name:
            self._say(f"I know these places: {', '.join(self.locations.keys())}")
            self._set_state(RobotState.IDLE)
            return

        # Start navigation
        self._navigate_to(location_name)

    def _navigate_to(self, location_name: str):
        """Send navigation goal to Nav2."""
        location = self.locations[location_name]

        self._say(f"Going to the {location_name}.")
        self._set_state(RobotState.EXECUTING)

        # Create the goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = location.x
        goal_pose.pose.position.y = location.y

        # Check if Nav2 is available
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self._say("Navigation system is not available.")
            self._set_state(RobotState.ERROR)
            return

        # Send the goal
        goal = NavigateToPose.Goal()
        goal.pose = goal_pose

        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_nav_goal_response)

    def _on_nav_goal_response(self, future):
        """Called when Nav2 accepts/rejects our goal."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self._say("Navigation goal was rejected.")
            self._set_state(RobotState.IDLE)
            return

        # Wait for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future):
        """Called when navigation completes."""
        result = future.result()

        if result.status == 4:  # SUCCEEDED
            self._say("I have arrived.")
            self._set_state(RobotState.IDLE)
        else:
            self._say("Navigation failed. There might be an obstacle.")
            self._set_state(RobotState.ERROR)
```

#### Step 9: Other Command Handlers

```python
    def _handle_find(self, command: str):
        """Handle 'find <object>' commands."""
        words = command.lower().split()
        try:
            idx = words.index("find")
            target = " ".join(words[idx + 1:])
        except (ValueError, IndexError):
            target = "something"

        self._say(f"Looking for {target}. Note: requires perception system.")
        self._set_state(RobotState.IDLE)

    def _handle_where_am_i(self, command: str):
        """Tell user current location."""
        self._say(f"I am in the {self.current_location}.")
        self._set_state(RobotState.IDLE)

    def _handle_capabilities(self, command: str):
        """List what the robot can do."""
        self._say("I can navigate to rooms, respond to voice, and tell my location.")
        self._set_state(RobotState.IDLE)

    def _handle_help(self, command: str):
        """Give usage instructions."""
        self._say(
            "Try: Go to the kitchen. Navigate to the bedroom. "
            "Where am I? What can you do? Stop."
        )
        self._set_state(RobotState.IDLE)

    def _handle_stop(self, command: str):
        """Stop current action."""
        self._say("Stopping.")
        self._set_state(RobotState.IDLE)
```

#### Step 10: Utility Methods and Main

```python
    def _say(self, message: str):
        """Speak a message via text-to-speech."""
        msg = String()
        msg.data = message
        self.speech_pub.publish(msg)
        self.get_logger().info(f'Saying: {message}')

    def _set_state(self, state: RobotState):
        """Update and publish robot state."""
        self.state = state
        status_msg = String()
        status_msg.data = f"State: {state.value}"
        self.status_pub.publish(status_msg)

    def _status_timer(self):
        """Periodic status check."""
        if self.state == RobotState.IDLE:
            self.get_logger().debug('Idle, waiting for commands')


def main(args=None):
    """Start the capstone robot node."""
    rclpy.init(args=args)

    node = CapstoneRobot()

    # Use multi-threaded executor for concurrent callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

### Phase 3: Launch System

Create a comprehensive launch file that brings up all components.

```python
# launch/capstone.launch.py
"""
Complete launch file for the capstone voice-controlled robot.
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare arguments
    use_sim = LaunchConfiguration('use_sim', default='true')
    use_whisper = LaunchConfiguration('use_whisper', default='true')
    whisper_model = LaunchConfiguration('whisper_model', default='small')
    llm_provider = LaunchConfiguration('llm_provider', default='local')

    # Package paths
    capstone_pkg = get_package_share_directory('capstone_bringup')
    nav2_pkg = get_package_share_directory('nav2_bringup')

    # Config files
    locations_file = os.path.join(capstone_pkg, 'config', 'locations.yaml')
    nav2_params = os.path.join(capstone_pkg, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('use_sim', default_value='true'),
        DeclareLaunchArgument('use_whisper', default_value='true'),
        DeclareLaunchArgument('whisper_model', default_value='small'),
        DeclareLaunchArgument('llm_provider', default_value='local'),

        # Simulation (if enabled)
        # Include your simulation launch here

        # Nav2 stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                nav2_pkg, '/launch/bringup_launch.py'
            ]),
            launch_arguments={
                'use_sim_time': use_sim,
                'params_file': nav2_params,
            }.items(),
        ),

        # Whisper speech recognition node
        Node(
            package='whisper_ros',
            executable='whisper_node',
            name='whisper_node',
            parameters=[{
                'model_size': whisper_model,
                'language': 'en',
                'device': 'cuda',
            }],
            condition=IfCondition(use_whisper),
            output='screen',
        ),

        # Main capstone integration node
        Node(
            package='capstone_robot',
            executable='capstone_node',
            name='capstone_robot',
            parameters=[{
                'locations_file': locations_file,
                'llm_provider': llm_provider,
                'use_wake_word': True,
                'wake_word': 'hey robot',
            }],
            output='screen',
        ),

        # Text-to-speech node (optional)
        # Node(
        #     package='tts_ros',
        #     executable='tts_node',
        #     ...
        # ),
    ])
```

---

### Phase 4: Testing and Demonstration

#### 4.1 Test Individual Components

Test each component in isolation before integration:

```bash
# Terminal 1: Start simulation
ros2 launch capstone_bringup simulation.launch.py

# Terminal 2: Test navigation
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 2.0}}}}"

# Terminal 3: Test speech recognition
ros2 run whisper_ros whisper_node

# Terminal 4: Monitor topics
ros2 topic echo /speech/transcription
```

#### 4.2 Integration Test

Run the complete system:

```bash
# Start everything
ros2 launch capstone_bringup capstone.launch.py

# In another terminal, monitor status
ros2 topic echo /planner/status

# Test voice commands:
# - "Hey robot, go to the kitchen"
# - "Hey robot, where am I?"
# - "Hey robot, what can you do?"
```

#### 4.3 Create Demo Script

Document a demonstration sequence:

```markdown
# Capstone Demo Script

## Setup
1. Launch simulation: `ros2 launch capstone_bringup capstone.launch.py`
2. Wait for "Hello! I am ready to help" message
3. Ensure microphone is connected

## Demo Sequence

### Part 1: Basic Navigation
1. Say: "Hey robot, go to the kitchen"
   - Expected: Robot navigates to kitchen area
   - Wait for "I have arrived" message

2. Say: "Hey robot, where am I?"
   - Expected: "I am currently in the kitchen"

3. Say: "Hey robot, go to the bedroom"
   - Expected: Robot navigates to bedroom

### Part 2: Help and Capabilities
4. Say: "Hey robot, what can you do?"
   - Expected: Lists capabilities

5. Say: "Hey robot, help"
   - Expected: Lists available commands

### Part 3: Error Handling
6. Say: "Hey robot, go to the moon"
   - Expected: "I don't know where moon is"

7. During navigation, say: "Hey robot, stop"
   - Expected: Robot stops moving

## Closing
- Show ROS node graph: `rqt_graph`
- Show topic flow: `ros2 topic list`
```

---

## Evaluation Criteria

### Technical Requirements (60 points)

| Requirement | Points | Criteria |
|-------------|--------|----------|
| Voice recognition works | 10 | Commands are correctly transcribed |
| Navigation functional | 15 | Robot reaches destinations |
| Planning generates valid plans | 10 | LLM outputs are parsed correctly |
| Multi-node integration | 15 | All nodes communicate properly |
| Error handling | 10 | System recovers from failures |

### Code Quality (20 points)

| Requirement | Points | Criteria |
|-------------|--------|----------|
| Code organization | 5 | Clear structure, separation of concerns |
| Documentation | 5 | Comments, README, usage instructions |
| ROS 2 best practices | 5 | Proper use of nodes, topics, actions |
| Testing | 5 | Launch files work, manual tests pass |

### Demonstration (20 points)

| Requirement | Points | Criteria |
|-------------|--------|----------|
| Live demo successful | 10 | System works as shown |
| Clear explanation | 5 | Can explain how it works |
| Handles questions | 5 | Understands the system deeply |

---

## Common Issues and Solutions

### Issue: Navigation fails with "No valid pose"

**Solution**: Check that the map is loaded and localization is running:
```bash
ros2 topic echo /amcl_pose  # Check localization
ros2 service call /reinitialize_global_localization std_srvs/srv/Empty
```

### Issue: Whisper transcription is slow

**Solutions**:
1. Use a smaller model: `whisper_model:=tiny`
2. Ensure GPU is being used: check `nvidia-smi`
3. Reduce audio quality if needed

### Issue: Robot doesn't respond to commands

**Solutions**:
1. Check wake word is being detected
2. Verify topic connections: `ros2 topic info /speech/command`
3. Check node is running: `ros2 node list`

### Issue: Plan executor action fails

**Solutions**:
1. Verify action server is running: `ros2 action list`
2. Check action server status: `ros2 action info /navigate_to_pose`
3. Ensure simulation is publishing required topics

---

## Extensions and Future Work

If you complete the minimum requirements, consider these extensions:

### 1. Object Manipulation

Add the ability to pick up and place objects:
- Integrate with MoveIt for arm control
- Add object detection and pose estimation
- Implement grasp planning

### 2. Multi-Room Search

Implement systematic search across rooms:
- Plan search path across all rooms
- Use perception to scan each location
- Report findings to user

### 3. Task Learning

Allow the robot to learn new tasks:
- Record successful task executions
- Extract patterns for future use
- Build task library over time

### 4. Natural Dialogue

Improve conversation capabilities:
- Handle follow-up questions
- Maintain conversation context
- Ask clarifying questions

---

## Submission Checklist

Before submitting your capstone project, verify:

- [ ] All code is in a clean Git repository
- [ ] README.md explains how to run the project
- [ ] Launch files work without errors
- [ ] Robot responds to at least 5 voice commands
- [ ] Navigation works to at least 3 locations
- [ ] Video demonstration recorded
- [ ] Code is commented and organized

---

## Summary

Congratulations on completing the capstone project. You have built a voice-controlled robot that integrates:

| Module | Component | Your Implementation |
|--------|-----------|---------------------|
| Module 1 | ROS 2 Communication | Nodes, topics, actions |
| Module 2 | Simulation | Gazebo/Isaac environment |
| Module 3 | AI Navigation | Nav2, localization |
| Module 4 | VLA | Whisper, LLM planning |

This project demonstrates that you can:
- Design complex robotic systems
- Integrate multiple AI components
- Build end-to-end solutions
- Debug multi-component systems

---

## What's Next?

You have completed the Physical AI & Humanoid Robotics textbook. Here are paths forward:

### Continue Learning
- Explore manipulation with MoveIt 2
- Study reinforcement learning for robotics
- Learn about real-world deployment challenges

### Build Projects
- Adapt the capstone for a physical robot
- Create a more sophisticated VLA system
- Contribute to open-source robotics projects

### Join the Community
- ROS Discourse: [discourse.ros.org](https://discourse.ros.org)
- NVIDIA Isaac Community
- Robotics conferences (ICRA, IROS, ROSCon)

### Career Paths
- Robotics Software Engineer
- AI/ML Engineer (Robotics)
- Research Scientist (Embodied AI)
- Robotics Startup Founder

---

## Final Words

You started this textbook wondering how robots work. Now you can build them.

The field of robotics is entering a new era. Large language models, advanced perception, and improved hardware are converging to make robots that truly understand and interact with the world. The skills you have learned position you at the forefront of this revolution.

Go build something amazing.

---

## Additional Resources

- [ROS 2 Navigation Documentation](https://navigation.ros.org/)
- [MoveIt 2 Documentation](https://moveit.ros.org/)
- [NVIDIA Isaac Documentation](https://developer.nvidia.com/isaac)
- [Robotics Conferences](https://www.ieee-ras.org/conferences-workshops)
- [ROS Discourse](https://discourse.ros.org/)
- [Physical AI Research Papers](https://arxiv.org/list/cs.RO/recent)
