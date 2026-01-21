---
sidebar_position: 4
title: "Chapter 3: Python Client Library (rclpy)"
description: "Master rclpy, the ROS 2 Python client library. Learn executors, callback groups, parameters, lifecycle nodes, and production-quality patterns."
keywords: [ROS 2, rclpy, Python, executor, callback, parameters, lifecycle, robotics, node, publisher, subscriber]
---

# Chapter 3: Python Client Library (rclpy)

## Chapter Overview

Chapters 1 and 2 introduced the *concepts* of nodes, topics, services, and actions. This chapter teaches you *how to implement them properly* using rclpy, the Python client library for ROS 2. Beyond basic syntax, this chapter covers production concerns: how callbacks are scheduled, how to handle parameters, and how to write code that is robust, testable, and maintainable.

**Prerequisites**: Completed [Chapter 1: Nodes and Topics](/docs/module-1-ros2/nodes-topics) and [Chapter 2: Services and Actions](/docs/module-1-ros2/services-actions)

**What You Will Learn**: Write production-quality ROS 2 Python code

---

## Learning Objectives

After completing this chapter, you will be able to:

- Create complete, executable ROS 2 Python nodes with proper structure
- Explain how executors schedule callbacks and choose the right executor type
- Use callback groups to enable concurrent execution
- Declare and use parameters for runtime configuration
- Implement proper logging, error handling, and graceful shutdown
- Structure ROS 2 Python packages following best practices

---

## The Anatomy of an rclpy Node

Every rclpy node follows the same basic structure. Understanding this structure is essential before exploring advanced features.

### The Minimal Node Pattern

```python
# Minimal structure of every rclpy node
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')  # Node name for the ROS graph
        # Initialize publishers, subscribers, timers here

def main(args=None):
    rclpy.init(args=args)               # Initialize ROS 2 client library
    node = MyNode()
    rclpy.spin(node)                     # Process callbacks until shutdown
    node.destroy_node()
    rclpy.shutdown()
```

The four essential steps are:

| Step | Function | Purpose |
|------|----------|---------|
| 1 | `rclpy.init()` | Initialize the ROS 2 communication layer |
| 2 | Create node | Instantiate your node class |
| 3 | `rclpy.spin()` | Process callbacks (blocks until shutdown) |
| 4 | `rclpy.shutdown()` | Clean up resources |

---

## Creating Publishers and Subscribers

Let us build a complete publisher and subscriber pair. These examples are fully executable.

### Complete Publisher Node

```python
# File: simple_publisher.py
# Prerequisites: ROS 2 Humble, std_msgs package
# Tested with: Python 3.10, ROS 2 Humble

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')

        # Create publisher: (message_type, topic_name, queue_size)
        self.publisher_ = self.create_publisher(String, 'chatter', 10)

        # Create timer: (period_seconds, callback)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.count = 0
        self.get_logger().info('Publisher node started')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.count}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # Handle Ctrl+C gracefully
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Expected Output:**
```
[INFO] [simple_publisher]: Publisher node started
[INFO] [simple_publisher]: Publishing: "Hello World: 0"
[INFO] [simple_publisher]: Publishing: "Hello World: 1"
```

### Complete Subscriber Node

```python
# File: simple_subscriber.py
# Prerequisites: ROS 2 Humble, std_msgs package
# Tested with: Python 3.10, ROS 2 Humble

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')

        # Create subscription: (msg_type, topic, callback, queue_size)
        self.subscription = self.create_subscription(
            String, 'chatter', self.listener_callback, 10)
        self.get_logger().info('Subscriber node started')

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Running the Nodes:**
```bash
# Terminal 1
python3 simple_publisher.py

# Terminal 2
python3 simple_subscriber.py
# Output: [INFO] [simple_subscriber]: I heard: "Hello World: 0"
```

---

## Understanding Executors

When you call `rclpy.spin(node)`, an **executor** processes callbacks. Understanding executors is critical for writing efficient robot code.

### What Is an Executor?

An executor is the engine that runs your callbacks. When a message arrives on a subscribed topic, the executor calls your callback function. When a timer fires, the executor calls your timer callback.

```
┌─────────────────────────────────────────────────────────────────┐
│                        EXECUTOR                                  │
│                                                                  │
│   Incoming Events:              Callbacks to Execute:            │
│   ┌─────────────┐               ┌─────────────────────┐         │
│   │ Message on  │──────────────▶│ subscriber_callback │         │
│   │ /sensor     │               └─────────────────────┘         │
│   └─────────────┘                                                │
│   ┌─────────────┐               ┌─────────────────────┐         │
│   │ Timer fires │──────────────▶│   timer_callback    │         │
│   └─────────────┘               └─────────────────────┘         │
│   ┌─────────────┐               ┌─────────────────────┐         │
│   │ Service req │──────────────▶│  service_callback   │         │
│   └─────────────┘               └─────────────────────┘         │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### SingleThreadedExecutor (Default)

The default executor processes callbacks **one at a time**. If a callback takes too long, other callbacks must wait.

```python
# This is what rclpy.spin() uses internally
from rclpy.executors import SingleThreadedExecutor

executor = SingleThreadedExecutor()
executor.add_node(node)
executor.spin()
```

**Behavior:**
```
Time ──────────────────────────────────────────────────────▶

│ callback_1 ████████│ callback_2 ████│ callback_3 ██████│
                     ▲
                     │
            callback_2 waits for callback_1 to finish
```

:::warning Blocking Callbacks
If `callback_1` takes 2 seconds, `callback_2` will not start for 2 seconds—even if its event occurred immediately. Never put blocking operations (like `time.sleep()` or long computations) in callbacks without considering the impact.
:::

### MultiThreadedExecutor

The multi-threaded executor can run callbacks **in parallel** using multiple threads.

```python
from rclpy.executors import MultiThreadedExecutor

executor = MultiThreadedExecutor(num_threads=4)
executor.add_node(node)
executor.spin()
```

**Behavior:**
```
Thread 1: │ callback_1 ████████│ callback_4 ████│
Thread 2: │ callback_2 ████│ callback_5 ██████████│
Thread 3: │ callback_3 ██████│
```

Callbacks from the same callback group cannot run concurrently by default. This brings us to callback groups.

---

## Callback Groups

Callback groups control which callbacks can run simultaneously in a multi-threaded executor.

### Two Types of Callback Groups

| Type | Behavior | Use Case |
|------|----------|----------|
| **MutuallyExclusiveCallbackGroup** | Only one callback runs at a time | Callbacks that share state |
| **ReentrantCallbackGroup** | Callbacks can run in parallel | Independent callbacks |

### Using Callback Groups

```python
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')

        # Group 1: sensor callbacks can run in parallel with each other
        self.sensor_group = ReentrantCallbackGroup()

        # Group 2: control callbacks run one at a time (they share motor state)
        self.control_group = MutuallyExclusiveCallbackGroup()

        # Assign subscriptions to groups
        self.camera_sub = self.create_subscription(
            Image, '/camera', self.camera_cb, 10,
            callback_group=self.sensor_group)

        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_cb, 10,
            callback_group=self.sensor_group)

        self.motor_sub = self.create_subscription(
            JointState, '/joints', self.motor_cb, 10,
            callback_group=self.control_group)
```

**Resulting Behavior:**
```
With MultiThreadedExecutor:
- camera_cb and lidar_cb CAN run at the same time (ReentrantCallbackGroup)
- motor_cb runs alone in its group (MutuallyExclusiveCallbackGroup)
- camera_cb and motor_cb CAN run at the same time (different groups)
```

:::tip When to Use Each Group
- **MutuallyExclusiveCallbackGroup**: When callbacks read/write shared data (default behavior)
- **ReentrantCallbackGroup**: When callbacks are independent and thread-safe
:::

---

## Parameters: Runtime Configuration

Parameters allow you to configure nodes without changing code. Parameters are essential for production robots where different environments require different settings.

### Declaring and Using Parameters

```python
class ConfigurableNode(Node):
    def __init__(self):
        super().__init__('configurable_node')

        # Declare parameters with default values
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('robot_name', 'my_robot')
        self.declare_parameter('enabled', True)

        # Get parameter values
        rate = self.get_parameter('publish_rate').value
        name = self.get_parameter('robot_name').value

        self.get_logger().info(f'Robot: {name}, Rate: {rate} Hz')

        # Create timer using parameter value
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)
```

### Setting Parameters from Command Line

```bash
# Set parameters when launching the node
ros2 run my_package my_node --ros-args -p publish_rate:=20.0 -p robot_name:="atlas"
```

### Setting Parameters from Launch File

```python
# In a launch file
Node(
    package='my_package',
    executable='my_node',
    parameters=[{
        'publish_rate': 20.0,
        'robot_name': 'atlas',
        'enabled': True
    }]
)
```

### Responding to Parameter Changes

Parameters can change at runtime. Use a callback to respond:

```python
from rcl_interfaces.msg import SetParametersResult

class DynamicNode(Node):
    def __init__(self):
        super().__init__('dynamic_node')
        self.declare_parameter('speed', 1.0)

        # Register callback for parameter changes
        self.add_on_set_parameters_callback(self.param_callback)

    def param_callback(self, params):
        for param in params:
            if param.name == 'speed':
                self.get_logger().info(f'Speed changed to: {param.value}')
        return SetParametersResult(successful=True)
```

**Changing Parameters at Runtime:**
```bash
# Change parameter while node is running
ros2 param set /dynamic_node speed 2.5
```

---

## Logging Best Practices

ROS 2 provides a logging system with severity levels. Proper logging is essential for debugging robot behavior.

### Log Levels

```python
# Available log levels (from least to most severe)
self.get_logger().debug('Detailed debugging info')   # Hidden by default
self.get_logger().info('Normal operational messages')
self.get_logger().warn('Something unexpected but not fatal')
self.get_logger().error('Something failed')
self.get_logger().fatal('Critical failure, node cannot continue')
```

### Throttled Logging

Avoid flooding logs with repeated messages:

```python
# Log at most once per second
self.get_logger().info('Sensor reading', throttle_duration_sec=1.0)

# Log only once (useful in loops)
self.get_logger().warn('Deprecated feature used', once=True)

# Skip the first N occurrences
self.get_logger().info('Connection attempt', skip_first=True)
```

### Conditional Logging

```python
# Only log expensive message if debug level is enabled
if self.get_logger().is_enabled_for(rclpy.logging.LoggingSeverity.DEBUG):
    expensive_data = self.compute_debug_info()
    self.get_logger().debug(f'Debug: {expensive_data}')
```

---

## Creating Services in rclpy

Services implement the request-response pattern from Chapter 2.

### Service Server

```python
# File: add_service.py
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class AddService(Node):
    def __init__(self):
        super().__init__('add_service')

        # Create service: (service_type, service_name, callback)
        self.srv = self.create_service(
            AddTwoInts, 'add_two_ints', self.add_callback)
        self.get_logger().info('Service ready')

    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'{request.a} + {request.b} = {response.sum}')
        return response
```

### Service Client (Asynchronous)

```python
# Asynchronous service call (recommended)
class AddClient(Node):
    def __init__(self):
        super().__init__('add_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        # call_async returns a Future object
        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        result = future.result()
        self.get_logger().info(f'Result: {result.sum}')
```

:::warning Avoid Synchronous Calls in Callbacks
Never use `client.call()` (synchronous) inside a callback—it will deadlock. Always use `call_async()` with futures.
:::

---

## Timers and Rates

Timers execute callbacks at regular intervals, essential for control loops and periodic tasks.

### Creating Timers

```python
class TimerNode(Node):
    def __init__(self):
        super().__init__('timer_node')

        # Timer at 10 Hz (0.1 second period)
        self.fast_timer = self.create_timer(0.1, self.fast_callback)

        # Timer at 1 Hz (1.0 second period)
        self.slow_timer = self.create_timer(1.0, self.slow_callback)

    def fast_callback(self):
        pass  # Called 10 times per second

    def slow_callback(self):
        pass  # Called once per second
```

### Rate Objects for Manual Timing

```python
# For loops that need timing (less common than timers)
rate = self.create_rate(10)  # 10 Hz

while rclpy.ok():
    self.do_work()
    rate.sleep()  # Sleep to maintain rate
```

---

## Graceful Shutdown

Proper shutdown handling prevents resource leaks and ensures clean termination.

### Handling Ctrl+C

```python
def main(args=None):
    rclpy.init(args=args)
    node = MyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down')
    finally:
        # Cleanup: cancel timers, close connections, save state
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()
```

### Context Managers (Python 3.10+)

```python
def main(args=None):
    rclpy.init(args=args)

    with MyNode() as node:  # Requires __enter__ and __exit__ methods
        rclpy.spin(node)

    rclpy.shutdown()
```

---

## Complete Example: Sensor Processing Node

This example combines publishers, subscribers, parameters, logging, and proper shutdown into a realistic sensor processing node.

```python
# File: sensor_processor.py
# A complete, production-style ROS 2 node
# Prerequisites: ROS 2 Humble, sensor_msgs, std_msgs
# Tested with: Python 3.10, ROS 2 Humble

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

class SensorProcessor(Node):
    def __init__(self):
        super().__init__('sensor_processor')

        # Declare parameters
        self.declare_parameter('min_range', 0.1)
        self.declare_parameter('max_range', 10.0)

        # Get parameters
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value

        # Callback group for concurrent sensor processing
        self.sensor_group = ReentrantCallbackGroup()

        # Subscriber for laser scan data
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10,
            callback_group=self.sensor_group)

        # Publisher for processed minimum distance
        self.min_dist_pub = self.create_publisher(Float32, '/min_distance', 10)

        # Diagnostics timer
        self.create_timer(5.0, self.diagnostics_callback)

        self.scan_count = 0
        self.get_logger().info(
            f'Sensor processor started (range: {self.min_range}-{self.max_range}m)')

    def scan_callback(self, msg):
        # Filter valid ranges
        valid_ranges = [
            r for r in msg.ranges
            if self.min_range < r < self.max_range
        ]

        if valid_ranges:
            min_distance = min(valid_ranges)

            # Publish minimum distance
            out_msg = Float32()
            out_msg.data = min_distance
            self.min_dist_pub.publish(out_msg)

            # Warn if obstacle is close
            if min_distance < 0.5:
                self.get_logger().warn(
                    f'Close obstacle: {min_distance:.2f}m',
                    throttle_duration_sec=1.0)

        self.scan_count += 1

    def diagnostics_callback(self):
        self.get_logger().info(
            f'Processed {self.scan_count} scans')

def main(args=None):
    rclpy.init(args=args)
    node = SensorProcessor()

    # Use multi-threaded executor for concurrent callbacks
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Running and Testing:**
```bash
# Terminal 1: Run the node with custom parameters
ros2 run my_package sensor_processor --ros-args -p min_range:=0.05 -p max_range:=5.0

# Terminal 2: Publish fake scan data for testing
ros2 topic pub /scan sensor_msgs/LaserScan "{ranges: [1.0, 0.3, 2.5, 0.8]}"

# Terminal 3: Monitor output
ros2 topic echo /min_distance
```

---

## Hands-On Exercise

### Exercise 1: Temperature Monitor Node

**Objective:** Build a complete temperature monitoring system with publisher, subscriber, and parameter configuration.

**Difficulty:** Intermediate

**Instructions:**

1. Create a `TemperatureSensor` node that publishes random temperature values to `/temperature`
2. Create a `TemperatureMonitor` node that subscribes and logs warnings when temperature exceeds a threshold
3. Make the warning threshold configurable via a parameter
4. Use throttled logging to avoid log spam

**Starter Code:**

```python
# File: temperature_sensor.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random

class TemperatureSensor(Node):
    def __init__(self):
        super().__init__('temperature_sensor')
        # TODO: Create publisher to '/temperature' topic
        # TODO: Create timer to publish at 1 Hz
        # TODO: Publish random temperature between 15.0 and 35.0
        pass

def main(args=None):
    # TODO: Initialize rclpy, create node, spin, shutdown
    pass
```

```python
# File: temperature_monitor.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class TemperatureMonitor(Node):
    def __init__(self):
        super().__init__('temperature_monitor')
        # TODO: Declare parameter 'warning_threshold' with default 30.0
        # TODO: Create subscription to '/temperature'
        # TODO: In callback, log warning if temp > threshold (throttled)
        pass

def main(args=None):
    # TODO: Initialize rclpy, create node, spin, shutdown
    pass
```

<details>
<summary>Click to reveal solution</summary>

```python
# File: temperature_sensor.py (Solution)
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random

class TemperatureSensor(Node):
    def __init__(self):
        super().__init__('temperature_sensor')
        self.publisher_ = self.create_publisher(Float64, '/temperature', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Temperature sensor started')

    def timer_callback(self):
        msg = Float64()
        msg.data = random.uniform(15.0, 35.0)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Temperature: {msg.data:.1f}°C')

def main(args=None):
    rclpy.init(args=args)
    node = TemperatureSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

```python
# File: temperature_monitor.py (Solution)
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class TemperatureMonitor(Node):
    def __init__(self):
        super().__init__('temperature_monitor')
        self.declare_parameter('warning_threshold', 30.0)
        self.threshold = self.get_parameter('warning_threshold').value

        self.subscription = self.create_subscription(
            Float64, '/temperature', self.temp_callback, 10)
        self.get_logger().info(f'Monitor started (threshold: {self.threshold}°C)')

    def temp_callback(self, msg):
        if msg.data > self.threshold:
            self.get_logger().warn(
                f'HIGH TEMP: {msg.data:.1f}°C exceeds {self.threshold}°C',
                throttle_duration_sec=2.0)
        else:
            self.get_logger().info(f'Normal: {msg.data:.1f}°C')

def main(args=None):
    rclpy.init(args=args)
    node = TemperatureMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

</details>

**Verification:**
```bash
# Run both nodes and verify:
# 1. Temperatures are published to /temperature
# 2. Warnings appear when temp > 30 (or your custom threshold)
# 3. Warnings are throttled (not every message)
ros2 param set /temperature_monitor warning_threshold 25.0
# Verify threshold change takes effect
```

---

## Summary

### Key Takeaways

1. **Node structure is consistent** - Initialize rclpy, create node, spin, shutdown. This pattern applies to every ROS 2 Python node.

2. **Executors control callback scheduling** - SingleThreadedExecutor processes one callback at a time; MultiThreadedExecutor enables parallelism.

3. **Callback groups enable concurrency** - Use ReentrantCallbackGroup for independent callbacks, MutuallyExclusiveCallbackGroup for callbacks sharing state.

4. **Parameters make nodes configurable** - Declare parameters with defaults, allow runtime changes, and use callbacks to respond to changes.

5. **Logging has levels and throttling** - Use appropriate severity levels and throttle repeated messages to keep logs useful.

### Code Pattern Quick Reference

```python
# Publisher
self.pub = self.create_publisher(MsgType, 'topic', 10)
self.pub.publish(msg)

# Subscriber
self.sub = self.create_subscription(MsgType, 'topic', self.callback, 10)

# Timer
self.timer = self.create_timer(period_sec, self.callback)

# Service Server
self.srv = self.create_service(SrvType, 'service', self.callback)

# Service Client (async)
self.client = self.create_client(SrvType, 'service')
future = self.client.call_async(request)

# Parameter
self.declare_parameter('name', default_value)
value = self.get_parameter('name').value
```

---

## What's Next

Now that you can write production-quality Python nodes, the next chapter introduces **URDF (Unified Robot Description Format)**. URDF defines your robot's physical structure—links, joints, and sensors—for visualization in RViz and simulation in Gazebo.

[Next: Chapter 4 - Robot Description (URDF) →](/docs/module-1-ros2/urdf)

---

## Additional Resources

- [rclpy API Reference](https://docs.ros2.org/latest/api/rclpy/) - Complete Python API documentation
- [ROS 2 Tutorials: Writing Nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html) - Official tutorial
- [Executors Documentation](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Executors.html) - Deep dive into executor models
- [Parameters Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html) - Parameter handling guide
