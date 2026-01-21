---
sidebar_position: 2
title: "Chapter 1: Nodes and Topics"
description: "Learn the fundamental building blocks of ROS 2 - nodes as modular processes and topics as publish-subscribe communication channels."
keywords: [ROS 2, nodes, topics, publish, subscribe, publisher, subscriber, DDS, middleware, rclpy]
---

# Chapter 1: Nodes and Topics

## Chapter Overview

Before writing any code, you need to understand two fundamental concepts that form the backbone of every ROS 2 application: **nodes** and **topics**. This chapter focuses on building a strong mental model of these concepts through real-world analogies and visual explanations. By the end, you will understand not just *what* nodes and topics are, but *why* ROS 2 is designed this way.

**Prerequisites**: Completed the [Module 1 Introduction](/docs/module-1-ros2), basic Python knowledge

**What You Will Learn**: The conceptual foundation for all ROS 2 development

---

## Learning Objectives

After completing this chapter, you will be able to:

- Explain what a node is using real-world analogies
- Describe why robots use multiple nodes instead of one big program
- Explain the publish-subscribe communication pattern
- Understand how topics connect nodes together
- Recognize when to use topics versus other communication methods

---

## Why Does a Robot Need Multiple Programs?

Imagine building a humanoid robot. This robot needs to:

- Process camera images to see the world
- Listen to voice commands
- Control 20+ motors in its arms and legs
- Maintain balance while walking
- Plan paths to navigate around obstacles

**The naive approach** would be to write one giant program that does everything. But this creates serious problems:

| Problem | Consequence |
|---------|-------------|
| One bug crashes everything | Camera failure stops the entire robot |
| Hard to develop | Team members cannot work independently |
| Hard to test | Must run the whole system to test one feature |
| Hard to reuse | Cannot use your camera code in another robot |

**The ROS 2 approach** is to split the robot's brain into many small, independent programs. Each program handles one specific job. If the camera program crashes, the balance program keeps running—the robot does not fall over.

---

## What Is a Node?

A **node** is a single program that does one specific job.

Think of nodes like workers in a factory:

```
┌─────────────────────────────────────────────────────────────┐
│                      ROBOT FACTORY                          │
│                                                             │
│   ┌─────────┐    ┌─────────┐    ┌─────────┐    ┌─────────┐ │
│   │ Camera  │    │  Voice  │    │ Balance │    │  Motor  │ │
│   │ Worker  │    │ Worker  │    │ Worker  │    │ Worker  │ │
│   │ (Node)  │    │ (Node)  │    │ (Node)  │    │ (Node)  │ │
│   └─────────┘    └─────────┘    └─────────┘    └─────────┘ │
│                                                             │
│   Each worker does ONE job and communicates with others     │
└─────────────────────────────────────────────────────────────┘
```

Each worker (node) is:

- **Independent**: Can start, stop, or restart without affecting others
- **Focused**: Does one job well instead of many jobs poorly
- **Replaceable**: Can swap out the camera worker for a better one

### Real-World Example

Consider a self-driving car. It might have these nodes:

| Node Name | Job |
|-----------|-----|
| `lidar_driver` | Reads data from the laser scanner |
| `camera_driver` | Captures images from cameras |
| `object_detector` | Identifies cars, pedestrians, signs |
| `path_planner` | Decides where to drive |
| `motor_controller` | Sends commands to steering and throttle |

Each node is a separate program. They all run at the same time, working together to drive the car.

---

## How Do Nodes Communicate?

If nodes are separate programs, how do they share information? They need to talk to each other.

Imagine our factory workers again. The Camera Worker sees an obstacle. How does the Motor Worker know to stop?

**Option 1: Direct Communication (Bad)**

```
Camera Worker walks over to Motor Worker and taps their shoulder.

Problem: What if Motor Worker is busy? What if there are 10 workers
who need to know about obstacles? Camera Worker spends all day
walking around instead of watching for obstacles.
```

**Option 2: Bulletin Board System (Good)**

```
Camera Worker posts a note on a bulletin board labeled "OBSTACLES".
Any worker who cares about obstacles checks that board.

Benefits:
- Camera Worker posts once, everyone who needs it can read
- Workers don't need to know who else exists
- New workers can start reading the board anytime
```

ROS 2 uses Option 2. The "bulletin boards" are called **topics**.

---

## What Is a Topic?

A **topic** is a named channel where nodes post and read messages.

```
┌──────────────┐                              ┌──────────────┐
│    Camera    │                              │    Motor     │
│     Node     │                              │     Node     │
└──────┬───────┘                              └──────▲───────┘
       │                                             │
       │ POSTS message                     READS message
       │                                             │
       ▼                                             │
┌──────────────────────────────────────────────────────────────┐
│                                                              │
│                   TOPIC: "/obstacle_detected"                │
│                                                              │
│              [Message: "obstacle at 2 meters"]               │
│                                                              │
└──────────────────────────────────────────────────────────────┘
```

Key terminology:

| Term | Meaning | Analogy |
|------|---------|---------|
| **Topic** | A named channel for messages | A labeled bulletin board |
| **Publisher** | A node that posts messages | Worker who writes notes |
| **Subscriber** | A node that reads messages | Worker who reads notes |
| **Message** | The data being shared | The note itself |

### The Publish-Subscribe Pattern

This communication style is called **publish-subscribe** (pub-sub):

1. Publishers post messages to a topic (they don't care who reads them)
2. Subscribers listen to a topic (they don't care who posted)
3. Publishers and subscribers never talk directly to each other

This **decoupling** is powerful:

- Add new subscribers without changing the publisher
- Replace a publisher without changing subscribers
- Multiple publishers can post to the same topic
- Multiple subscribers can read from the same topic

---

## How Data Flows Through Topics

Let's trace how sensor data becomes robot movement:

```
┌─────────────┐         ┌─────────────┐         ┌─────────────┐
│   Camera    │         │  Obstacle   │         │   Motor     │
│    Node     │         │  Detector   │         │ Controller  │
└──────┬──────┘         └──────┬──────┘         └──────┬──────┘
       │                       │                       │
       │ publishes             │ subscribes            │ subscribes
       │ images                │ to images             │ to commands
       ▼                       │                       │
  ┌─────────┐                  │                       │
  │ /camera │ ─────────────────┘                       │
  │ /image  │                                          │
  └─────────┘                  │                       │
                               │ publishes             │
                               │ commands              │
                               ▼                       │
                          ┌─────────┐                  │
                          │  /cmd   │ ─────────────────┘
                          │  /vel   │
                          └─────────┘
```

**Step by step:**

1. Camera Node captures an image and **publishes** it to `/camera/image`
2. Obstacle Detector Node **subscribes** to `/camera/image` and receives the image
3. Obstacle Detector processes the image, detects an obstacle
4. Obstacle Detector **publishes** "slow down" command to `/cmd/vel`
5. Motor Controller **subscribes** to `/cmd/vel` and receives the command
6. Motor Controller slows down the robot

Notice: Camera Node has no idea Motor Controller exists. They are completely independent.

---

## Topic Names and Message Types

### Topic Names

Topics have names that look like file paths:

| Topic Name | Purpose |
|------------|---------|
| `/camera/image` | Camera images |
| `/cmd_vel` | Velocity commands |
| `/scan` | Laser scan data |
| `/odom` | Robot position estimate |

The `/` creates a hierarchy. `/camera/image` and `/camera/depth` are both under `/camera`.

### Message Types

Every topic has a **message type** that defines what data it carries. Think of it as the format of the notes on the bulletin board.

| Message Type | Contains | Example Use |
|--------------|----------|-------------|
| `std_msgs/String` | A text string | Simple text messages |
| `std_msgs/Float64` | A decimal number | Temperature readings |
| `geometry_msgs/Twist` | Linear and angular velocity | Robot movement commands |
| `sensor_msgs/Image` | Camera image data | Vision processing |

**Important**: All publishers and subscribers on a topic must use the same message type. You cannot post a number to a topic expecting text.

---

## When to Use Topics

Topics are ideal for:

- **Continuous data streams**: Sensor readings, camera images, robot position
- **One-to-many communication**: One sensor, many consumers
- **Fire-and-forget**: Publisher does not need confirmation

Topics are NOT ideal for:

- **Request-response**: "Give me the current map" (use Services instead)
- **Long-running tasks**: "Navigate to the kitchen" (use Actions instead)

You will learn about Services and Actions in Chapter 2.

---

## Seeing Nodes and Topics in Action

Before writing code, let's observe a running ROS 2 system using command-line tools.

Open a terminal and run the built-in demo:

```bash
# Terminal 1: Start a demo publisher
ros2 run demo_nodes_cpp talker
```

```bash
# Terminal 2: Start a demo subscriber
ros2 run demo_nodes_cpp listener
```

Now inspect the system:

```bash
# List all running nodes
ros2 node list
# Output: /talker, /listener

# List all active topics
ros2 topic list
# Output: /chatter, /parameter_events, /rosout

# See messages flowing on a topic
ros2 topic echo /chatter
# Output: data: 'Hello World: 1', data: 'Hello World: 2', ...
```

You just observed:
- Two nodes (`talker` and `listener`) running independently
- The `talker` publishing to the `/chatter` topic
- The `listener` subscribing to `/chatter`

---

## Your First Publisher and Subscriber

Now that you understand the concepts, here is the minimal code to create your own.

**Publisher** (posts messages):

```python
# Key parts of a publisher node
self.publisher_ = self.create_publisher(String, 'chatter', 10)
# Args: message type, topic name, queue size

msg = String()
msg.data = 'Hello World'
self.publisher_.publish(msg)
```

**Subscriber** (reads messages):

```python
# Key parts of a subscriber node
self.subscription = self.create_subscription(
    String, 'chatter', self.callback_function, 10)
# Args: message type, topic name, callback, queue size

def callback_function(self, msg):
    print(f'I heard: {msg.data}')
```

:::tip Focus on Concepts First
Do not worry about memorizing the code syntax. The important thing is understanding *why* we have publishers and subscribers. The syntax becomes natural with practice.
:::

---

## Summary

### Key Takeaways

1. **Nodes are independent programs** - Each node does one job. If one crashes, others keep running.

2. **Topics are communication channels** - Named bulletin boards where nodes post and read messages.

3. **Publish-subscribe decouples nodes** - Publishers and subscribers do not know about each other. This makes the system flexible and modular.

4. **Messages have types** - All communication on a topic uses the same message format.

5. **ROS 2 CLI tools let you inspect the system** - Use `ros2 node list`, `ros2 topic list`, and `ros2 topic echo` to see what's happening.

### Mental Model Checklist

Before moving on, make sure you can answer:

- [ ] Why do robots use many small programs instead of one big program?
- [ ] What is the difference between a node and a topic?
- [ ] What does "publish" mean? What does "subscribe" mean?
- [ ] Why is it good that publishers don't know about subscribers?

---

## What's Next

Now that you understand nodes and topics conceptually, the next chapter introduces **Services and Actions**—different communication patterns for when you need request-response or long-running tasks.

[Next: Chapter 2 - Services and Actions →](/docs/module-1-ros2/services-actions)

---

## Additional Resources

- [ROS 2 Concepts: Nodes](https://docs.ros.org/en/humble/Concepts/Basic/About-Nodes.html) - Official documentation
- [ROS 2 Concepts: Topics](https://docs.ros.org/en/humble/Concepts/Basic/About-Topics.html) - Official documentation
- [ROS 2 CLI Tools Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html) - Hands-on practice
