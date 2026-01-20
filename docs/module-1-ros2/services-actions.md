---
sidebar_position: 3
title: "Chapter 2: Services and Actions"
description: "Learn ROS 2 services for request-response communication and actions for long-running tasks with feedback and cancellation."
keywords: [ROS 2, services, actions, request-response, feedback, client, server, robotics]
---

# Chapter 2: Services and Actions

## Chapter Overview

In Chapter 1, you learned that topics use a publish-subscribe pattern where publishers send data without expecting a response. But what if you need an answer? What if you need to ask the robot a question and wait for a reply? This chapter introduces two additional communication patterns: **services** for quick question-answer exchanges, and **actions** for long-running tasks that provide progress updates.

**Prerequisites**: Completed [Chapter 1: Nodes and Topics](/docs/module-1-ros2/nodes-topics)

**What You Will Learn**: When and why to use services and actions instead of topics

---

## Learning Objectives

After completing this chapter, you will be able to:

- Explain why topics alone are not enough for all robot communication
- Describe the request-response pattern used by services
- Describe the goal-feedback-result pattern used by actions
- Choose the right communication pattern for a given task
- Recognize services and actions when inspecting a ROS 2 system

---

## Why Do We Need More Than Topics?

Recall from Chapter 1: topics are like bulletin boards. A node posts a message, and anyone interested can read it. The poster does not wait for a response and does not know if anyone read the message.

This works great for streaming data:

- Camera publishes images continuously → works with topics
- Lidar publishes distance scans repeatedly → works with topics
- Robot publishes its position every 100ms → works with topics

But some situations need a different approach:

| Situation | Why Topics Don't Work |
|-----------|----------------------|
| "What is the robot's current battery level?" | You need ONE answer, not a continuous stream |
| "Take a photo and give it to me" | You need confirmation the photo was taken |
| "Navigate to the kitchen" | This takes time; you need progress updates |
| "Pick up the cup" | You might need to cancel if something goes wrong |

ROS 2 provides two additional patterns for these situations: **Services** and **Actions**.

---

## What Is a Service?

A **service** is a request-response communication pattern. One node asks a question, another node answers it.

### The Restaurant Analogy

Think of ordering food at a restaurant counter:

```
┌─────────────┐                              ┌─────────────┐
│   Customer  │                              │   Kitchen   │
│   (Client)  │                              │  (Server)   │
└──────┬──────┘                              └──────┬──────┘
       │                                            │
       │  "I'd like a burger, please"              │
       │  ─────────────────────────────────────►   │
       │            (REQUEST)                       │
       │                                            │
       │         Kitchen prepares food              │
       │                                            │
       │   "Here's your burger"                     │
       │  ◄─────────────────────────────────────   │
       │           (RESPONSE)                       │
       │                                            │
```

Key characteristics:

- **Synchronous**: Customer waits until they get the food
- **One-to-one**: One request produces one response
- **Blocking**: Customer cannot do other things while waiting

### Service vs Topic Comparison

| Aspect | Topic | Service |
|--------|-------|---------|
| Pattern | Publish-subscribe | Request-response |
| Direction | One-way | Two-way |
| Waiting | Publisher does not wait | Client waits for response |
| Use case | Continuous data | One-time queries |

### When to Use Services

Services are ideal for:

- **Quick queries**: "What is the battery level?" "What time is it?"
- **Configuration changes**: "Set the motor speed to 50%"
- **Triggering instant actions**: "Take a photo now"
- **Calculations**: "Convert these coordinates"

Services are NOT ideal for:

- **Long operations**: Navigation, pick-and-place (use Actions)
- **Continuous data**: Sensor streams (use Topics)

:::tip Rule of Thumb
If the operation completes in less than a second, use a Service. If it takes longer or needs progress updates, use an Action.
:::

---

## Service Structure: Request and Response

Every service has a defined **interface** with two parts:

```
┌─────────────────────────────────────────┐
│           SERVICE INTERFACE             │
├─────────────────────────────────────────┤
│  REQUEST (what the client sends):       │
│    - input data                         │
│    - parameters                         │
├─────────────────────────────────────────┤
│  RESPONSE (what the server returns):    │
│    - result data                        │
│    - success/failure status             │
└─────────────────────────────────────────┘
```

### Example: AddTwoInts Service

```
Request:
  int64 a
  int64 b
---
Response:
  int64 sum
```

The `---` separates the request from the response. The client sends two numbers (`a` and `b`), and the server returns their `sum`.

### Common Built-in Services

| Service Type | Request | Response | Use Case |
|--------------|---------|----------|----------|
| `std_srvs/Empty` | (nothing) | (nothing) | Trigger an action |
| `std_srvs/SetBool` | `bool data` | `bool success, string message` | Enable/disable |
| `std_srvs/Trigger` | (nothing) | `bool success, string message` | Trigger with feedback |

---

## Service Roles: Client and Server

Two roles participate in service communication:

```
┌──────────────────┐                    ┌──────────────────┐
│  SERVICE CLIENT  │                    │  SERVICE SERVER  │
│                  │                    │                  │
│  - Sends request │  ──── REQUEST ───► │  - Receives req  │
│  - Waits         │                    │  - Processes     │
│  - Gets response │  ◄─── RESPONSE ─── │  - Sends response│
└──────────────────┘                    └──────────────────┘
```

| Role | What It Does | Analogy |
|------|--------------|---------|
| **Server** | Waits for requests, processes them, sends responses | Restaurant kitchen |
| **Client** | Sends a request, waits for a response | Customer ordering food |

A service has exactly **one server** but can have **multiple clients**. If two customers both order food, the kitchen handles them one at a time.

---

## What Is an Action?

An **action** is for long-running tasks that need progress feedback and can be cancelled.

### The Pizza Delivery Analogy

Think of ordering pizza delivery:

```
┌─────────────┐                              ┌─────────────┐
│   Customer  │                              │   Pizzeria  │
│   (Client)  │                              │  (Server)   │
└──────┬──────┘                              └──────┬──────┘
       │                                            │
       │  "I'd like a large pepperoni"              │
       │  ─────────────────────────────────────►   │
       │              (GOAL)                        │
       │                                            │
       │  "Order received, preparing..."            │
       │  ◄─────────────────────────────────────   │
       │          (GOAL ACCEPTED)                   │
       │                                            │
       │  "Pizza in the oven..."                    │
       │  ◄─────────────────────────────────────   │
       │            (FEEDBACK)                      │
       │                                            │
       │  "Driver on the way, ETA 10 min..."        │
       │  ◄─────────────────────────────────────   │
       │            (FEEDBACK)                      │
       │                                            │
       │  "Delivered! Enjoy your pizza."            │
       │  ◄─────────────────────────────────────   │
       │             (RESULT)                       │
```

Key characteristics:

- **Asynchronous**: Customer can do other things while waiting
- **Feedback**: Regular updates on progress
- **Cancellable**: Customer can cancel the order
- **Long-running**: Task takes significant time

### Action vs Service Comparison

| Aspect | Service | Action |
|--------|---------|--------|
| Duration | Short (< 1 second) | Long (seconds to minutes) |
| Feedback | None (just final result) | Continuous progress updates |
| Cancellation | Not supported | Built-in cancellation |
| Client waiting | Blocks until done | Can continue other work |

---

## Action Structure: Goal, Feedback, Result

Every action has three parts:

```
┌─────────────────────────────────────────┐
│           ACTION INTERFACE              │
├─────────────────────────────────────────┤
│  GOAL (what the client wants):          │
│    - target position                    │
│    - parameters for the task            │
├─────────────────────────────────────────┤
│  FEEDBACK (progress updates):           │
│    - current progress                   │
│    - estimated time remaining           │
├─────────────────────────────────────────┤
│  RESULT (final outcome):                │
│    - success/failure                    │
│    - final data                         │
└─────────────────────────────────────────┘
```

### Example: NavigateToPose Action

```
Goal:
  geometry_msgs/PoseStamped pose    # Where to go
---
Feedback:
  float32 distance_remaining        # How far left
  float32 estimated_time            # ETA
---
Result:
  bool success                      # Did we arrive?
  string message                    # Details
```

---

## Action Lifecycle

An action goes through several states:

```
    ┌─────────┐
    │  GOAL   │  Client sends goal to server
    │  SENT   │
    └────┬────┘
         │
         ▼
    ┌─────────┐
    │ ACCEPTED│  Server agrees to work on it
    │   or    │  (or REJECTED if busy/invalid)
    │REJECTED │
    └────┬────┘
         │
         ▼
    ┌─────────┐
    │EXECUTING│  Server works, sends feedback
    │    +    │  ← ← ← FEEDBACK LOOP
    │FEEDBACK │
    └────┬────┘
         │
    ┌────┴────┐
    │         │
    ▼         ▼
┌───────┐ ┌────────┐
│SUCCESS│ │CANCELED│  Client can cancel anytime
└───────┘ └────────┘
```

The ability to **cancel** is critical for robotics. If a robot is navigating to the kitchen and you suddenly need it in the living room, you can cancel the current goal and send a new one.

---

## When to Use Each Pattern

Choosing the right communication pattern can be confusing at first. This decision guide helps you pick the correct one by asking three simple questions.

### The Decision Flowchart

```
                    ┌─────────────────────┐
                    │  Do you need data   │
                    │    continuously?    │
                    └──────────┬──────────┘
                               │
              ┌────────────────┴────────────────┐
              │ YES                             │ NO
              ▼                                 ▼
        ┌──────────┐                  ┌─────────────────┐
        │  TOPIC   │                  │ Do you need a   │
        │          │                  │    response?    │
        └──────────┘                  └────────┬────────┘
                                               │
                              ┌────────────────┴────────────────┐
                              │ NO                              │ YES
                              ▼                                 ▼
                        ┌──────────┐               ┌─────────────────┐
                        │  TOPIC   │               │ Is it quick     │
                        │(one-way) │               │ (< 1 second)?   │
                        └──────────┘               └────────┬────────┘
                                                           │
                                          ┌────────────────┴────────────────┐
                                          │ YES                             │ NO
                                          ▼                                 ▼
                                    ┌──────────┐                      ┌──────────┐
                                    │ SERVICE  │                      │  ACTION  │
                                    └──────────┘                      └──────────┘
```

### Walking Through the Flowchart

Let us follow this flowchart step by step:

**Step 1: "Do you need data continuously?"**

This is your first question. Think about whether you need a stream of data that keeps coming, or just a single piece of information.

| If YES (continuous data) | If NO (one-time or occasional) |
|--------------------------|-------------------------------|
| Camera images streaming at 30 fps | "What is the battery level right now?" |
| Lidar scans every 100ms | "Navigate to the kitchen" |
| Robot position updates | "Take one photo" |

- **YES → Use TOPIC.** Topics are designed for continuous data streams. The publisher keeps sending, subscribers keep receiving.
- **NO → Go to Step 2.**

---

**Step 2: "Do you need a response?"**

If you do not need continuous data, ask yourself: do you need confirmation or an answer back?

| If YES (need response) | If NO (fire and forget) |
|------------------------|------------------------|
| "Calculate the distance" → need the result | "Log this message" → do not care about confirmation |
| "Is the gripper open?" → need yes/no | "Play a sound" → just trigger it |
| "Move to position X" → need to know when done | "Update the display" → one-way notification |

- **NO → Use TOPIC (one-way).** Sometimes topics are used for one-way commands where you do not need acknowledgment.
- **YES → Go to Step 3.**

---

**Step 3: "Is it quick (less than 1 second)?"**

If you need a response, the final question is: how long will it take?

| If YES (quick, < 1 second) | If NO (long, > 1 second) |
|---------------------------|-------------------------|
| "What is 5 + 3?" → instant calculation | "Navigate to the kitchen" → takes 30 seconds |
| "Get battery level" → instant read | "Pick up the cup" → takes 5 seconds |
| "Set motor speed to 50" → instant config | "Scan the room" → takes 10 seconds |

- **YES → Use SERVICE.** Quick operations that return immediately work well with services. The client sends a request, waits briefly, and gets a response.
- **NO → Use ACTION.** Long operations need actions because:
  - You want progress updates ("50% complete...")
  - You might need to cancel midway
  - You do not want to block the client for seconds or minutes

---

### Applying the Flowchart: Examples

Let us practice with real scenarios:

**Example 1: "Stream camera images to a vision node"**
- Step 1: Do I need data continuously? **YES** (images at 30 fps)
- **Answer: TOPIC**

**Example 2: "Ask the robot for its current battery percentage"**
- Step 1: Do I need data continuously? **NO** (just once)
- Step 2: Do I need a response? **YES** (I want the percentage)
- Step 3: Is it quick? **YES** (reading a sensor is instant)
- **Answer: SERVICE**

**Example 3: "Tell the robot to navigate to the kitchen"**
- Step 1: Do I need data continuously? **NO** (one command)
- Step 2: Do I need a response? **YES** (I want to know when it arrives)
- Step 3: Is it quick? **NO** (navigation takes 30+ seconds)
- **Answer: ACTION**

**Example 4: "Send a velocity command to make the robot move"**
- Step 1: Do I need data continuously? **YES** (velocity commands sent at 10 Hz to keep moving)
- **Answer: TOPIC**

**Example 5: "Trigger the robot to take a single photo"**
- Step 1: Do I need data continuously? **NO** (one photo)
- Step 2: Do I need a response? **YES** (I want the photo or confirmation)
- Step 3: Is it quick? **YES** (taking a photo is instant)
- **Answer: SERVICE**

---

### Quick Reference Table

| Task | Pattern | Why |
|------|---------|-----|
| Stream camera images | Topic | Continuous data |
| Get battery percentage | Service | Quick query, one response |
| Set motor speed | Service | Configuration change |
| Navigate to a location | Action | Long task, need progress |
| Pick up an object | Action | Long task, might cancel |
| Publish robot position | Topic | Continuous updates |
| Calculate path length | Service | Quick computation |

---

## Seeing Services and Actions in ROS 2

Use CLI tools to inspect services and actions on a running system:

```bash
# List all services
ros2 service list
# Output: /get_parameters, /set_parameters, ...

# Get info about a service (type)
ros2 service type /get_parameters
# Output: rcl_interfaces/srv/GetParameters

# Call a service from command line
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"
# Output: sum: 8
```

```bash
# List all actions
ros2 action list
# Output: /navigate_to_pose, /follow_path, ...

# Get info about an action
ros2 action info /navigate_to_pose
# Output: Action servers: 1, Action clients: 0

# Send a goal (if you have a navigation stack running)
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: ...}"
```

---

## Minimal Code Examples

Now that you understand the concepts, here is the essential code structure.

**Service Server** (answers requests):

```python
# Create service that adds two numbers
self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.callback)

def callback(self, request, response):
    response.sum = request.a + request.b  # Do the work
    return response                        # Send answer back
```

**Service Client** (asks questions):

```python
# Create client and send request
self.client = self.create_client(AddTwoInts, 'add_two_ints')
request = AddTwoInts.Request()
request.a = 5
request.b = 3
future = self.client.call_async(request)  # Send and wait
```

:::tip Focus on Concepts
The code syntax will become familiar with practice. What matters now is understanding *when* to use services vs topics vs actions.
:::

---

## Summary

### Key Takeaways

1. **Topics are for streaming data** - Continuous, one-way communication without waiting for responses.

2. **Services are for quick request-response** - Ask a question, get an answer, typically under 1 second.

3. **Actions are for long-running tasks** - Support progress feedback and cancellation, used for operations taking seconds to minutes.

4. **Choose the right pattern** - Use the decision flowchart to pick the appropriate communication method.

5. **CLI tools help you explore** - Use `ros2 service list` and `ros2 action list` to discover what's available.

### Mental Model Checklist

Before moving on, make sure you can answer:

- [ ] What is the difference between a service and a topic?
- [ ] When would you use an action instead of a service?
- [ ] Why is cancellation important for actions?
- [ ] In the restaurant analogy, who is the client and who is the server?

### Communication Patterns Summary

```
┌─────────────────────────────────────────────────────────────┐
│                  ROS 2 COMMUNICATION                        │
├─────────────────────────────────────────────────────────────┤
│  TOPIC     │  Bulletin board   │  Streaming data            │
│  SERVICE   │  Restaurant order │  Quick questions           │
│  ACTION    │  Pizza delivery   │  Long tasks with updates   │
└─────────────────────────────────────────────────────────────┘
```

---

## What's Next

Now that you understand all three communication patterns, the next chapter dives deeper into **rclpy**, the Python client library. You will learn how to write complete, production-quality nodes that use topics, services, and actions together.

[Next: Chapter 3 - Python Client Library (rclpy) →](/docs/module-1-ros2/rclpy)

---

## Additional Resources

- [ROS 2 Concepts: Services](https://docs.ros.org/en/humble/Concepts/Basic/About-Services.html) - Official documentation
- [ROS 2 Concepts: Actions](https://docs.ros.org/en/humble/Concepts/Basic/About-Actions.html) - Official documentation
- [Service Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html) - Hands-on practice
