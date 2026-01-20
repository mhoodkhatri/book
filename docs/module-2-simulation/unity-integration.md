---
sidebar_position: 4
title: "Chapter 3: Unity Integration"
description: "Integrate Unity game engine with ROS 2 for photo-realistic rendering, complex environments, and advanced simulation scenarios in robotics."
keywords: [Unity, ROS 2, simulation, rendering, environments, robotics, game engine, visualization]
---

# Chapter 3: Unity Integration

## Chapter Overview

In the previous chapters, you learned to use Gazebo for robot simulation. Gazebo excels at physics simulation, but what if you need **photo-realistic graphics**? What if you want to test your robot in a virtual shopping mall, warehouse, or hospital—environments that are complex to build from scratch?

This is where **Unity** shines. Unity is a professional game engine used to create video games, architectural visualizations, and training simulations. It provides:

- **Photo-realistic rendering** with advanced lighting, shadows, and materials
- **Vast asset ecosystem** with thousands of pre-built 3D models
- **Procedural generation** tools for creating infinite environment variations
- **Cross-platform support** for running simulations anywhere

Think of Unity as a **movie studio for your robot**. While Gazebo is like a physics laboratory where you test how things work, Unity is like a Hollywood set where everything looks stunning and realistic. Many robotics teams use both: Gazebo for physics-accurate testing and Unity for perception training and demonstrations.

**Prerequisites**: Completed [Chapter 2: Physics Simulation](/docs/module-2-simulation/physics-sim), basic familiarity with 3D graphics concepts

**What You Will Learn**: Set up Unity with ROS 2 and create realistic simulation environments

---

## Learning Objectives

After completing this chapter, you will be able to:

- Explain when to use Unity versus Gazebo for robot simulation
- Install Unity and configure the ROS-TCP-Connector package
- Create a basic robot simulation scene in Unity
- Bridge Unity and ROS 2 for bidirectional communication
- Import and use 3D assets from the Unity Asset Store
- Understand the tradeoffs between visual fidelity and physics accuracy

---

## When to Use Unity vs Gazebo

Before diving into setup, let us understand when each tool is the right choice. This will save you time by using the right tool for your specific needs.

### Comparison Table

| Feature | Gazebo | Unity |
|---------|--------|-------|
| **Physics accuracy** | Excellent | Good (requires tuning) |
| **Visual quality** | Basic | Photo-realistic |
| **ROS 2 integration** | Native | Via ROS-TCP-Connector |
| **Learning curve** | Moderate | Steep |
| **Asset availability** | Limited | Thousands available |
| **Custom environments** | Code-heavy | Visual editor |
| **Real-time performance** | Good | Excellent |
| **Sensor simulation** | Built-in | Requires plugins |

### Use Gazebo When...

- Physics accuracy is critical (manipulation, contact dynamics)
- You need built-in sensor plugins (lidar, cameras, IMU)
- You want tight ROS 2 integration out of the box
- Your team already knows Gazebo

### Use Unity When...

- Visual realism matters (perception training, demos, ML)
- You need complex environments (buildings, cities, nature)
- You want to leverage existing 3D assets
- You are training computer vision models (domain randomization)
- You need high frame rates for real-time applications

### Hybrid Approach: Best of Both Worlds

Many professional teams use **both** simulators:

```
┌─────────────────────────────────────────────────────────────────┐
│                    HYBRID SIMULATION WORKFLOW                    │
│                                                                  │
│   Development & Physics Testing:                                 │
│   ┌─────────────────────────────────────────────────────────┐   │
│   │                      GAZEBO                               │   │
│   │  • Control algorithm development                          │   │
│   │  • Physics-based testing                                  │   │
│   │  • Sensor noise modeling                                  │   │
│   └─────────────────────────────────────────────────────────┘   │
│                              │                                   │
│                              │ Export robot model                │
│                              ▼                                   │
│   Perception & ML Training:                                      │
│   ┌─────────────────────────────────────────────────────────┐   │
│   │                       UNITY                                │   │
│   │  • Photo-realistic camera images                          │   │
│   │  • Domain randomization for ML                            │   │
│   │  • Complex environment scenarios                          │   │
│   └─────────────────────────────────────────────────────────┘   │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## Installing Unity and ROS 2 Integration

Let us set up Unity to communicate with ROS 2.

### Step 1: Install Unity Hub and Unity Editor

1. Download Unity Hub from [unity.com/download](https://unity.com/download)
2. Install Unity Hub
3. Open Unity Hub and sign in (free account)
4. Go to **Installs** → **Install Editor**
5. Select **Unity 2022.3 LTS** (Long Term Support)
6. Include these modules:
   - Linux Build Support (for deployment)
   - Documentation

```bash
# After installation, verify Unity is accessible
# Unity Hub installs Unity to a standard location
ls ~/Unity/Hub/Editor/
# Expected: 2022.3.x folder
```

### Step 2: Create a New Unity Project

1. Open Unity Hub
2. Click **New Project**
3. Select **3D (URP)** template (Universal Render Pipeline for better graphics)
4. Name your project: `RobotSimulation`
5. Click **Create project**

:::tip Why URP?
The Universal Render Pipeline (URP) provides better visual quality than the built-in renderer while maintaining good performance. It is ideal for robotics simulation where you want realistic visuals without the complexity of HDRP.
:::

### Step 3: Install ROS-TCP-Connector Package

The ROS-TCP-Connector enables communication between Unity and ROS 2.

1. In Unity, go to **Window** → **Package Manager**
2. Click the **+** button → **Add package from git URL**
3. Enter: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`
4. Click **Add**

Alternatively, edit `Packages/manifest.json` and add:

```json
{
  "dependencies": {
    "com.unity.robotics.ros-tcp-connector": "https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector"
  }
}
```

### Step 4: Install ROS-TCP-Endpoint on Your ROS 2 System

The endpoint runs on your ROS 2 machine and bridges to Unity:

```bash
# Create a workspace if you don't have one
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone the ROS-TCP-Endpoint package
git clone -b main-ros2 https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git

# Build the workspace
cd ~/ros2_ws
colcon build --packages-select ros_tcp_endpoint
source install/setup.bash
```

### Step 5: Verify Communication

Start the ROS-TCP-Endpoint:

```bash
# Terminal 1: Start the endpoint
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
# Expected: "Starting server on 0.0.0.0:10000"
```

In Unity:
1. Go to **Robotics** → **ROS Settings**
2. Set **ROS IP Address** to `127.0.0.1` (if running locally) or your ROS machine's IP
3. Set **ROS Port** to `10000`
4. Click **Connect**

You should see "Connected" in the Unity console.

---

## Understanding the Communication Architecture

Unity and ROS 2 communicate over TCP using a custom protocol. Understanding this architecture helps you debug connection issues.

```
┌─────────────────────────────────────────────────────────────────┐
│                  UNITY-ROS 2 COMMUNICATION                       │
│                                                                  │
│  ┌───────────────────┐          ┌───────────────────┐           │
│  │      UNITY        │          │       ROS 2       │           │
│  │                   │          │                   │           │
│  │  ┌─────────────┐  │   TCP    │  ┌─────────────┐  │           │
│  │  │   Unity     │  │◄────────▶│  │   Endpoint  │  │           │
│  │  │  Simulation │  │  :10000  │  │   Node      │  │           │
│  │  └─────────────┘  │          │  └──────┬──────┘  │           │
│  │         │         │          │         │         │           │
│  │         │         │          │         │         │           │
│  │  ┌──────▼──────┐  │          │  ┌──────▼──────┐  │           │
│  │  │  Publisher/ │  │          │  │  ROS Topics │  │           │
│  │  │  Subscriber │  │          │  │  & Services │  │           │
│  │  │   Scripts   │  │          │  │             │  │           │
│  │  └─────────────┘  │          │  └─────────────┘  │           │
│  │                   │          │                   │           │
│  └───────────────────┘          └───────────────────┘           │
│                                                                  │
│   Unity scripts publish/subscribe to ROS topics through the      │
│   TCP endpoint. No ROS installation needed on Unity machine.     │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

**Key Points:**
- Unity does NOT need ROS installed—only the ROS-TCP-Connector package
- The endpoint translates between Unity's C# and ROS 2 messages
- Communication is bidirectional: Unity can both publish and subscribe
- You can run Unity on Windows/Mac while ROS 2 runs on Linux

---

## Creating Your First Unity Robot Scene

Let us build a simple scene with a robot that communicates with ROS 2.

### Step 1: Set Up the Environment

1. In the Unity Hierarchy, right-click → **3D Object** → **Plane**
2. Scale the plane to (10, 1, 10) for a 10m x 10m floor
3. Rename it to `Ground`
4. Create a material for the ground:
   - Right-click in Project → **Create** → **Material**
   - Name it `GroundMaterial`
   - Set color to gray
   - Drag onto the Ground object

### Step 2: Add a Simple Robot Model

For simplicity, we will create a basic robot from primitives:

1. Create an empty GameObject: **GameObject** → **Create Empty**
2. Rename it to `Robot`
3. Add child objects:
   - **Cube** for the body (scale: 0.5, 0.3, 0.7)
   - **Cylinders** for wheels (4 wheels, scale appropriately)
4. Position the robot at (0, 0.2, 0)

For a real project, you would import a URDF or 3D model instead.

### Step 3: Create a ROS Publisher Script

Create a script that publishes the robot's position to ROS 2:

```csharp
// File: RobotPositionPublisher.cs
// Place in: Assets/Scripts/

using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;  // geometry_msgs

public class RobotPositionPublisher : MonoBehaviour
{
    // ROS connection
    private ROSConnection ros;

    // Topic name
    public string topicName = "/robot_position";

    // Publish rate
    public float publishRate = 10f;  // 10 Hz
    private float timeElapsed = 0f;

    void Start()
    {
        // Get the ROS connection instance
        ros = ROSConnection.GetOrCreateInstance();

        // Register the publisher
        ros.RegisterPublisher<PointMsg>(topicName);

        Debug.Log($"Publishing robot position to {topicName}");
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;

        // Publish at the specified rate
        if (timeElapsed >= 1f / publishRate)
        {
            PublishPosition();
            timeElapsed = 0f;
        }
    }

    void PublishPosition()
    {
        // Get the robot's position in Unity coordinates
        Vector3 unityPos = transform.position;

        // Convert to ROS coordinates (Unity: Y-up, ROS: Z-up)
        // Unity (x, y, z) -> ROS (z, -x, y)
        PointMsg msg = new PointMsg
        {
            x = unityPos.z,
            y = -unityPos.x,
            z = unityPos.y
        };

        ros.Publish(topicName, msg);
    }
}
```

**Important: Coordinate System Conversion**

Unity and ROS use different coordinate systems. This is a common source of bugs:

```
┌─────────────────────────────────────────────────────────────────┐
│                  COORDINATE SYSTEM DIFFERENCES                   │
│                                                                  │
│   UNITY (Left-handed, Y-up):       ROS (Right-handed, Z-up):    │
│                                                                  │
│         Y (up)                           Z (up)                  │
│         │                                │                       │
│         │                                │                       │
│         │                                │                       │
│         └───────── X (right)             └───────── X (forward)  │
│        /                                /                        │
│       /                                /                         │
│      Z (forward)                      Y (left)                   │
│                                                                  │
│   Conversion: ROS_x = Unity_z                                    │
│               ROS_y = -Unity_x                                   │
│               ROS_z = Unity_y                                    │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### Step 4: Create a ROS Subscriber Script

Create a script that receives velocity commands from ROS 2:

```csharp
// File: RobotVelocitySubscriber.cs
// Place in: Assets/Scripts/

using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;  // geometry_msgs

public class RobotVelocitySubscriber : MonoBehaviour
{
    // ROS connection
    private ROSConnection ros;

    // Topic name
    public string topicName = "/cmd_vel";

    // Movement parameters
    public float linearSpeed = 2f;   // meters per second
    public float angularSpeed = 90f;  // degrees per second

    // Current velocity command
    private Vector3 linearVelocity = Vector3.zero;
    private float angularVelocity = 0f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        // Subscribe to the cmd_vel topic
        ros.Subscribe<TwistMsg>(topicName, VelocityCallback);

        Debug.Log($"Subscribed to velocity commands on {topicName}");
    }

    void VelocityCallback(TwistMsg msg)
    {
        // Convert ROS velocity to Unity coordinates
        // ROS linear: (x=forward, y=left, z=up)
        // Unity: (x=right, y=up, z=forward)
        linearVelocity = new Vector3(
            -(float)msg.linear.y * linearSpeed,  // ROS left -> Unity right (inverted)
            0f,
            (float)msg.linear.x * linearSpeed    // ROS forward -> Unity forward
        );

        // Angular velocity around Z in ROS = around Y in Unity
        angularVelocity = (float)msg.angular.z * angularSpeed;
    }

    void Update()
    {
        // Apply movement
        transform.Translate(linearVelocity * Time.deltaTime, Space.Self);
        transform.Rotate(0f, angularVelocity * Time.deltaTime, 0f);
    }
}
```

### Step 5: Attach Scripts to Robot

1. Select the `Robot` GameObject
2. Click **Add Component** → search for `RobotPositionPublisher`
3. Click **Add Component** → search for `RobotVelocitySubscriber`
4. Configure the topic names if needed

### Step 6: Test the Integration

1. Start the ROS-TCP-Endpoint:
   ```bash
   ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
   ```

2. Press Play in Unity

3. In ROS 2, check the position topic:
   ```bash
   ros2 topic echo /robot_position
   # Should show x, y, z coordinates updating
   ```

4. Send velocity commands:
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}, angular: {z: 0.3}}"
   # Robot should move forward while turning
   ```

---

## Importing 3D Assets for Realistic Environments

One of Unity's biggest advantages is access to thousands of pre-built 3D models.

### Using the Unity Asset Store

1. Open **Window** → **Asset Store** (or visit [assetstore.unity.com](https://assetstore.unity.com))
2. Search for relevant assets:
   - "warehouse environment"
   - "hospital interior"
   - "office building"
   - "industrial factory"
3. Filter by price (many free assets available)
4. Click **Add to My Assets** then **Import in Unity**

### Recommended Free Assets for Robotics

| Asset | Use Case | Notes |
|-------|----------|-------|
| Polygon Starter Pack | General environments | Low-poly, good performance |
| Warehouse Pack | Logistics robots | Shelves, pallets, forklifts |
| Simple Office | Indoor navigation | Desks, walls, doors |
| Nature Starter Kit | Outdoor robots | Trees, terrain, rocks |

### Optimizing Assets for Simulation

Large assets can slow down your simulation. Here are optimization tips:

1. **Reduce polygon count**: Use LOD (Level of Detail) groups
2. **Bake lighting**: Pre-compute lighting for static objects
3. **Disable unnecessary colliders**: Only keep colliders on objects the robot interacts with
4. **Use texture atlases**: Combine multiple textures into one

---

## Adding Basic Sensors in Unity

While Gazebo has built-in sensor plugins, Unity requires custom scripts for sensors.

### Simple Camera Sensor

```csharp
// File: CameraSensor.cs
// Captures images and publishes to ROS 2

using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class CameraSensor : MonoBehaviour
{
    public string topicName = "/camera/image_raw";
    public int width = 640;
    public int height = 480;
    public float publishRate = 30f;

    private Camera sensorCamera;
    private RenderTexture renderTexture;
    private Texture2D texture2D;
    private ROSConnection ros;
    private float timeElapsed = 0f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(topicName);

        // Setup render texture
        sensorCamera = GetComponent<Camera>();
        renderTexture = new RenderTexture(width, height, 24);
        sensorCamera.targetTexture = renderTexture;
        texture2D = new Texture2D(width, height, TextureFormat.RGB24, false);
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;
        if (timeElapsed >= 1f / publishRate)
        {
            CaptureAndPublish();
            timeElapsed = 0f;
        }
    }

    void CaptureAndPublish()
    {
        // Capture from render texture
        RenderTexture.active = renderTexture;
        texture2D.ReadPixels(new Rect(0, 0, width, height), 0, 0);
        texture2D.Apply();
        RenderTexture.active = null;

        // Convert to ROS message
        byte[] imageData = texture2D.GetRawTextureData();
        ImageMsg msg = new ImageMsg
        {
            height = (uint)height,
            width = (uint)width,
            encoding = "rgb8",
            step = (uint)(width * 3),
            data = imageData
        };

        ros.Publish(topicName, msg);
    }
}
```

### Basic Lidar Simulation (Raycast-based)

```csharp
// File: LidarSensor.cs
// Simple 2D lidar using raycasts

using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using System.Collections.Generic;

public class LidarSensor : MonoBehaviour
{
    public string topicName = "/scan";
    public float maxRange = 10f;
    public int numRays = 360;
    public float publishRate = 10f;

    private ROSConnection ros;
    private float timeElapsed = 0f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<LaserScanMsg>(topicName);
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;
        if (timeElapsed >= 1f / publishRate)
        {
            ScanAndPublish();
            timeElapsed = 0f;
        }
    }

    void ScanAndPublish()
    {
        float[] ranges = new float[numRays];
        float angleIncrement = 360f / numRays;

        for (int i = 0; i < numRays; i++)
        {
            float angle = i * angleIncrement;
            Vector3 direction = Quaternion.Euler(0, angle, 0) * transform.forward;

            RaycastHit hit;
            if (Physics.Raycast(transform.position, direction, out hit, maxRange))
            {
                ranges[i] = hit.distance;
            }
            else
            {
                ranges[i] = maxRange;  // No hit = max range
            }
        }

        // Create ROS message
        LaserScanMsg msg = new LaserScanMsg
        {
            angle_min = 0f,
            angle_max = Mathf.PI * 2,
            angle_increment = Mathf.Deg2Rad * angleIncrement,
            range_min = 0.1f,
            range_max = maxRange,
            ranges = ranges
        };

        ros.Publish(topicName, msg);
    }
}
```

---

## Hands-On Exercise

### Exercise 1: Build a Warehouse Navigation Scene

**Objective:** Create a Unity scene with a warehouse environment and a robot that responds to ROS 2 commands.

**Difficulty:** Intermediate

**Instructions:**

1. Create a new Unity scene
2. Build a simple warehouse layout using cubes (shelves, walls)
3. Add a robot with position publisher and velocity subscriber
4. Connect to ROS 2 and control the robot with teleop
5. Bonus: Add a camera sensor to the robot

**Verification:**
```bash
# Terminal 1: Start endpoint
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0

# Terminal 2: Run teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel

# Terminal 3: Watch position
ros2 topic echo /robot_position
```

---

## Common Unity-ROS Integration Issues

### Issue 1: Connection Failed

**Cause:** Endpoint not running or wrong IP address.

**Solution:**
```bash
# Check endpoint is running
ros2 node list  # Should show /TCPEndpoint

# Check IP address
hostname -I  # Use this IP in Unity settings
```

### Issue 2: Messages Not Received

**Cause:** Topic name mismatch or message type mismatch.

**Solution:** Check exact topic names and message types on both sides.

### Issue 3: Robot Moves Wrong Direction

**Cause:** Coordinate system conversion error.

**Solution:** Review the coordinate conversion formulas. Unity is Y-up, ROS is Z-up.

---

## Summary

### Key Takeaways

1. **Unity excels at visual realism** while Gazebo excels at physics. Use the right tool for your needs, or use both in a hybrid workflow.

2. **ROS-TCP-Connector bridges Unity and ROS 2** over TCP. Unity does not need ROS installed—only the connector package.

3. **Coordinate systems differ** between Unity (Y-up, left-handed) and ROS (Z-up, right-handed). Always convert coordinates in your scripts.

4. **Unity's Asset Store provides environments** that would take weeks to build from scratch. Leverage existing assets for complex scenes.

5. **Sensors require custom scripts** in Unity, unlike Gazebo's built-in plugins. Plan for additional development time.

### Quick Reference

```csharp
// Basic publisher
ros.RegisterPublisher<MsgType>("topic_name");
ros.Publish("topic_name", message);

// Basic subscriber
ros.Subscribe<MsgType>("topic_name", CallbackFunction);

// Coordinate conversion (Unity to ROS)
rosX = unityZ;
rosY = -unityX;
rosZ = unityY;
```

---

## What's Next

Now that you can create visually realistic environments in Unity, the final chapter of this module covers **sensor simulation**. You will learn to simulate cameras, lidar, IMU, and other sensors with realistic noise models—essential for testing perception algorithms.

[Next: Chapter 4 - Sensor Simulation →](/docs/module-2-simulation/sensors)

---

## Additional Resources

- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub) - Official Unity robotics resources
- [ROS-TCP-Connector Documentation](https://github.com/Unity-Technologies/ROS-TCP-Connector) - Package documentation
- [Unity Learn](https://learn.unity.com/) - Free Unity tutorials
- [Unity Asset Store](https://assetstore.unity.com/) - 3D models and environments
