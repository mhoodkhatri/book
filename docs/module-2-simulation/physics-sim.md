---
sidebar_position: 3
title: "Chapter 2: Physics Simulation"
description: "Understand physics engines in robot simulation. Configure dynamics, friction, contacts, and inertia for realistic robot behavior in Gazebo."
keywords: [physics engine, dynamics, friction, collision, DART, simulation, contacts, inertia, Gazebo]
---

# Chapter 2: Physics Simulation

## Chapter Overview

In Chapter 1, you learned to launch Gazebo and spawn robots. But what makes objects fall, collide, and bounce? What makes your robot wheels grip the ground instead of sliding? The answer is the **physics engine**.

A physics engine is the software that computes all the forces, collisions, and movements in your simulation. It is the invisible hand that makes everything behave like it would in the real world. Understanding how the physics engine works is crucial because **wrong physics settings lead to unrealistic behavior**—and code that works in a bad simulation often fails on real hardware.

Think of the physics engine as the **laws of nature** inside your virtual world. Just like real physics governs how fast things fall and how hard they bounce, the physics engine governs the same things in simulation. The difference is that you can **configure** these laws—change gravity, adjust friction, or make objects perfectly rigid.

**Prerequisites**: Completed [Chapter 1: Gazebo Basics](/docs/module-2-simulation/gazebo-basics)

**What You Will Learn**: Configure physics for realistic and efficient simulation

---

## Learning Objectives

After completing this chapter, you will be able to:

- Explain what a physics engine does and why it matters for robotics
- Choose between different physics engines (DART, ODE, Bullet)
- Configure physics step size and real-time factor
- Set friction and contact parameters for realistic ground interaction
- Understand inertia and mass distribution for stable dynamics
- Tune physics parameters to balance realism and performance

---

## What Does a Physics Engine Do?

Every simulation frame, the physics engine must answer thousands of questions:

- Is the robot touching the ground? Where exactly?
- How much friction force acts on each wheel?
- What happens when the arm hits the table?
- How does gravity pull on every link?

### The Physics Loop

The physics engine runs in a continuous loop:

```
┌─────────────────────────────────────────────────────────────────┐
│                    THE PHYSICS SIMULATION LOOP                   │
│                                                                  │
│    ┌─────────────┐                                              │
│    │   START     │                                              │
│    └──────┬──────┘                                              │
│           │                                                      │
│           ▼                                                      │
│    ┌─────────────────────────────────────────────┐              │
│    │  1. COLLISION DETECTION                      │              │
│    │     Which objects are touching?              │              │
│    │     Where exactly do they touch?             │              │
│    └──────────────────┬──────────────────────────┘              │
│                       │                                          │
│                       ▼                                          │
│    ┌─────────────────────────────────────────────┐              │
│    │  2. FORCE COMPUTATION                        │              │
│    │     Gravity, friction, contact forces        │              │
│    │     Motor torques, spring forces             │              │
│    └──────────────────┬──────────────────────────┘              │
│                       │                                          │
│                       ▼                                          │
│    ┌─────────────────────────────────────────────┐              │
│    │  3. INTEGRATION                              │              │
│    │     Update velocities from forces            │              │
│    │     Update positions from velocities         │              │
│    └──────────────────┬──────────────────────────┘              │
│                       │                                          │
│                       ▼                                          │
│    ┌─────────────────────────────────────────────┐              │
│    │  4. CONSTRAINT SOLVING                       │              │
│    │     Joints stay connected                    │              │
│    │     Objects don't pass through each other    │              │
│    └──────────────────┬──────────────────────────┘              │
│                       │                                          │
│                       └──────────────► REPEAT                    │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

Each cycle of this loop is called a **physics step**. The physics engine runs many steps per second—typically 1000 steps per second for robotics simulation.

---

## Physics Engines in Gazebo

Gazebo supports multiple physics engines. Each has different strengths:

| Engine | Full Name | Strengths | Best For |
|--------|-----------|-----------|----------|
| **DART** | Dynamic Animation and Robotics Toolkit | Accurate joints, stable | Humanoids, arms |
| **ODE** | Open Dynamics Engine | Fast, well-tested | General robotics |
| **Bullet** | Bullet Physics | Good soft bodies | Deformable objects |
| **TPE** | Trivial Physics Engine | Very fast, simple | Simple tests |

### Which Engine Should You Use?

For most robotics work, **DART** is the recommended choice. It handles articulated robots (robots with joints) very well and provides stable, accurate simulation. This textbook uses DART throughout.

### Selecting the Physics Engine

In your SDF world file, specify the engine in the `<physics>` tag:

```xml
<physics name="1ms" type="dart">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
</physics>
```

Change `type="dart"` to `type="ode"` or `type="bullet"` to use a different engine.

---

## The Physics Step Size

The **step size** is how much simulated time passes in each physics step. This is one of the most important parameters you will configure.

### Understanding Step Size

Imagine you are filming a robot with a camera:

- A **small step size** (0.001 seconds) is like filming at 1000 frames per second—you capture every tiny movement, but it takes more computation.
- A **large step size** (0.01 seconds) is like filming at 100 frames per second—you miss some details, but it runs faster.

```
┌─────────────────────────────────────────────────────────────────┐
│                 STEP SIZE COMPARISON                             │
│                                                                  │
│   Small step (0.001s):                                          │
│   ●──●──●──●──●──●──●──●──●──●   (many steps, smooth motion)    │
│                                                                  │
│   Large step (0.01s):                                           │
│   ●────────●────────●────────●   (fewer steps, may miss details)│
│                                                                  │
│   Too large step:                                                │
│   ●──────────────────────────●   (objects pass through walls!)   │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### The Problem with Large Step Sizes

If your step size is too large, fast-moving objects can "tunnel" through walls. The physics engine checks for collisions at each step—if an object moves past a wall in a single step, the collision is missed.

This is why robot simulations typically use step sizes of 0.001 seconds (1 millisecond) or smaller.

### Configuring Step Size

```xml
<physics name="physics_config" type="dart">
  <!-- Step size in seconds (0.001 = 1ms = 1000 Hz) -->
  <max_step_size>0.001</max_step_size>

  <!-- How fast simulation runs relative to real time -->
  <!-- 1.0 = real-time, 2.0 = twice as fast, 0.5 = half speed -->
  <real_time_factor>1.0</real_time_factor>

  <!-- Maximum real-time updates per second for rendering -->
  <real_time_update_rate>1000</real_time_update_rate>
</physics>
```

### Choosing the Right Step Size

| Step Size | Physics Rate | Best For |
|-----------|-------------|----------|
| 0.001 s | 1000 Hz | Robot arms, humanoids, precise work |
| 0.002 s | 500 Hz | Mobile robots, general purpose |
| 0.005 s | 200 Hz | Simple robots, fast simulation |
| 0.01 s | 100 Hz | Rough testing only (may be unstable) |

:::tip Rule of Thumb
Start with 0.001 seconds. If simulation is too slow, try 0.002 seconds. Only go larger if you understand the tradeoffs.
:::

---

## Collision Detection and Contact Dynamics

When objects touch, the physics engine must compute **contact forces** that prevent them from passing through each other.

### How Collision Detection Works

The physics engine uses a two-phase approach:

**Phase 1: Broad Phase** - Quick check using bounding boxes to find pairs of objects that *might* be touching.

**Phase 2: Narrow Phase** - Detailed check on the candidate pairs to find exact contact points.

```
┌─────────────────────────────────────────────────────────────────┐
│                COLLISION DETECTION PHASES                        │
│                                                                  │
│   BROAD PHASE (fast, approximate):                              │
│   ┌─────────┐    ┌─────────┐                                    │
│   │ ┌───┐   │    │   ┌───┐ │   Bounding boxes overlap?          │
│   │ │ A │   │    │   │ B │ │   YES → Check narrow phase          │
│   │ └───┘   │    │   └───┘ │   NO  → Skip (not touching)         │
│   └─────────┘    └─────────┘                                    │
│                                                                  │
│   NARROW PHASE (slow, precise):                                 │
│       ┌───┐                                                      │
│       │ A │ ←── Contact point                                    │
│       └─┬─┘     Contact normal                                   │
│         │       Penetration depth                                │
│       ┌─┴─┐                                                      │
│       │ B │                                                      │
│       └───┘                                                      │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### Contact Parameters

When objects touch, several parameters control the resulting forces:

```xml
<collision name="collision">
  <geometry>
    <box><size>1 1 1</size></box>
  </geometry>
  <surface>
    <friction>
      <ode>
        <mu>0.9</mu>      <!-- Friction coefficient (0-1+) -->
        <mu2>0.9</mu2>    <!-- Secondary friction coefficient -->
      </ode>
    </friction>
    <contact>
      <ode>
        <kp>1000000</kp>  <!-- Contact stiffness (higher = harder surface) -->
        <kd>100</kd>      <!-- Contact damping (higher = less bounce) -->
      </ode>
    </contact>
  </surface>
</collision>
```

Let me explain each parameter in simple terms:

**Friction (`mu`)**: How grippy the surface is. 0 = ice (no friction), 1 = rubber (high friction), >1 = sticky. Real rubber on asphalt is about 0.8-1.0.

**Stiffness (`kp`)**: How hard the surface is. Higher values make objects bounce off instead of sinking in. Too low and objects may sink into each other.

**Damping (`kd`)**: How much energy is absorbed on contact. Higher values reduce bouncing. Too high and objects may stick together.

---

## Friction: Making Wheels Grip

Friction is critical for wheeled and legged robots. Without friction, wheels spin without moving the robot forward—like driving on ice.

### Types of Friction

Physics engines typically model two types of friction:

**Static Friction**: The force needed to *start* moving an object. A box sitting on a slope doesn't slide until the slope is steep enough to overcome static friction.

**Dynamic Friction**: The force acting on a *moving* object. Once the box starts sliding, dynamic friction (usually lower than static) resists its motion.

### Configuring Friction for Wheels

For a robot wheel to work properly, you need sufficient friction between the wheel and the ground.

```xml
<!-- In your wheel link's collision element -->
<surface>
  <friction>
    <ode>
      <mu>1.0</mu>     <!-- High friction so wheel grips -->
      <mu2>1.0</mu2>
      <fdir1>0 0 1</fdir1>  <!-- Friction direction (optional) -->
    </ode>
  </friction>
</surface>
```

```xml
<!-- Also configure the ground plane -->
<surface>
  <friction>
    <ode>
      <mu>0.8</mu>     <!-- Ground friction -->
      <mu2>0.8</mu2>
    </ode>
  </friction>
</surface>
```

### Common Friction Values

| Material Pair | Friction Coefficient |
|---------------|---------------------|
| Rubber on dry concrete | 0.6 - 0.9 |
| Rubber on wet concrete | 0.4 - 0.6 |
| Steel on steel | 0.4 - 0.6 |
| Teflon on anything | 0.04 |
| Ice on ice | 0.03 |

:::warning Friction Applies to Both Surfaces
Friction depends on BOTH surfaces touching. If your wheel has friction 1.0 but the ground has friction 0.0, the effective friction will be very low. Configure both!
:::

---

## Mass and Inertia: How Objects Move

Mass and inertia determine how objects respond to forces. Getting these right is essential for realistic robot dynamics.

### Mass: How Heavy

Mass is straightforward—how many kilograms the object weighs. Heavier objects need more force to accelerate.

```xml
<inertial>
  <mass>5.0</mass>  <!-- 5 kilograms -->
</inertial>
```

### Inertia: How Mass Is Distributed

Inertia is more subtle. It describes how mass is spread out, which affects how easily an object rotates.

Consider two objects with the same mass:

```
┌─────────────────────────────────────────────────────────────────┐
│                    INERTIA COMPARISON                            │
│                                                                  │
│   Solid sphere (low inertia):     Hollow sphere (high inertia): │
│                                                                  │
│        ●●●●                              ○○○○                    │
│       ●●●●●●                            ○    ○                   │
│       ●●●●●●                            ○    ○                   │
│        ●●●●                              ○○○○                    │
│                                                                  │
│   Mass near center →                Mass far from center →       │
│   Easy to spin                      Hard to spin                 │
│                                                                  │
│   Think: figure skater             Think: figure skater          │
│   arms pulled in = fast spin       arms out = slow spin          │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### The Inertia Matrix

Inertia is represented as a 3x3 matrix, but since it is symmetric, you only need to specify 6 values:

```xml
<inertial>
  <mass>5.0</mass>
  <inertia>
    <ixx>0.1</ixx>  <!-- Resistance to rotation around X axis -->
    <ixy>0.0</ixy>  <!-- Usually 0 for symmetric objects -->
    <ixz>0.0</ixz>
    <iyy>0.1</iyy>  <!-- Resistance to rotation around Y axis -->
    <iyz>0.0</iyz>
    <izz>0.05</izz> <!-- Resistance to rotation around Z axis -->
  </inertia>
</inertial>
```

### Calculating Inertia for Simple Shapes

For common shapes, inertia can be calculated from formulas:

**Solid Box (width w, depth d, height h, mass m):**
```
Ixx = (1/12) * m * (d² + h²)
Iyy = (1/12) * m * (w² + h²)
Izz = (1/12) * m * (w² + d²)
```

**Solid Cylinder (radius r, length l, mass m, rotating around length axis):**
```
Ixx = Iyy = (1/12) * m * (3r² + l²)
Izz = (1/2) * m * r²
```

**Solid Sphere (radius r, mass m):**
```
Ixx = Iyy = Izz = (2/5) * m * r²
```

### Inertia Example: A 1m Cube, 10kg

```python
# Python calculation for a 1m x 1m x 1m cube, mass 10kg
m = 10.0  # kg
w = d = h = 1.0  # meters

ixx = (1/12) * m * (d**2 + h**2)  # = 1.67
iyy = (1/12) * m * (w**2 + h**2)  # = 1.67
izz = (1/12) * m * (w**2 + d**2)  # = 1.67
```

```xml
<inertial>
  <mass>10.0</mass>
  <inertia>
    <ixx>1.67</ixx>
    <ixy>0</ixy>
    <ixz>0</ixz>
    <iyy>1.67</iyy>
    <iyz>0</iyz>
    <izz>1.67</izz>
  </inertia>
</inertial>
```

---

## Tuning Physics for Performance

Realistic physics comes at a computational cost. Here are strategies to improve performance while maintaining acceptable accuracy.

### Strategy 1: Simplify Collision Geometry

Complex mesh collisions are slow. Use simple shapes (boxes, cylinders, spheres) for collision, even if the visual geometry is detailed.

```xml
<!-- Visual: detailed mesh -->
<visual>
  <geometry>
    <mesh><uri>model://robot/meshes/arm_detailed.dae</uri></mesh>
  </geometry>
</visual>

<!-- Collision: simple cylinder (much faster) -->
<collision>
  <geometry>
    <cylinder><radius>0.05</radius><length>0.3</length></cylinder>
  </geometry>
</collision>
```

### Strategy 2: Increase Step Size (Carefully)

If your simulation is slow, try increasing the step size from 0.001 to 0.002 seconds. Test carefully—some scenarios may become unstable.

### Strategy 3: Reduce Real-Time Factor

If you do not need real-time simulation, let it run slower but more accurately:

```xml
<physics name="accurate" type="dart">
  <max_step_size>0.0005</max_step_size>  <!-- Very accurate -->
  <real_time_factor>0.5</real_time_factor>  <!-- Half speed is OK -->
</physics>
```

### Strategy 4: Disable Unnecessary Contacts

If objects will never touch, disable collision checking between them:

```xml
<collision name="collision">
  <geometry>...</geometry>
  <!-- Disable collision with specific other objects -->
</collision>
```

---

## Hands-On Exercise

### Exercise 1: Tune a Bouncing Ball

**Objective:** Configure physics parameters to make a ball bounce realistically.

**Difficulty:** Intermediate

**Instructions:**

1. Create a world with a ground plane and a sphere
2. Configure the sphere's mass and inertia
3. Adjust contact parameters to control bounce height
4. Experiment with different friction values

**Starter Code:**

```xml
<?xml version="1.0" ?>
<!-- File: bouncing_ball.sdf -->
<sdf version="1.8">
  <world name="bouncing_ball_world">

    <physics name="1ms" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>

    <light type="directional" name="sun">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
    </light>

    <!-- Ground with configured surface -->
    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane><normal>0 0 1</normal><size>10 10</size></plane>
          </geometry>
          <surface>
            <!-- TODO: Configure friction -->
            <!-- TODO: Configure contact stiffness and damping -->
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane><normal>0 0 1</normal><size>10 10</size></plane>
          </geometry>
        </visual>
      </link>
    </model>

    <!-- Bouncing ball -->
    <model name="ball">
      <pose>0 0 2 0 0 0</pose>  <!-- Start 2m above ground -->
      <link name="link">
        <collision name="collision">
          <geometry>
            <sphere><radius>0.1</radius></sphere>
          </geometry>
          <surface>
            <!-- TODO: Configure ball surface -->
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <sphere><radius>0.1</radius></sphere>
          </geometry>
          <material><ambient>1 0 0 1</ambient></material>
        </visual>
        <inertial>
          <!-- TODO: Configure mass and inertia for a 0.5kg ball -->
        </inertial>
      </link>
    </model>

  </world>
</sdf>
```

<details>
<summary>Click to reveal solution</summary>

```xml
<?xml version="1.0" ?>
<!-- File: bouncing_ball.sdf (Solution) -->
<sdf version="1.8">
  <world name="bouncing_ball_world">

    <physics name="1ms" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>

    <light type="directional" name="sun">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
    </light>

    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane><normal>0 0 1</normal><size>10 10</size></plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.8</mu>
                <mu2>0.8</mu2>
              </ode>
            </friction>
            <contact>
              <ode>
                <kp>1000000</kp>  <!-- Hard surface -->
                <kd>1</kd>        <!-- Low damping = more bounce -->
              </ode>
            </contact>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane><normal>0 0 1</normal><size>10 10</size></plane>
          </geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient></material>
        </visual>
      </link>
    </model>

    <model name="ball">
      <pose>0 0 2 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <sphere><radius>0.1</radius></sphere>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.5</mu>
                <mu2>0.5</mu2>
              </ode>
            </friction>
            <contact>
              <ode>
                <kp>1000000</kp>
                <kd>1</kd>  <!-- Low damping for bouncy ball -->
              </ode>
            </contact>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <sphere><radius>0.1</radius></sphere>
          </geometry>
          <material><ambient>1 0 0 1</ambient></material>
        </visual>
        <inertial>
          <mass>0.5</mass>
          <!-- Sphere inertia: I = (2/5) * m * r^2 -->
          <!-- I = (2/5) * 0.5 * 0.1^2 = 0.002 -->
          <inertia>
            <ixx>0.002</ixx>
            <iyy>0.002</iyy>
            <izz>0.002</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </inertia>
        </inertial>
      </link>
    </model>

  </world>
</sdf>
```

</details>

**Verification:**
```bash
# Launch and observe
gz sim bouncing_ball.sdf

# The ball should:
# 1. Fall from 2m
# 2. Bounce multiple times
# 3. Eventually settle

# Experiment: Change kd to 100 and observe less bouncing
```

---

## Common Physics Problems and Solutions

### Problem 1: Objects Jitter or Vibrate

**Cause:** Step size too large or contact parameters poorly tuned.

**Solution:** Reduce step size. Increase contact damping (`kd`).

### Problem 2: Objects Pass Through Each Other

**Cause:** Step size too large for object speeds, or missing collision geometry.

**Solution:** Reduce step size. Ensure all objects have `<collision>` elements.

### Problem 3: Robot Tips Over Unexpectedly

**Cause:** Incorrect mass/inertia values, center of mass in wrong place.

**Solution:** Verify inertial properties match real robot. Check `<origin>` in `<inertial>`.

### Problem 4: Wheels Spin Without Moving Robot

**Cause:** Friction too low between wheels and ground.

**Solution:** Increase friction on both wheel and ground surfaces.

---

## Summary

### Key Takeaways

1. **The physics engine simulates reality** by computing forces, collisions, and motion. It runs hundreds to thousands of times per second.

2. **Step size is critical** - Smaller steps are more accurate but slower. 0.001 seconds is typical for robotics.

3. **Friction makes wheels work** - Both surfaces (wheel and ground) need friction configured. Typical values are 0.6-1.0.

4. **Inertia affects rotation** - Mass distribution matters as much as total mass. Use formulas for simple shapes.

5. **Simplify collision geometry** - Use simple shapes for collision even if visuals are detailed. This dramatically improves performance.

### Quick Reference: Physics Parameters

```xml
<physics type="dart">
  <max_step_size>0.001</max_step_size>     <!-- 1ms step -->
  <real_time_factor>1.0</real_time_factor>  <!-- Real-time -->
</physics>

<surface>
  <friction>
    <ode>
      <mu>0.8</mu>        <!-- Friction coefficient -->
    </ode>
  </friction>
  <contact>
    <ode>
      <kp>1000000</kp>    <!-- Stiffness -->
      <kd>100</kd>        <!-- Damping -->
    </ode>
  </contact>
</surface>
```

---

## What's Next

Now that you understand physics simulation, the next chapter introduces **Unity integration**. Unity provides photo-realistic rendering and a vast ecosystem of assets, complementing Gazebo's physics strengths.

[Next: Chapter 3 - Unity Integration →](/docs/module-2-simulation/unity-integration)

---

## Additional Resources

- [DART Physics Engine](https://dartsim.github.io/) - Official DART documentation
- [SDF Surface Specification](http://sdformat.org/spec?elem=surface) - Complete surface parameters
- [Gazebo Physics Tutorial](https://gazebosim.org/docs/fortress/physics) - Official physics guide
