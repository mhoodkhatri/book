# Exercise Template

This template defines the structure for hands-on exercises within chapters.

---

## Exercise Structure

```markdown
## Hands-on Exercise: [Descriptive Title]

**Objective**: [One clear sentence stating what reader will accomplish]

**Difficulty**: [Beginner | Intermediate | Advanced]

**Estimated Time**: [15-45 minutes]

### Prerequisites

Ensure you have:

- [ ] [Software requirement 1]
- [ ] [Software requirement 2]
- [ ] [Completed prior section/chapter]

### Step 1: [Action Title]

[1-2 sentences explaining what this step accomplishes]

```bash
# Command to execute
[command here]
```

[Optional: Explain what the command does]

### Step 2: [Action Title]

[Explanation]

```python
# Filename: exercise_file.py
# Code with comments
[code here]
```

### Step 3: [Action Title]

[Explanation and instructions]

---

**Checkpoint**: Verify your progress

After completing Step 3, run:

```bash
[verification command]
```

**Expected Result**:
```
[what successful output looks like]
```

**Troubleshooting**: If you see `[error message]`, check that [common fix].

---

### Step 4: [Continue as needed]

[More steps...]

---

**Final Checkpoint**: Exercise Complete

Verify the full exercise by:

```bash
[final verification]
```

**Success Criteria**:
- [ ] [Criterion 1]
- [ ] [Criterion 2]
- [ ] [Criterion 3]

---

### Stretch Goals (Optional)

Ready for more? Try these challenges:

1. **[Challenge 1 Title]**: [Description of advanced modification]
2. **[Challenge 2 Title]**: [Description of extension]

### What You Learned

In this exercise, you practiced:

- [Skill 1]
- [Skill 2]
- [Skill 3]
```

---

## Difficulty Levels

| Level | Target Reader | Characteristics |
|-------|---------------|-----------------|
| Beginner | New to ROS 2 | Step-by-step, no assumptions, full code provided |
| Intermediate | Completed Module 1 | Some gaps to fill, combines concepts |
| Advanced | Completed Module 1-2+ | Problem-solving required, minimal scaffolding |

---

## Checkpoint Guidelines

Every exercise MUST have at least one checkpoint.

**Good Checkpoint**:
```markdown
**Checkpoint**: After Step 3, verify the node is running:

```bash
ros2 node list
```

**Expected**:
```
/minimal_publisher
```

If you don't see the node, ensure:
- The terminal has ROS 2 sourced (`source /opt/ros/humble/setup.bash`)
- The script has no syntax errors
```

**Bad Checkpoint**:
```markdown
Checkpoint: Make sure it works.
```

---

## Step Writing Guidelines

### DO:
- Start each step with an action verb
- Include exact commands with copy-paste accuracy
- Explain the "why" not just the "what"
- Provide expected output for commands
- Include troubleshooting for common errors

### DON'T:
- Combine multiple unrelated actions in one step
- Assume reader knows unstated prerequisites
- Use vague instructions ("configure appropriately")
- Skip explaining non-obvious commands
- Leave readers guessing about success/failure

---

## Exercise Complexity Targets

| Chapter Position | Exercise Count | Total Steps | Time |
|-----------------|---------------|-------------|------|
| Early in module | 1 | 5-7 | 15-25 min |
| Mid-module | 1-2 | 7-10 | 25-35 min |
| End of module | 1 | 10-15 | 35-45 min |

---

## Example: Beginner Exercise

```markdown
## Hands-on Exercise: Create Your First ROS 2 Node

**Objective**: Create and run a minimal ROS 2 node that prints a message.

**Difficulty**: Beginner

**Estimated Time**: 20 minutes

### Prerequisites

Ensure you have:

- [ ] ROS 2 Humble installed
- [ ] A terminal with ROS 2 environment sourced
- [ ] Python 3.10+ available

### Step 1: Create a Workspace

Create a new ROS 2 workspace directory:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

This creates the standard workspace structure with a `src` directory for packages.

### Step 2: Create a Python Package

Generate a new Python package:

```bash
ros2 pkg create --build-type ament_python my_first_node
```

This creates a package named `my_first_node` with Python build configuration.

---

**Checkpoint**: Verify package creation

```bash
ls ~/ros2_ws/src/my_first_node/
```

**Expected**:
```
my_first_node  package.xml  resource  setup.cfg  setup.py  test
```

---

### Step 3: Create the Node Script

Create a new file `~/ros2_ws/src/my_first_node/my_first_node/hello_node.py`:

```python
#!/usr/bin/env python3
"""A minimal ROS 2 node that logs a greeting."""

import rclpy
from rclpy.node import Node

class HelloNode(Node):
    def __init__(self):
        super().__init__('hello_node')
        # Log a message when node starts
        self.get_logger().info('Hello from ROS 2!')

def main(args=None):
    rclpy.init(args=args)
    node = HelloNode()
    # Spin once then shutdown (no continuous operation)
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 4: Register the Entry Point

Edit `setup.py` to add the console script entry point:

```python
entry_points={
    'console_scripts': [
        'hello_node = my_first_node.hello_node:main',
    ],
},
```

### Step 5: Build and Run

Build the workspace and run the node:

```bash
cd ~/ros2_ws
colcon build --packages-select my_first_node
source install/setup.bash
ros2 run my_first_node hello_node
```

---

**Final Checkpoint**: Exercise Complete

**Expected Output**:
```
[INFO] [timestamp] [hello_node]: Hello from ROS 2!
```

**Success Criteria**:
- [ ] Package builds without errors
- [ ] Node runs and displays the greeting
- [ ] Node exits cleanly after printing

---

### Stretch Goals (Optional)

1. **Add a Parameter**: Modify the node to accept a custom greeting via ROS 2 parameters
2. **Continuous Logging**: Change the node to log a message every second using a timer

### What You Learned

In this exercise, you practiced:

- Creating a ROS 2 workspace and Python package
- Writing a minimal ROS 2 node class
- Registering and running a node via entry points
```
