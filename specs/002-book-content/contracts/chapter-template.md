# Chapter Template

This template defines the required structure for all chapter content. Every chapter MUST follow this structure exactly.

---

## File Structure

```markdown
---
sidebar_position: [NUMBER]
title: "Chapter [N]: [Title]"
description: [150-160 character description for SEO]
keywords: [keyword1, keyword2, keyword3, keyword4, keyword5]
---

# Chapter [N]: [Title]

## Chapter Overview

[1-2 paragraphs: What this chapter covers, why it matters, where it fits in the book]

**Prerequisites**: [List required prior knowledge and chapters]

**What You'll Build**: [Concrete outcome - what reader will create/understand]

## Learning Objectives

After completing this chapter, you will be able to:

- [Verb] [skill/knowledge] [context]
- [Verb] [skill/knowledge] [context]
- [Verb] [skill/knowledge] [context]
- [Verb] [skill/knowledge] [context]

## [Main Concept Section Title]

[500-1000 tokens of conceptual explanation]

### [Subsection if needed]

[200-500 tokens]

## [Second Concept Section]

[500-1000 tokens]

## Practical Example: [Example Title]

[Introduction to what we're building]

### Step 1: [Step Title]

[Explanation]

```python
# Filename: example_file.py
# Code with at least 2 inline comments
```

### Step 2: [Step Title]

[Explanation and code]

### Running the Example

```bash
# Commands to run
```

**Expected Output**:
```
[Expected terminal output]
```

## Hands-on Exercise: [Exercise Title]

**Objective**: [Clear goal statement]

**Prerequisites**:
- [Required setup item 1]
- [Required setup item 2]

**Steps**:

1. [Atomic instruction]
   ```bash
   # Command if applicable
   ```

2. [Atomic instruction]

3. [Atomic instruction]

**Checkpoint**: After step [N], verify:
```bash
# Verification command
# Expected: [what success looks like]
```

**Stretch Goal** (Optional): [Advanced challenge]

## Summary

In this chapter, you learned:

- [Key takeaway 1]
- [Key takeaway 2]
- [Key takeaway 3]
- [Key takeaway 4]
- [Key takeaway 5]

## Next Steps

Continue to [Chapter N+1: Title](/docs/module-X/next-chapter) to learn about [topic].
```

---

## Section Token Targets

| Section | Target Tokens | Target Words |
|---------|---------------|--------------|
| Chapter Overview | 150-250 | 110-190 |
| Learning Objectives | 80-150 | 60-110 |
| Main Concept (H2) | 500-1000 | 375-750 |
| Subsection (H3) | 200-500 | 150-375 |
| Practical Example | 400-800 | 300-600 |
| Exercise | 300-500 | 225-375 |
| Summary | 100-200 | 75-150 |
| **Total Chapter** | **2000-4000** | **1500-3000** |

---

## Code Block Requirements

Every code block MUST:

1. Specify language: `python`, `bash`, `yaml`, `cpp`, `cmake`, `xml`, `json`
2. Include filename comment if applicable: `# Filename: my_node.py`
3. Have at least 2 inline comments explaining key logic
4. Be syntactically valid and copy-paste runnable
5. Show expected output for commands

**Good Example**:
```python
#!/usr/bin/env python3
"""Minimal publisher demonstrating ROS 2 topic publication."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        # Create publisher with queue depth of 10
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        # Fire callback every 500ms
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello: {self.count}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        self.count += 1

def main():
    rclpy.init()
    node = MinimalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Bad Example** (DO NOT use):
```python
# Missing imports, no comments, incomplete
node = Node('test')
node.publish(msg)
```

---

## Learning Objective Verbs

Use Bloom's Taxonomy action verbs:

| Level | Verbs |
|-------|-------|
| Remember | define, list, recall, identify, name |
| Understand | explain, describe, summarize, interpret, classify |
| Apply | implement, execute, use, demonstrate, solve |
| Analyze | differentiate, compare, debug, troubleshoot, examine |
| Evaluate | assess, evaluate, justify, critique, judge |
| Create | design, construct, build, develop, compose |

**Examples**:
- "Explain the difference between topics and services"
- "Implement a publisher-subscriber system"
- "Debug common ROS 2 communication issues"

---

## Forbidden Content

NEVER include:

- Placeholder text: "coming soon", "TBD", "content here"
- Marketing language: "amazing", "revolutionary", "cutting-edge"
- Emojis (unless explicitly requested)
- Casual language: "gonna", "cool", "awesome"
- First person singular: "I think", "I believe"
- Unexplained acronyms on first use
- Code without comments
- Incomplete code snippets without `[Partial]` label
