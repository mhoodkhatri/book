# Module Index Template

This template defines the required structure for module index pages (`index.md`).

---

## File Structure

```markdown
---
sidebar_position: 1
title: "Module [N]: [Metaphor Title]"
description: [150-160 character description for SEO]
keywords: [keyword1, keyword2, keyword3, keyword4, keyword5]
---

# Module [N]: [Metaphor Title]

**[Technical Subtitle]** | Weeks [X-Y]

## Module Overview

[2-3 paragraphs explaining:
- What this module covers
- Why it matters for humanoid robotics
- How it connects to other modules]

## Prerequisites

Before starting this module, you should:

- [ ] [Required knowledge item 1]
- [ ] [Required knowledge item 2]
- [ ] [Required software/hardware item]
- [ ] [Completed prior module if applicable]

## What You'll Learn

By the end of this module, you will be able to:

1. [Module-level outcome 1]
2. [Module-level outcome 2]
3. [Module-level outcome 3]
4. [Module-level outcome 4]

## Chapter Overview

### [Chapter 1: Title]

[2-3 sentences describing chapter content and what reader will learn]

**Key Topics**: [topic1], [topic2], [topic3]

### [Chapter 2: Title]

[2-3 sentences]

**Key Topics**: [topic1], [topic2], [topic3]

### [Chapter 3: Title]

[2-3 sentences]

**Key Topics**: [topic1], [topic2], [topic3]

### [Chapter 4: Title] (if applicable)

[2-3 sentences]

**Key Topics**: [topic1], [topic2], [topic3]

## Learning Path

```
[Chapter 1] ──▶ [Chapter 2] ──▶ [Chapter 3] ──▶ [Chapter 4]
   │              │               │               │
   ▼              ▼               ▼               ▼
[Skill A]    [Skill B]       [Skill C]       [Skill D]
```

**Suggested Pace**: [X hours per chapter, Y weeks total]

## Getting Started

Ready to begin? Start with [Chapter 1: Title](/docs/module-X/chapter-1).
```

---

## Module Metadata Reference

| Module | Metaphor Title | Technical Subtitle | Weeks |
|--------|---------------|-------------------|-------|
| 1 | The Robotic Nervous System | ROS 2 Fundamentals | 3-5 |
| 2 | The Digital Twin | Simulation with Gazebo & Unity | 6-7 |
| 3 | The AI-Robot Brain | NVIDIA Isaac Integration | 8-10 |
| 4 | Vision-Language-Action | VLA Models for Robotics | 11-13 |

---

## Section Guidelines

### Module Overview
- 200-400 words
- Connect to real-world robotics applications
- Explain the metaphor (e.g., "nervous system" for communication)
- Reference both preceding and following modules

### Prerequisites
- Be specific about versions (ROS 2 Humble, Ubuntu 22.04)
- Include both knowledge and software requirements
- Use checkboxes for reader self-assessment

### What You'll Learn
- 4-6 module-level outcomes
- Higher abstraction than chapter objectives
- Focus on capability ("you will be able to...")

### Chapter Overview
- Brief but informative
- Highlight key topics as inline tags
- Don't duplicate learning objectives

### Learning Path
- ASCII diagram showing progression
- Map chapters to skills acquired
- Include time estimates

---

## Example: Module 1 Index

```markdown
---
sidebar_position: 1
title: "Module 1: The Robotic Nervous System"
description: Master ROS 2 fundamentals including nodes, topics, services, and actions for building robust robot communication systems.
keywords: [ROS 2, robotics, nodes, topics, services, actions, middleware]
---

# Module 1: The Robotic Nervous System

**ROS 2 Fundamentals** | Weeks 3-5

## Module Overview

Just as the human nervous system coordinates signals between the brain and body, ROS 2
(Robot Operating System 2) serves as the communication backbone for robotic systems.
This module introduces the fundamental concepts that enable different parts of a robot
to work together seamlessly.

ROS 2 is the industry-standard middleware for professional robotics development.
Understanding its architecture is essential before working with simulation, AI
integration, or advanced perception systems covered in later modules.

By the end of this module, you'll have built working ROS 2 applications that
demonstrate the core communication patterns used in production robotics systems.

## Prerequisites

Before starting this module, you should:

- [ ] Have Python 3.10+ programming experience
- [ ] Be comfortable with Linux command line basics
- [ ] Have ROS 2 Humble installed on Ubuntu 22.04
- [ ] Have completed the [Introduction](/docs/intro)

...
```
