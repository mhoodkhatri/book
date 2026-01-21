# Quickstart: Content Authoring Reference

**Feature**: 002-book-content
**Purpose**: Quick reference for writing professional textbook content

---

## Content Writing Checklist

### Before Writing

- [ ] Read the spec: `specs/002-book-content/spec.md`
- [ ] Review chapter template: `contracts/chapter-template.md`
- [ ] Check terminology glossary below
- [ ] Identify target file in `docs/` structure

### During Writing

- [ ] Use active voice
- [ ] Include 2+ comments per code block
- [ ] Keep H2 sections to 500-1000 tokens
- [ ] Reference prerequisites explicitly
- [ ] Define acronyms on first use

### After Writing

- [ ] Run `npm run build` to validate Docusaurus
- [ ] Check for placeholder text ("coming soon", "TBD")
- [ ] Verify code is copy-paste runnable
- [ ] Confirm learning objectives are measurable
- [ ] Add Next Steps link

---

## Quick Templates

### Learning Objective

```markdown
- [Verb] [skill] [context]
```

**Examples**:
- Explain the role of nodes in ROS 2 architecture
- Create a publisher node that sends string messages
- Debug common topic communication issues

### Code Block

```python
#!/usr/bin/env python3
"""Brief description of purpose."""

import rclpy  # Comment explaining import
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        # Comment explaining key logic
        self.do_something()
```

### Checkpoint

```markdown
**Checkpoint**: After step N, verify:

```bash
[verification command]
```

**Expected**:
```
[expected output]
```
```

### Admonition (Docusaurus)

```markdown
:::info Hardware Requirements
Content here
:::

:::tip Pro Tip
Content here
:::

:::caution Warning
Content here
:::
```

---

## Terminology Glossary

Use these terms consistently. Never substitute.

| Correct Term | Never Use |
|--------------|-----------|
| node | process, module, component |
| topic | channel, stream, queue |
| publisher | sender, producer, emitter |
| subscriber | receiver, consumer, listener |
| message | packet, payload, data |
| service | RPC, API call |
| action | job, task (alone) |
| callback | handler |
| workspace | project, folder |
| package | module, library |
| launch file | startup script |
| URDF | robot model (be specific) |

---

## File Naming

| Content Type | Pattern | Example |
|--------------|---------|---------|
| Module index | `index.md` | `module-1-ros2/index.md` |
| Chapter | `kebab-case.md` | `nodes-topics.md` |
| Supporting | `kebab-case.md` | `workstation.md` |

---

## Content Lengths

| Section | Words | Tokens |
|---------|-------|--------|
| Chapter Overview | 110-190 | 150-250 |
| Learning Objectives | 60-110 | 80-150 |
| H2 Section | 375-750 | 500-1000 |
| H3 Subsection | 150-375 | 200-500 |
| Exercise | 225-375 | 300-500 |
| Summary | 75-150 | 100-200 |
| **Total Chapter** | **1500-3000** | **2000-4000** |

---

## Priority Order

Write content in this order:

1. **Module 1** (P1): Foundation - all subsequent content depends on this
   - `module-1-ros2/index.md`
   - `nodes-topics.md`
   - `services-actions.md`
   - `rclpy.md`
   - `urdf.md`

2. **Module 2** (P2): Simulation
   - `module-2-simulation/index.md`
   - `gazebo-basics.md`
   - `physics-sim.md`
   - `unity-integration.md`
   - `sensors.md`

3. **Module 3** (P3): NVIDIA Isaac
   - `module-3-nvidia-isaac/index.md`
   - `isaac-sim.md`
   - `isaac-ros.md`
   - `vslam.md`
   - `nav2.md`

4. **Module 4** (P4): VLA
   - `module-4-vla/index.md`
   - `whisper.md`
   - `cognitive-planning.md`
   - `capstone.md`

5. **Supporting** (P5):
   - `intro.md`
   - `hardware/workstation.md`
   - `hardware/edge-kit.md`
   - `appendix/assessments.md`
   - `appendix/resources.md`

---

## Code Language Tags

| Language | Fence Tag | Use For |
|----------|-----------|---------|
| Python | `python` | ROS 2 nodes, scripts |
| C++ | `cpp` | rclcpp nodes |
| Bash | `bash` | Terminal commands |
| YAML | `yaml` | Config files, params |
| XML | `xml` | URDF, launch (XML) |
| CMake | `cmake` | CMakeLists.txt |
| JSON | `json` | Config files |

---

## Forbidden Content

| Category | Examples |
|----------|----------|
| Placeholder | "coming soon", "TBD", "content here" |
| Marketing | "amazing", "revolutionary", "cutting-edge" |
| Casual | "gonna", "cool", "awesome", "stuff" |
| Emojis | Any emoji unless explicitly requested |
| First person | "I think", "I believe" |
| Incomplete code | Missing imports, pseudocode without label |

---

## Validation Commands

```bash
# Build site to check for errors
npm run build

# Serve locally for review
npm run start

# Check specific file syntax (if available)
npx markdownlint docs/path/to/file.md
```

---

## Quick Reference Links

- Spec: `specs/002-book-content/spec.md`
- Plan: `specs/002-book-content/plan.md`
- Research: `specs/002-book-content/research.md`
- Data Model: `specs/002-book-content/data-model.md`
- Chapter Template: `contracts/chapter-template.md`
- Module Template: `contracts/module-index-template.md`
- Exercise Template: `contracts/exercise-template.md`
