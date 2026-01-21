# Data Model: Professional Textbook Content

**Feature**: 002-book-content
**Date**: 2026-01-19
**Purpose**: Define content entities, their structure, and relationships

---

## Entity Overview

This feature deals with **content entities** rather than database entities. The data model describes the structure of textbook content for authoring consistency.

```
┌─────────────────────────────────────────────────────────────────┐
│                          TEXTBOOK                                │
│  (Physical AI & Humanoid Robotics)                              │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐       │
│  │ MODULE 1 │  │ MODULE 2 │  │ MODULE 3 │  │ MODULE 4 │       │
│  │  (ROS2)  │  │  (Sim)   │  │ (Isaac)  │  │  (VLA)   │       │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘       │
│       │             │             │             │               │
│  ┌────┴────┐   ┌────┴────┐   ┌────┴────┐   ┌────┴────┐        │
│  │ 4 Chaps │   │ 4 Chaps │   │ 4 Chaps │   │ 3 Chaps │        │
│  └─────────┘   └─────────┘   └─────────┘   └─────────┘        │
│                                                                  │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │                  SUPPORTING CONTENT                      │   │
│  │  intro.md | hardware/ | appendix/                       │   │
│  └─────────────────────────────────────────────────────────┘   │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## Entity Definitions

### 1. Module

A major thematic unit containing related chapters.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| id | string | Yes | Unique identifier (e.g., "module-1-ros2") |
| title | string | Yes | Display title (e.g., "The Robotic Nervous System") |
| subtitle | string | Yes | Technical subtitle (e.g., "ROS 2 Fundamentals") |
| sidebar_position | number | Yes | Order in navigation |
| description | string | Yes | 1-2 sentence summary for SEO/preview |
| keywords | string[] | Yes | SEO keywords array |
| prerequisites | string[] | Yes | Required prior knowledge |
| learning_outcomes | string[] | Yes | Module-level outcomes (4-6 items) |
| chapters | Chapter[] | Yes | Ordered list of chapters |
| estimated_time | string | No | Suggested completion time |

**File Location**: `docs/module-X-name/index.md`

**Validation Rules**:
- `id` must match directory name
- `chapters` must reference existing chapter files
- `learning_outcomes` must be verifiable statements

---

### 2. Chapter

A focused lesson unit within a module.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| id | string | Yes | Unique identifier (e.g., "nodes-topics") |
| title | string | Yes | Chapter title with number (e.g., "Chapter 1: Nodes and Topics") |
| sidebar_position | number | Yes | Order within module |
| description | string | Yes | SEO description (150-160 chars) |
| keywords | string[] | Yes | SEO keywords (5-10 terms) |
| prerequisites | string[] | No | Required prior chapters/knowledge |
| learning_objectives | LearningObjective[] | Yes | 4-6 verifiable objectives |
| sections | Section[] | Yes | Content sections |
| exercises | Exercise[] | Yes | At least 1 hands-on exercise |
| summary_points | string[] | Yes | 5-7 key takeaways |
| next_chapter | string | No | Link to next chapter |

**File Location**: `docs/module-X-name/chapter-file.md`

**Validation Rules**:
- Must contain at least 1 code snippet
- Must contain at least 1 exercise
- `learning_objectives` must be measurable
- Total content: 2000-4000 words

---

### 3. Section

A semantically complete content block within a chapter.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| heading | string | Yes | H2 or H3 heading text |
| level | number | Yes | Heading level (2 or 3) |
| content | string | Yes | Markdown content |
| token_count | number | No | Estimated tokens (target: 500-1000) |
| code_snippets | CodeSnippet[] | No | Code blocks in section |

**Validation Rules**:
- H2 sections: 500-1000 tokens
- H3 sections: 200-500 tokens
- No ambiguous pronouns crossing section boundaries
- Key terms appear in first 50 words

---

### 4. CodeSnippet

A complete, runnable code example.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| language | string | Yes | Code fence language (python, bash, yaml, cpp, cmake) |
| filename | string | No | Suggested filename for reader |
| content | string | Yes | Full code content |
| comments_count | number | Yes | Number of inline comments (min: 2) |
| runnable | boolean | Yes | Whether code is copy-paste executable |
| prerequisites | string[] | No | Required setup to run |
| expected_output | string | No | Expected terminal/result output |

**Supported Languages**:
- `python` - ROS 2 Python (rclpy)
- `cpp` - ROS 2 C++ (rclcpp)
- `bash` - Shell commands
- `yaml` - Configuration files
- `cmake` - CMakeLists.txt
- `xml` - URDF, launch files
- `json` - Configuration

**Validation Rules**:
- Minimum 2 inline comments (SC-008)
- If `runnable: true`, must include all imports
- If `runnable: true`, must be syntactically valid
- `expected_output` required for command-line examples

---

### 5. Exercise

A hands-on practice activity.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| title | string | Yes | Exercise title |
| objective | string | Yes | Clear goal statement |
| prerequisites | string[] | Yes | Required setup |
| difficulty | enum | Yes | "beginner" | "intermediate" | "advanced" |
| steps | ExerciseStep[] | Yes | Numbered instructions |
| checkpoints | Checkpoint[] | Yes | Verification points |
| stretch_goals | string[] | No | Optional advanced challenges |
| estimated_time | string | No | Expected completion time |

**Validation Rules**:
- At least 3 steps
- At least 1 checkpoint
- Steps must be atomic and numbered
- `objective` must be measurable

---

### 6. ExerciseStep

A single instruction within an exercise.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| number | number | Yes | Step number |
| instruction | string | Yes | What to do |
| code | CodeSnippet | No | Associated code |
| note | string | No | Additional context or warning |

---

### 7. Checkpoint

A verification point within an exercise.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| after_step | number | Yes | Step number this follows |
| description | string | Yes | What to verify |
| verification_command | string | No | Command to check |
| expected_result | string | Yes | What success looks like |

---

### 8. LearningObjective

A measurable learning outcome.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| verb | string | Yes | Action verb (Bloom's taxonomy) |
| skill | string | Yes | What learner will do |
| context | string | No | Specific context |

**Valid Verbs** (Bloom's Taxonomy):
- Remember: define, list, recall, identify
- Understand: explain, describe, summarize, interpret
- Apply: implement, execute, use, demonstrate
- Analyze: differentiate, compare, debug, troubleshoot
- Evaluate: assess, evaluate, justify, critique
- Create: design, construct, build, develop

**Examples**:
- "Explain the role of nodes in ROS 2 architecture"
- "Create a publisher node that sends string messages"
- "Debug common topic communication issues"

---

## Entity Relationships

```
TEXTBOOK (1)
    │
    ├─── intro.md (1)
    │
    ├─── MODULE (4)
    │       │
    │       └─── CHAPTER (3-4 per module)
    │               │
    │               ├─── SECTION (5-8 per chapter)
    │               │       │
    │               │       └─── CODE_SNIPPET (0-3 per section)
    │               │
    │               ├─── EXERCISE (1-2 per chapter)
    │               │       │
    │               │       ├─── EXERCISE_STEP (3-10 per exercise)
    │               │       │
    │               │       └─── CHECKPOINT (1-3 per exercise)
    │               │
    │               └─── LEARNING_OBJECTIVE (4-6 per chapter)
    │
    ├─── hardware/ (2 pages)
    │
    └─── appendix/ (2 pages)
```

---

## Content Inventory with Entities

| File | Entity Type | Status |
|------|-------------|--------|
| `docs/intro.md` | Textbook Introduction | Placeholder |
| `docs/module-1-ros2/index.md` | Module | Placeholder |
| `docs/module-1-ros2/nodes-topics.md` | Chapter | Placeholder |
| `docs/module-1-ros2/services-actions.md` | Chapter | Placeholder |
| `docs/module-1-ros2/rclpy.md` | Chapter | Placeholder |
| `docs/module-1-ros2/urdf.md` | Chapter | Placeholder |
| `docs/module-2-simulation/index.md` | Module | Placeholder |
| `docs/module-2-simulation/gazebo-basics.md` | Chapter | Placeholder |
| `docs/module-2-simulation/physics-sim.md` | Chapter | Placeholder |
| `docs/module-2-simulation/unity-integration.md` | Chapter | Placeholder |
| `docs/module-2-simulation/sensors.md` | Chapter | Placeholder |
| `docs/module-3-nvidia-isaac/index.md` | Module | Placeholder |
| `docs/module-3-nvidia-isaac/isaac-sim.md` | Chapter | Placeholder |
| `docs/module-3-nvidia-isaac/isaac-ros.md` | Chapter | Placeholder |
| `docs/module-3-nvidia-isaac/vslam.md` | Chapter | Placeholder |
| `docs/module-3-nvidia-isaac/nav2.md` | Chapter | Placeholder |
| `docs/module-4-vla/index.md` | Module | Placeholder |
| `docs/module-4-vla/whisper.md` | Chapter | Placeholder |
| `docs/module-4-vla/cognitive-planning.md` | Chapter | Placeholder |
| `docs/module-4-vla/capstone.md` | Chapter | Placeholder |
| `docs/hardware/workstation.md` | Supporting | Placeholder |
| `docs/hardware/edge-kit.md` | Supporting | Placeholder |
| `docs/appendix/assessments.md` | Supporting | Placeholder |
| `docs/appendix/resources.md` | Supporting | Placeholder |

**Total**: 24 content pages (15 chapters + 4 module indexes + 1 intro + 4 supporting)

---

## Validation Checklist

Before marking any chapter as complete, verify:

- [ ] YAML frontmatter complete (sidebar_position, title, description, keywords)
- [ ] Chapter overview section present
- [ ] 4-6 learning objectives with action verbs
- [ ] All H2 sections 500-1000 tokens
- [ ] All code snippets have 2+ comments
- [ ] All code snippets syntactically valid
- [ ] At least 1 hands-on exercise
- [ ] Exercise has checkpoints
- [ ] Summary with 5-7 bullet points
- [ ] Next Steps link (except final chapter in module)
- [ ] No placeholder text ("coming soon", "TBD", etc.)
- [ ] No marketing language or casual tone
- [ ] Terminology matches glossary
