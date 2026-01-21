# Tasks: Professional Textbook Content

**Input**: Design documents from `/specs/002-book-content/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: No test tasks included - not explicitly requested in the feature specification. This is a content authoring feature where validation is done through Docusaurus build and manual review.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story. Each module can be completed and validated independently.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4, US5)
- Include exact file paths in descriptions

## Path Conventions

- **Documentation**: `docs/` at repository root (Docusaurus content)
- **Modules**: `docs/module-X-name/` for each module
- **Supporting**: `docs/hardware/`, `docs/appendix/`
- **Templates**: `specs/002-book-content/contracts/` for reference

---

## Phase 1: Setup (Content Authoring Infrastructure)

**Purpose**: Validate existing infrastructure and prepare for content authoring

- [ ] T001 Validate Docusaurus build works with existing placeholder content by running `npm run build`
- [ ] T002 [P] Review and confirm chapter template structure in specs/002-book-content/contracts/chapter-template.md
- [ ] T003 [P] Review and confirm module index template in specs/002-book-content/contracts/module-index-template.md
- [ ] T004 [P] Review terminology glossary in specs/002-book-content/quickstart.md

---

## Phase 2: Foundational (Course Introduction)

**Purpose**: Create the course introduction that provides context for ALL modules

**⚠️ CRITICAL**: The introduction establishes the textbook's scope and prepares readers for all subsequent content

- [ ] T005 Write complete course introduction content in docs/intro.md
  - Book purpose and target audience
  - Course structure overview (4 modules)
  - Prerequisites and learning path
  - How to use this textbook
  - Hardware and software requirements summary

**Checkpoint**: Introduction complete - readers have context to begin any module

---

## Phase 3: User Story 1 - Engineering Student Learns ROS 2 (Priority: P1) 🎯 MVP

**Goal**: Complete Module 1 so students can learn ROS 2 fundamentals and build their first publisher-subscriber system

**Independent Test**: A reader with Python experience can complete Module 1 Chapter 1 and successfully create a working ROS 2 node with publisher and subscriber

### Module 1 Index

- [ ] T006 [US1] Write Module 1 index content in docs/module-1-ros2/index.md
  - Module overview (nervous system metaphor)
  - Prerequisites checklist
  - Learning outcomes (4-6 items)
  - Chapter overview with key topics
  - Learning path diagram
  - Getting started link

### Chapter 1.1: Nodes and Topics

- [ ] T007 [US1] Write Chapter 1.1 content in docs/module-1-ros2/nodes-topics.md
  - YAML frontmatter (sidebar_position, title, description, keywords)
  - Chapter overview with prerequisites
  - Learning objectives (4-6 verifiable)
  - Conceptual explanation: What are nodes? (500-1000 tokens)
  - Conceptual explanation: Topics and publish-subscribe pattern (500-1000 tokens)
  - Practical example: Create minimal publisher node
  - Practical example: Create minimal subscriber node
  - Hands-on exercise: Build custom pub-sub system
  - Summary (5-7 bullet points)
  - Next steps link to Chapter 1.2

### Chapter 1.2: Services and Actions

- [ ] T008 [US1] Write Chapter 1.2 content in docs/module-1-ros2/services-actions.md
  - YAML frontmatter
  - Chapter overview with prerequisites (Chapter 1.1)
  - Learning objectives (4-6 verifiable)
  - Conceptual explanation: Request-response with services (500-1000 tokens)
  - Conceptual explanation: Long-running tasks with actions (500-1000 tokens)
  - Practical example: Create a service server and client
  - Practical example: Create an action server and client
  - Hands-on exercise: Build a service-based application
  - Summary and next steps

### Chapter 1.3: Python Client Library (rclpy)

- [ ] T009 [US1] Write Chapter 1.3 content in docs/module-1-ros2/rclpy.md
  - YAML frontmatter
  - Chapter overview with prerequisites (Chapters 1.1, 1.2)
  - Learning objectives (4-6 verifiable)
  - Conceptual explanation: rclpy architecture (500-1000 tokens)
  - Conceptual explanation: Node lifecycle and executors (500-1000 tokens)
  - Practical example: Complete rclpy node patterns
  - Practical example: Parameters and configuration
  - Hands-on exercise: Build parameterized node
  - Summary and next steps

### Chapter 1.4: Robot Description Format (URDF)

- [ ] T010 [US1] Write Chapter 1.4 content in docs/module-1-ros2/urdf.md
  - YAML frontmatter
  - Chapter overview with prerequisites
  - Learning objectives (4-6 verifiable)
  - Conceptual explanation: URDF structure and elements (500-1000 tokens)
  - Conceptual explanation: Links, joints, and transformations (500-1000 tokens)
  - Practical example: Create a simple robot URDF
  - Practical example: Visualize URDF with RViz2
  - Hands-on exercise: Build a robot arm URDF
  - Summary and next steps (link to Module 2)

**Checkpoint**: Module 1 complete and independently testable. Reader can create ROS 2 nodes, use topics/services/actions, work with rclpy, and define robot descriptions.

---

## Phase 4: User Story 2 - Robotics Practitioner Implements Simulation (Priority: P2)

**Goal**: Complete Module 2 so practitioners can create digital twins with accurate physics and sensors

**Independent Test**: Reader can launch a Gazebo simulation with a robot model and demonstrate sensor data publication to ROS 2 topics

### Module 2 Index

- [ ] T011 [US2] Write Module 2 index content in docs/module-2-simulation/index.md
  - Module overview (digital twin metaphor)
  - Prerequisites (Module 1 completion)
  - Learning outcomes (4-6 items)
  - Chapter overview with key topics
  - Learning path diagram
  - Getting started link

### Chapter 2.1: Gazebo Basics

- [ ] T012 [US2] Write Chapter 2.1 content in docs/module-2-simulation/gazebo-basics.md
  - YAML frontmatter
  - Chapter overview with prerequisites
  - Learning objectives (4-6 verifiable)
  - Conceptual explanation: Gazebo architecture and SDF (500-1000 tokens)
  - Conceptual explanation: World files and model spawning (500-1000 tokens)
  - Practical example: Launch Gazebo with empty world
  - Practical example: Spawn URDF robot in Gazebo
  - Hands-on exercise: Create first Gazebo simulation
  - Summary and next steps

### Chapter 2.2: Physics Simulation

- [ ] T013 [US2] Write Chapter 2.2 content in docs/module-2-simulation/physics-sim.md
  - YAML frontmatter
  - Chapter overview with prerequisites (Chapter 2.1)
  - Learning objectives (4-6 verifiable)
  - Conceptual explanation: Physics engines and collision detection (500-1000 tokens)
  - Conceptual explanation: Inertia, friction, and contact dynamics (500-1000 tokens)
  - Practical example: Configure physics properties
  - Practical example: Simulate robot-object interaction
  - Hands-on exercise: Tune physics for realistic behavior
  - Summary and next steps

### Chapter 2.3: Unity Integration

- [ ] T014 [US2] Write Chapter 2.3 content in docs/module-2-simulation/unity-integration.md
  - YAML frontmatter
  - Chapter overview with prerequisites
  - Learning objectives (4-6 verifiable)
  - Conceptual explanation: Unity-ROS 2 communication (500-1000 tokens)
  - Conceptual explanation: When to use Unity vs Gazebo (500-1000 tokens)
  - Practical example: Setup Unity with ROS 2 connector
  - Practical example: Publish sensor data from Unity
  - Hands-on exercise: Create Unity-ROS 2 pipeline
  - Summary and next steps

### Chapter 2.4: Sensor Simulation

- [ ] T015 [US2] Write Chapter 2.4 content in docs/module-2-simulation/sensors.md
  - YAML frontmatter
  - Chapter overview with prerequisites
  - Learning objectives (4-6 verifiable)
  - Conceptual explanation: Camera, lidar, and IMU models (500-1000 tokens)
  - Conceptual explanation: Noise models and calibration (500-1000 tokens)
  - Practical example: Add camera sensor to robot
  - Practical example: Add lidar and publish point clouds
  - Hands-on exercise: Build multi-sensor robot simulation
  - Summary and next steps (link to Module 3)

**Checkpoint**: Module 2 complete and independently testable. Reader can create Gazebo simulations, configure physics, integrate Unity, and simulate sensors.

---

## Phase 5: User Story 3 - AI Developer Integrates NVIDIA Isaac (Priority: P3)

**Goal**: Complete Module 3 so AI developers can deploy autonomous navigation on simulated robots

**Independent Test**: Reader can deploy the Nav2 stack on a simulated robot and demonstrate autonomous navigation to a goal position

### Module 3 Index

- [ ] T016 [US3] Write Module 3 index content in docs/module-3-nvidia-isaac/index.md
  - Module overview (AI-robot brain metaphor)
  - Prerequisites (Modules 1-2, NVIDIA GPU)
  - Hardware requirements box
  - Learning outcomes (4-6 items)
  - Chapter overview with key topics
  - Learning path diagram
  - Getting started link

### Chapter 3.1: NVIDIA Isaac Sim

- [ ] T017 [US3] Write Chapter 3.1 content in docs/module-3-nvidia-isaac/isaac-sim.md
  - YAML frontmatter
  - Chapter overview with hardware requirements
  - Learning objectives (4-6 verifiable)
  - Conceptual explanation: Isaac Sim architecture and Omniverse (500-1000 tokens)
  - Conceptual explanation: USD format and scene building (500-1000 tokens)
  - Practical example: Launch Isaac Sim and navigate UI
  - Practical example: Import robot and create environment
  - Hands-on exercise: Build Isaac Sim scene
  - Summary and next steps

### Chapter 3.2: Isaac ROS Integration

- [ ] T018 [US3] Write Chapter 3.2 content in docs/module-3-nvidia-isaac/isaac-ros.md
  - YAML frontmatter
  - Chapter overview with prerequisites (Chapter 3.1)
  - Learning objectives (4-6 verifiable)
  - Conceptual explanation: Isaac ROS package ecosystem (500-1000 tokens)
  - Conceptual explanation: GPU-accelerated perception pipeline (500-1000 tokens)
  - Practical example: Setup Isaac ROS workspace
  - Practical example: Run perception pipeline
  - Hands-on exercise: Integrate Isaac ROS with simulation
  - Summary and next steps

### Chapter 3.3: Visual SLAM

- [ ] T019 [US3] Write Chapter 3.3 content in docs/module-3-nvidia-isaac/vslam.md
  - YAML frontmatter
  - Chapter overview with prerequisites
  - Learning objectives (4-6 verifiable)
  - Conceptual explanation: SLAM fundamentals (500-1000 tokens)
  - Conceptual explanation: Visual odometry and loop closure (500-1000 tokens)
  - Practical example: Run Isaac ROS Visual SLAM
  - Practical example: Build map from exploration
  - CPU alternative section with ORB-SLAM2
  - Hands-on exercise: Map an environment
  - Summary and next steps

### Chapter 3.4: Navigation 2 Stack

- [ ] T020 [US3] Write Chapter 3.4 content in docs/module-3-nvidia-isaac/nav2.md
  - YAML frontmatter
  - Chapter overview with prerequisites
  - Learning objectives (4-6 verifiable)
  - Conceptual explanation: Nav2 architecture (500-1000 tokens)
  - Conceptual explanation: Path planning and behavior trees (500-1000 tokens)
  - Practical example: Configure Nav2 for robot
  - Practical example: Send navigation goals
  - Hands-on exercise: Autonomous navigation mission
  - Summary and next steps (link to Module 4)

**Checkpoint**: Module 3 complete and independently testable. Reader can use Isaac Sim, integrate Isaac ROS, implement VSLAM, and deploy Nav2 for autonomous navigation.

---

## Phase 6: User Story 4 - Researcher Builds VLA Pipeline (Priority: P4)

**Goal**: Complete Module 4 so researchers can build end-to-end voice-to-action pipelines

**Independent Test**: Reader can complete the capstone project and demonstrate a robot responding to a voice command with appropriate action

### Module 4 Index

- [ ] T021 [US4] Write Module 4 index content in docs/module-4-vla/index.md
  - Module overview (VLA research frontier)
  - Prerequisites (Modules 1-3)
  - Learning outcomes (4-6 items)
  - Chapter overview with key topics
  - Learning path diagram
  - Getting started link

### Chapter 4.1: Speech to Action with Whisper

- [ ] T022 [US4] Write Chapter 4.1 content in docs/module-4-vla/whisper.md
  - YAML frontmatter
  - Chapter overview with prerequisites
  - Learning objectives (4-6 verifiable)
  - Conceptual explanation: Whisper model architecture (500-1000 tokens)
  - Conceptual explanation: ROS 2 integration patterns (500-1000 tokens)
  - Practical example: Setup Whisper for real-time transcription
  - Practical example: Create speech command ROS 2 node
  - Hands-on exercise: Voice-controlled robot commands
  - Summary and next steps

### Chapter 4.2: Cognitive Planning with LLMs

- [ ] T023 [US4] Write Chapter 4.2 content in docs/module-4-vla/cognitive-planning.md
  - YAML frontmatter
  - Chapter overview with prerequisites (Chapter 4.1)
  - Learning objectives (4-6 verifiable)
  - Conceptual explanation: LLM task decomposition (500-1000 tokens)
  - Conceptual explanation: LangChain/LlamaIndex for robotics (500-1000 tokens)
  - Practical example: Create task planning service
  - Practical example: Generate action sequences from natural language
  - Hands-on exercise: Build cognitive planning pipeline
  - Summary and next steps

### Chapter 4.3: Capstone Project

- [ ] T024 [US4] Write Chapter 4.3 content in docs/module-4-vla/capstone.md
  - YAML frontmatter
  - Chapter overview (integration of all modules)
  - Learning objectives (4-6 verifiable)
  - Project specification: Voice-controlled robot assistant
  - Architecture diagram and component overview
  - Step-by-step integration guide
  - Testing and debugging guidance
  - Stretch goals and extensions
  - Summary of course completion

**Checkpoint**: Module 4 complete and independently testable. Reader can integrate Whisper for speech recognition, use LLMs for task planning, and build a complete VLA pipeline.

---

## Phase 7: User Story 5 - Technical Reviewer Evaluates Textbook (Priority: P5)

**Goal**: Complete supporting content so reviewers can assess the textbook's professional quality

**Independent Test**: External reviewer can assess the textbook against a quality rubric and find no factual errors, clear explanations, and working code across all chapters

### Hardware Documentation

- [ ] T025 [P] [US5] Write workstation requirements in docs/hardware/workstation.md
  - Development workstation specifications
  - Minimum vs recommended specs
  - GPU requirements for Module 3
  - Software installation checklist
  - Troubleshooting common setup issues

- [ ] T026 [P] [US5] Write edge kit documentation in docs/hardware/edge-kit.md
  - Jetson Orin Nano specifications
  - Use cases and limitations
  - Setup and configuration guide
  - Deployment considerations

### Appendix Content

- [ ] T027 [P] [US5] Write assessment rubrics in docs/appendix/assessments.md
  - Chapter completion criteria
  - Exercise evaluation rubrics
  - Module assessment guidelines
  - Self-assessment checklists

- [ ] T028 [P] [US5] Write additional resources in docs/appendix/resources.md
  - Official documentation links
  - Community resources
  - Recommended reading
  - Video tutorials and courses
  - Research papers for further study

**Checkpoint**: Supporting content complete. Textbook has comprehensive hardware guidance, assessment tools, and additional resources.

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Final quality assurance and consistency checks across all content

- [ ] T029 [P] Run Docusaurus build validation with `npm run build`
- [ ] T030 [P] Verify all code snippets have minimum 2 inline comments
- [ ] T031 Verify terminology consistency across all chapters (node, topic, publisher, etc.)
- [ ] T032 Verify all chapters have complete YAML frontmatter
- [ ] T033 Verify all chapters have learning objectives with Bloom's taxonomy verbs
- [ ] T034 Verify all practical examples have expected output
- [ ] T035 Verify all exercises have checkpoints and verification commands
- [ ] T036 [P] Verify all Next Steps links are correct and functional
- [ ] T037 Remove any remaining placeholder text ("coming soon", "TBD", etc.)
- [ ] T038 Final review for marketing language or casual tone violations
- [ ] T039 Run quickstart.md validation checklist against completed content

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - validates infrastructure
- **Foundational (Phase 2)**: Depends on Setup - intro provides context
- **User Story 1-4 (Phases 3-6)**: Depends on Foundational
  - US1 (Module 1): Can start after Foundational - **MVP delivery point**
  - US2 (Module 2): Can start after Foundational, benefits from US1 completion for cross-references
  - US3 (Module 3): Can start after Foundational, builds on US1-2 concepts
  - US4 (Module 4): Can start after Foundational, integrates all prior modules
- **User Story 5 (Phase 7)**: Can run in parallel with any user story - supporting content
- **Polish (Phase 8)**: Depends on all content phases being complete

### User Story Dependencies

- **US1 (Module 1)**: No dependencies on other stories - **FOUNDATIONAL, DO FIRST**
- **US2 (Module 2)**: Conceptually builds on US1 but can be written independently
- **US3 (Module 3)**: Conceptually builds on US1-2 but can be written independently
- **US4 (Module 4)**: Integrates all modules - most dependent on prior content
- **US5 (Supporting)**: Independent - can be written in parallel with any module

### Within Each Module

- Module index MUST be written first (provides context)
- Chapters can be written sequentially (1 → 2 → 3 → 4) or in parallel if different authors
- Each chapter follows: frontmatter → overview → concepts → examples → exercise → summary

### Parallel Opportunities

**Phase 1 (Setup)**:
```
Task: T002 Review chapter template
Task: T003 Review module index template
Task: T004 Review terminology glossary
```

**User Story 5 (Supporting - can run parallel to any module)**:
```
Task: T025 Write workstation requirements
Task: T026 Write edge kit documentation
Task: T027 Write assessment rubrics
Task: T028 Write additional resources
```

**Phase 8 (Polish - multiple parallel checks)**:
```
Task: T029 Run Docusaurus build
Task: T030 Verify code comments
Task: T036 Verify Next Steps links
```

---

## Parallel Example: Module 1 Chapters

If multiple authors are available, Module 1 chapters can be written in parallel after the index:

```bash
# First, complete the index (T006):
Task: "Write Module 1 index content in docs/module-1-ros2/index.md"

# Then, launch all chapters in parallel:
Task: "Write Chapter 1.1 content in docs/module-1-ros2/nodes-topics.md"
Task: "Write Chapter 1.2 content in docs/module-1-ros2/services-actions.md"
Task: "Write Chapter 1.3 content in docs/module-1-ros2/rclpy.md"
Task: "Write Chapter 1.4 content in docs/module-1-ros2/urdf.md"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (validate infrastructure)
2. Complete Phase 2: Foundational (write intro.md)
3. Complete Phase 3: User Story 1 (Module 1 - all 5 files)
4. **STOP and VALIDATE**: Run `npm run build`, review content against templates
5. **MVP DELIVERED**: Students can learn ROS 2 fundamentals

### Incremental Delivery

1. Setup + Foundational → Infrastructure ready
2. Add User Story 1 (Module 1) → Validate → **MVP for ROS 2 basics**
3. Add User Story 2 (Module 2) → Validate → **Simulation capability added**
4. Add User Story 3 (Module 3) → Validate → **Isaac/Nav2 capability added**
5. Add User Story 4 (Module 4) → Validate → **Complete VLA pipeline**
6. Add User Story 5 (Supporting) → Validate → **Full textbook complete**
7. Polish phase → Final quality checks

### Recommended Single-Author Path

For a single author, follow priority order:

1. **T001-T004**: Setup validation
2. **T005**: Write intro.md (foundational)
3. **T006-T010**: Complete Module 1 sequentially (US1/P1)
4. **T011-T015**: Complete Module 2 sequentially (US2/P2)
5. **T016-T020**: Complete Module 3 sequentially (US3/P3)
6. **T021-T024**: Complete Module 4 sequentially (US4/P4)
7. **T025-T028**: Complete supporting content (US5/P5)
8. **T029-T039**: Polish and validation

---

## Content Quality Checklist (Per Chapter)

Before marking any chapter task as complete, verify:

- [ ] YAML frontmatter complete (sidebar_position, title, description, keywords)
- [ ] Chapter overview present with prerequisites
- [ ] 4-6 learning objectives with Bloom's taxonomy verbs
- [ ] All H2 sections 500-1000 tokens
- [ ] All code snippets have 2+ inline comments
- [ ] All code snippets syntactically valid for ROS 2 Humble
- [ ] At least 1 hands-on exercise with checkpoints
- [ ] Summary with 5-7 bullet points
- [ ] Next Steps link (except final chapter in textbook)
- [ ] No placeholder text
- [ ] No marketing language or casual tone
- [ ] Terminology matches glossary

---

## Summary

| Metric | Count |
|--------|-------|
| **Total Tasks** | 39 |
| **Setup Tasks** | 4 |
| **Foundational Tasks** | 1 |
| **US1 (Module 1) Tasks** | 5 |
| **US2 (Module 2) Tasks** | 5 |
| **US3 (Module 3) Tasks** | 5 |
| **US4 (Module 4) Tasks** | 4 |
| **US5 (Supporting) Tasks** | 4 |
| **Polish Tasks** | 11 |
| **Parallel Opportunities** | 15 tasks marked [P] |

| User Story | Files | Priority |
|------------|-------|----------|
| US1 (Module 1) | 5 (1 index + 4 chapters) | P1 - MVP |
| US2 (Module 2) | 5 (1 index + 4 chapters) | P2 |
| US3 (Module 3) | 5 (1 index + 4 chapters) | P3 |
| US4 (Module 4) | 4 (1 index + 3 chapters) | P4 |
| US5 (Supporting) | 4 (hardware + appendix) | P5 |
| Foundational | 1 (intro.md) | Required |
| **Total Content** | **24 files** | |

**MVP Scope**: Complete Setup (T001-T004), Foundational (T005), and User Story 1 (T006-T010) = 10 tasks for a functional ROS 2 fundamentals module.
