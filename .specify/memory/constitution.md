<!--
Sync Impact Report
==================
Version change: 0.0.0 → 1.0.0 (Initial constitution)
Modified principles: N/A (new file)
Added sections:
  - Core Principles (7 principles)
  - Technical Stack Constraints
  - Content Architecture
  - Bonus Features Requirements
  - Development Workflow
  - Governance
Removed sections: N/A
Templates requiring updates:
  - .specify/templates/plan-template.md ✅ (compatible - no updates needed)
  - .specify/templates/spec-template.md ✅ (compatible - no updates needed)
  - .specify/templates/tasks-template.md ✅ (compatible - no updates needed)
Follow-up TODOs: None
==================
-->

# Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles

### I. Docusaurus-First Documentation

All textbook content MUST be authored in Docusaurus-compatible MDX format. The book
structure MUST follow Docusaurus conventions with proper sidebar configuration, versioning
support, and static asset management. Content MUST be deployable to GitHub Pages without
modification. No proprietary documentation formats are permitted.

**Rationale**: Ensures consistent rendering, searchability, and maintainability across the
entire textbook while enabling seamless CI/CD deployment.

### II. Spec-Driven Development (SDD)

Every feature MUST begin with a specification (`/sp.specify`), proceed through planning
(`/sp.plan`), generate tasks (`/sp.tasks`), and follow the implementation workflow. No code
or content MUST be written without a corresponding spec. All changes MUST trace back to
documented requirements.

**Rationale**: Maintains traceability between requirements and implementation, enables
systematic progress tracking, and ensures nothing is built without explicit justification.

### III. RAG-First Content Design

All textbook content MUST be structured for optimal retrieval by the embedded RAG chatbot.
Chapters MUST use clear semantic headings, avoid ambiguous pronouns across section
boundaries, and include explicit keyword anchors. Content MUST be chunked appropriately
for vector embedding (target: 500-1000 tokens per logical section).

**Rationale**: The RAG chatbot is a core deliverable. Content that cannot be effectively
retrieved defeats the purpose of the AI-native textbook.

### IV. Modular Content Architecture

The book MUST strictly follow the 4-Module structure:
- **Module 1**: The Robotic Nervous System (ROS 2)
- **Module 2**: The Digital Twin (Gazebo & Unity)
- **Module 3**: The AI-Robot Brain (NVIDIA Isaac)
- **Module 4**: Vision-Language-Action (VLA)

Each module MUST be independently readable while maintaining narrative flow. Cross-module
references MUST use explicit links rather than implicit assumptions.

**Rationale**: Matches the course curriculum and enables incremental learning paths.

### V. Code-Content Parity

Every code example in the textbook MUST be:
1. Syntactically correct and executable in the stated environment
2. Accompanied by clear prerequisites (dependencies, environment setup)
3. Tested in the CI pipeline where applicable
4. Versioned with explicit framework/library versions

No pseudocode or partial snippets without explicit "[Partial]" or "[Conceptual]" labels.

**Rationale**: Students will copy-paste code. Broken examples destroy trust and learning.

### VI. Accessibility-First Design

All content MUST:
- Support the Personalization button (adapt difficulty based on user background)
- Support the Urdu Translation button (maintain technical accuracy in translation)
- Include alt-text for all diagrams and images
- Use semantic HTML in MDX components

**Rationale**: Bonus points require personalization and translation. Accessibility is
non-negotiable for an educational resource.

### VII. Security and Data Integrity

- Authentication MUST use Better-Auth with secure session management
- User background data MUST be stored securely in Neon Postgres
- API keys and secrets MUST NEVER be committed; use environment variables
- RAG queries MUST be sanitized to prevent prompt injection
- All user data handling MUST comply with basic privacy principles

**Rationale**: Educational platforms handle user data. Security failures are unacceptable.

## Technical Stack Constraints

The following technology stack is MANDATORY per hackathon requirements:

| Component | Technology | Constraint |
|-----------|------------|------------|
| Documentation | Docusaurus | Latest stable version |
| Hosting | GitHub Pages | Primary deployment target |
| Backend | FastAPI | Python 3.10+ |
| Database | Neon Serverless Postgres | Free tier compatible |
| Vector Store | Qdrant Cloud | Free tier compatible |
| LLM Integration | OpenAI Agents/ChatKit SDK | Latest stable |
| Authentication | Better-Auth | Required for bonus points |
| Development | Claude Code + Spec-Kit Plus | Mandatory workflow |

**Deviation Policy**: Stack changes require explicit justification in an ADR and user
approval. No stack component may be substituted without documented rationale.

## Content Architecture

### Book Structure (Strict Adherence Required)

```
docs/
├── intro.md                    # Course overview, learning outcomes
├── module-1-ros2/
│   ├── index.md                # Module introduction
│   ├── 01-nodes-topics.md      # ROS 2 Nodes and Topics
│   ├── 02-services-actions.md  # Services and Actions
│   ├── 03-rclpy.md             # Python integration with rclpy
│   └── 04-urdf.md              # URDF for humanoids
├── module-2-simulation/
│   ├── index.md
│   ├── 01-gazebo-basics.md     # Gazebo fundamentals
│   ├── 02-physics-sim.md       # Physics simulation
│   ├── 03-unity-integration.md # Unity for visualization
│   └── 04-sensors.md           # LiDAR, Depth Cameras, IMUs
├── module-3-nvidia-isaac/
│   ├── index.md
│   ├── 01-isaac-sim.md         # Isaac Sim introduction
│   ├── 02-isaac-ros.md         # Isaac ROS integration
│   ├── 03-vslam.md             # Visual SLAM
│   └── 04-nav2.md              # Navigation for bipeds
├── module-4-vla/
│   ├── index.md
│   ├── 01-whisper.md           # Voice-to-Action with Whisper
│   ├── 02-cognitive-planning.md # LLM cognitive planning
│   └── 03-capstone.md          # Autonomous Humanoid project
├── hardware/
│   ├── workstation.md          # Digital Twin Workstation specs
│   └── edge-kit.md             # Jetson Orin Nano kit
└── appendix/
    ├── assessments.md          # Course assessments
    └── resources.md            # Additional resources
```

### Weekly Mapping

| Weeks | Content |
|-------|---------|
| 1-2 | intro.md + Module 1 start |
| 3-5 | Module 1 (ROS 2 Fundamentals) |
| 6-7 | Module 2 (Simulation) |
| 8-10 | Module 3 (NVIDIA Isaac) |
| 11-12 | Module 4 intro + Humanoid Development |
| 13 | Module 4 Capstone + Conversational Robotics |

## Bonus Features Requirements

### RAG Chatbot (Core - 100 points)

- MUST answer questions about book content
- MUST support user-selected text context (highlight-to-ask)
- MUST use Qdrant Cloud for vector storage
- MUST use OpenAI Agents/ChatKit SDK
- MUST be embedded within the Docusaurus site

### Reusable Intelligence (+50 points)

- MUST create Claude Code Subagents for repetitive tasks
- MUST define Agent Skills for book authoring workflows
- MUST document subagent/skill usage in specs

### Authentication & Personalization (+50 points each)

- MUST implement Better-Auth signup/signin
- MUST capture user software/hardware background at signup
- MUST implement "Personalize" button at chapter start
- Personalization MUST adjust content based on stored profile

### Translation (+50 points)

- MUST implement "Urdu" translation button at chapter start
- Translation MUST maintain technical term accuracy
- Original content MUST remain accessible

## Development Workflow

### Spec-Driven Process

1. **Specify** (`/sp.specify`): Define feature requirements
2. **Clarify** (`/sp.clarify`): Resolve ambiguities
3. **Plan** (`/sp.plan`): Create implementation architecture
4. **Tasks** (`/sp.tasks`): Generate actionable task list
5. **Implement** (`/sp.implement`): Execute tasks
6. **Commit** (`/sp.git.commit_pr`): Version and submit

### Quality Gates

- All content MUST pass Docusaurus build
- All code examples MUST be syntax-validated
- RAG chatbot MUST correctly answer 5 sample questions per module
- Personalization MUST demonstrably change content
- Translation MUST render correctly in RTL layout

### PHR Requirements

Every significant interaction MUST generate a Prompt History Record in:
- `history/prompts/constitution/` - Constitutional changes
- `history/prompts/<feature>/` - Feature-specific work
- `history/prompts/general/` - General interactions

## Governance

This constitution supersedes all other development practices for this hackathon project.

### Amendment Process

1. Amendments MUST be documented with rationale
2. Amendments MUST increment version appropriately:
   - MAJOR: Principle removal or fundamental redefinition
   - MINOR: New principle or significant expansion
   - PATCH: Clarifications and wording changes
3. All amendments MUST update the Sync Impact Report

### Compliance

- All PRs MUST verify constitution compliance
- Complexity beyond requirements MUST be justified
- Stack deviations MUST be documented in ADRs
- Timeline: Submission deadline Sunday, Nov 30, 2025 at 6:00 PM

### ADR Triggers

Create an ADR (`/sp.adr`) when:
- Choosing between competing technical approaches
- Deviating from the mandated stack
- Making irreversible architectural decisions
- Implementing non-obvious performance optimizations

**Version**: 1.0.0 | **Ratified**: 2025-01-18 | **Last Amended**: 2025-01-18
