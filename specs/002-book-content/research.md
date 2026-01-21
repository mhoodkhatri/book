# Research: Professional Textbook Content Authoring

**Feature**: 002-book-content
**Date**: 2026-01-19
**Purpose**: Resolve NEEDS CLARIFICATION items and establish best practices for professional technical textbook content

---

## 1. Technical Writing Best Practices for Robotics Content

### Decision: O'Reilly/Manning Style Guide Alignment

**Rationale**: Industry-standard technical publishers have established conventions that readers expect. Following these patterns increases perceived quality and readability.

**Key Practices**:

1. **Active Voice Preference**: "The node publishes messages" not "Messages are published by the node"
2. **Second Person for Instructions**: "You will create a publisher node" when addressing the reader directly in exercises
3. **Third Person for Concepts**: "A node is an independent process" for conceptual explanations
4. **Present Tense for Facts**: "ROS 2 uses DDS for communication"
5. **Imperative for Commands**: "Run the following command:"

**Alternatives Considered**:
- Academic textbook style (rejected: too formal, slower pacing)
- Blog-style casual writing (rejected: lacks authority, inconsistent with spec requirements)

---

## 2. Code Example Standards for ROS 2 Humble

### Decision: Complete, Runnable Examples with Contextual Comments

**Rationale**: Spec FR-004, FR-005 require production-quality, copy-paste runnable code. Incomplete snippets frustrate learners.

**Standards**:

1. **Minimum 2 Inline Comments** per code block (SC-008)
2. **Full Import Statements**: Always show imports, even if repetitive
3. **Type Hints**: Include Python type hints for clarity
4. **Error Handling**: Show basic error handling where appropriate
5. **Expected Output**: Include expected terminal output in code comments or following text

**Code Block Format**:
```python
#!/usr/bin/env python3
"""Brief module docstring explaining purpose."""

import rclpy  # ROS 2 Python client library
from rclpy.node import Node
from std_msgs.msg import String  # Standard message type for strings

class MinimalPublisher(Node):
    """A minimal publisher node that demonstrates topic publication."""

    def __init__(self):
        super().__init__('minimal_publisher')
        # Create a publisher on 'topic' with queue size 10
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        # Timer callback fires every 0.5 seconds
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        # Log message for debugging
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Alternatives Considered**:
- Pseudocode snippets (rejected: spec explicitly requires runnable code)
- Abbreviated examples (rejected: confuses beginners, violates FR-005)

---

## 3. Chapter Structure for RAG Optimization

### Decision: Hierarchical Sections with 500-1000 Token Chunks

**Rationale**: Constitution Principle III requires RAG-optimized content. Sections must be semantically complete and appropriately sized for vector embedding.

**Section Guidelines**:

1. **H2 Headers for Major Sections**: "## What are Nodes?", "## Creating a Publisher"
2. **H3 Headers for Subsections**: "### Node Lifecycle", "### Publisher Configuration"
3. **Avoid Pronoun References Across Sections**: Each section should be readable in isolation
4. **Explicit Keyword Anchors**: Include key terms early in each section
5. **Target Length**: 500-1000 tokens per H2 section (approximately 375-750 words)

**Section Independence Test**: Can this section answer a question without reading previous sections? If not, add explicit context.

**Alternatives Considered**:
- Longer flowing sections (rejected: poor RAG retrieval, violates constitution)
- Shorter micro-sections (rejected: fragmented learning experience)

---

## 4. Exercise Design Patterns

### Decision: Scaffolded Exercises with Clear Checkpoints

**Rationale**: SC-009 requires independently completable exercises. Effective exercises build skills progressively.

**Exercise Structure**:

1. **Objective**: Clear statement of what will be accomplished
2. **Prerequisites**: Explicit list of required setup
3. **Steps**: Numbered, atomic instructions
4. **Checkpoints**: Verification points with expected output
5. **Stretch Goals**: Optional advanced challenges

**Exercise Template**:
```markdown
### Exercise: Create Your First Publisher

**Objective**: Create a ROS 2 node that publishes string messages to a topic.

**Prerequisites**:
- ROS 2 Humble installed and sourced
- Python 3.10+ available
- Workspace created at `~/ros2_ws`

**Steps**:

1. Create a new Python package:
   ```bash
   cd ~/ros2_ws/src
   ros2 pkg create --build-type ament_python my_publisher
   ```

2. Create the publisher node file...

**Checkpoint 1**: After step 2, verify the file exists:
```bash
ls ~/ros2_ws/src/my_publisher/my_publisher/
# Expected: __init__.py  publisher_node.py
```

**Stretch Goal**: Modify the publisher to accept a custom message string as a command-line argument.
```

**Alternatives Considered**:
- Open-ended exercises (rejected: too ambiguous for beginners)
- Copy-paste only exercises (rejected: doesn't build understanding)

---

## 5. Terminology Consistency Strategy

### Decision: Maintain Central Glossary and Use Consistent Terms

**Rationale**: SC-006 requires consistent terminology. Inconsistent terms confuse readers and break RAG retrieval.

**Core Terms (must be used consistently)**:

| Term | Definition | Never Use Instead |
|------|------------|-------------------|
| node | An independent ROS 2 process | process, module, component |
| topic | Named bus for publish-subscribe messages | channel, stream, queue |
| publisher | Node that sends messages to a topic | sender, producer, emitter |
| subscriber | Node that receives messages from a topic | receiver, consumer, listener |
| message | Data structure exchanged via topics | packet, payload, data |
| service | Request-response communication pattern | RPC, API call |
| action | Long-running task with feedback | job, task (alone) |
| callback | Function invoked on events | handler (use sparingly) |
| workspace | Directory structure for ROS 2 packages | project, folder |
| package | Unit of ROS 2 software distribution | module, library |

**Enforcement**: Search-and-replace review before publication.

**Alternatives Considered**:
- Inline glossary definitions (rejected: clutters text, use once-per-chapter)
- Allowing synonyms (rejected: violates consistency requirement)

---

## 6. NVIDIA Isaac Content Approach

### Decision: GPU Requirements Clearly Stated, CPU Alternatives Where Feasible

**Rationale**: Spec assumption #3 acknowledges not all readers have NVIDIA GPUs. Balance between demonstrating full capability and accessibility.

**Approach**:

1. **Clear Hardware Requirements**: State GPU requirements at module and chapter start
2. **Isaac Sim**: Requires NVIDIA GPU, no CPU alternative (state clearly)
3. **Isaac ROS**: Some packages have CPU fallbacks, document both paths
4. **VSLAM**: Can demonstrate with ORB-SLAM2 as CPU alternative
5. **Nav2**: CPU-capable, focus on standard deployment

**Hardware Requirements Box**:
```markdown
:::info Hardware Requirements
This chapter requires:
- NVIDIA GPU with CUDA 11.7+ support (RTX 2070 or better recommended)
- Ubuntu 22.04 LTS
- 32GB RAM minimum
- 50GB free disk space

**No GPU?** See alternative approaches in the [CPU Alternatives](#cpu-alternatives) section.
:::
```

**Alternatives Considered**:
- Omit GPU requirements silently (rejected: frustrates readers without hardware)
- Provide only CPU content (rejected: misses key Isaac capabilities)

---

## 7. Module 4 VLA Content Depth

### Decision: Practical Integration Focus, Not Research Paper Depth

**Rationale**: Module 4 covers cutting-edge topics (Whisper, LLM planning). Focus on practical integration rather than theoretical depth.

**Content Approach**:

1. **Whisper**: Focus on integration with ROS 2, not ASR theory
2. **Cognitive Planning**: Show LangChain/LlamaIndex integration patterns, not LLM internals
3. **Capstone**: Integration project demonstrating end-to-end pipeline

**Depth Calibration**:
- Assume reader understands "what an LLM is"
- Focus on "how to connect LLM to robot action"
- Provide working code over theoretical explanation

**Alternatives Considered**:
- Deep theoretical coverage (rejected: out of scope, better resources exist)
- Trivial examples (rejected: doesn't demonstrate real capability)

---

## 8. Content Length Targets

### Decision: 2000-4000 Words Per Chapter

**Rationale**: SC-003 targets 60-90 minutes per chapter. At ~200 words/minute reading speed with code work, 2000-4000 words is appropriate.

**Breakdown**:
- Chapter Overview: 150-200 words
- Learning Objectives: 100-150 words
- Conceptual Explanation: 800-1500 words
- Practical Example: 600-1000 words (including code)
- Exercise: 300-500 words
- Summary: 100-200 words

**Alternatives Considered**:
- Shorter chapters (rejected: insufficient depth)
- Longer chapters (rejected: fatigue, exceeds time target)

---

## Summary

All NEEDS CLARIFICATION items have been resolved:

| Item | Resolution |
|------|------------|
| Writing style | O'Reilly/Manning technical style |
| Code standards | Complete, runnable, 2+ comments |
| RAG optimization | 500-1000 token sections, H2/H3 hierarchy |
| Exercise design | Scaffolded with checkpoints |
| Terminology | Central glossary, strict consistency |
| GPU requirements | Clear statements, alternatives where feasible |
| VLA depth | Practical integration focus |
| Content length | 2000-4000 words per chapter |

**Next Step**: Phase 1 - Create data-model.md and contracts/
