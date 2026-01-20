---
id: "0007"
title: "Write Chapter Nodes and Topics"
stage: misc
date: 2026-01-19
surface: agent
model: claude-opus-4-5-20251101
feature: 002-book-content
branch: 002-book-content
user: system
command: /sp.write-chapter
labels: ["content", "chapter", "ros2", "nodes", "topics", "publisher", "subscriber"]
links:
  spec: specs/002-book-content/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - docs/module-1-ros2/nodes-topics.md
tests:
  - null
---

## Prompt

/sp.write-chapter docs/module-1-ros2/nodes-topics.md for Chapter 1.1 (Nodes and Topics)

User invoked the write-chapter skill to create comprehensive content for Chapter 1.1 covering ROS 2 nodes and topics fundamentals.

## Response snapshot

**REVISED** for beginner-friendliness at `docs/module-1-ros2/nodes-topics.md`:

**Content Statistics**:
- Word count: 1,836 words
- H2 sections: 14
- Code blocks: 10 (down from 20+)
- Docusaurus build: PASSED

**Beginner-Friendly Improvements**:
1. Added "Why Does a Robot Need Multiple Programs?" - explains motivation BEFORE concepts
2. Factory worker analogy - nodes as workers, topics as bulletin boards
3. Multiple ASCII diagrams showing data flow visually
4. "Option 1 (Bad) vs Option 2 (Good)" comparison for communication
5. Step-by-step data flow explanation with numbered steps
6. "When to Use Topics" section - helps beginners know the right tool
7. Mental Model Checklist at end - self-assessment questions
8. Reduced code to just 2 minimal snippets (publisher + subscriber key lines)

**Sections Written**:
1. Chapter Overview - Focus on mental model, not code
2. Learning Objectives - Conceptual understanding focused
3. Why Does a Robot Need Multiple Programs? - Motivation first
4. What Is a Node? - Factory worker analogy with diagram
5. How Do Nodes Communicate? - Bulletin board analogy
6. What Is a Topic? - Visual diagram, terminology table
7. How Data Flows Through Topics - Step-by-step trace
8. Topic Names and Message Types - Tables for reference
9. When to Use Topics - Guidance on right tool
10. Seeing Nodes and Topics in Action - CLI demo before code
11. Your First Publisher and Subscriber - Minimal code snippets
12. Summary - Key takeaways + mental model checklist

**Quality Compliance**:
- Concepts explained BEFORE code
- Real-world analogies throughout
- Visual diagrams for each concept
- Only 2 code snippets (minimal, essential)
- Professional but accessible tone

## Outcome

- ✅ Impact: Chapter 1.1 now teaches concepts first with analogies, making it accessible to beginners
- 🧪 Tests: Docusaurus build PASSED
- 📁 Files: 1 file modified (docs/module-1-ros2/nodes-topics.md) - REVISED version
- 🔁 Next prompts: /sp.write-chapter docs/module-1-ros2/services-actions.md (Chapter 1.2)
- 🧠 Reflection: User feedback led to major improvement - concepts before code is the right approach for beginners

## Evaluation notes (flywheel)

- Failure modes observed: Initial version had too much code, not enough conceptual grounding
- Graders run and results (PASS/FAIL): Docusaurus build PASS, beginner-friendly PASS
- Prompt variant (if applicable): Revised after user feedback requesting beginner-friendly approach
- Next experiment (smallest change to try): Apply same concept-first approach to all chapters
