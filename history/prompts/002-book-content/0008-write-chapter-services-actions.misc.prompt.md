---
id: "0008"
title: "Write Chapter Services and Actions"
stage: misc
date: 2026-01-19
surface: agent
model: claude-opus-4-5-20251101
feature: 002-book-content
branch: 002-book-content
user: system
command: /sp.write-chapter
labels: ["content", "chapter", "ros2", "services", "actions", "request-response"]
links:
  spec: specs/002-book-content/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - docs/module-1-ros2/services-actions.md
tests:
  - null
---

## Prompt

/sp.write-chapter docs/module-1-ros2/services-actions.md for Chapter 1.2 (Services and Actions)

User requested Chapter 1.2 following the same beginner-friendly approach established in Chapter 1.1.

## Response snapshot

Wrote complete Chapter 1.2 at `docs/module-1-ros2/services-actions.md`:

**Content Statistics**:
- Word count: 2,028 words
- H2 sections: 15
- Code blocks: 2 minimal snippets
- ASCII diagrams: 6
- Docusaurus build: PASSED

**Beginner-Friendly Approach**:
1. "Why Do We Need More Than Topics?" - establishes motivation first
2. Restaurant analogy for services (customer orders, kitchen serves)
3. Pizza delivery analogy for actions (goal, feedback, result)
4. Side-by-side comparison tables (Topic vs Service vs Action)
5. Decision flowchart for choosing the right pattern
6. Visual diagrams for request-response and goal-feedback-result flows
7. Action lifecycle state diagram
8. Mental Model Checklist for self-assessment
9. Only 2 code snippets (service server + client essentials)

**Sections Written**:
1. Chapter Overview - Motivation for new patterns
2. Learning Objectives - 5 conceptual outcomes
3. Why Do We Need More Than Topics? - Situations table
4. What Is a Service? - Restaurant analogy with diagram
5. Service vs Topic Comparison - Table
6. When to Use Services - Guidelines
7. Service Structure - Request/Response explanation
8. Service Roles - Client and Server
9. What Is an Action? - Pizza delivery analogy
10. Action vs Service Comparison - Table
11. Action Structure - Goal/Feedback/Result
12. Action Lifecycle - State diagram
13. When to Use Each Pattern - Decision flowchart + reference table
14. CLI Tools - ros2 service/action commands
15. Summary - Key takeaways + mental model checklist

## Outcome

- ✅ Impact: Chapter 1.2 teaches when and why to use services vs actions with clear analogies
- 🧪 Tests: Docusaurus build PASSED
- 📁 Files: 1 file modified (docs/module-1-ros2/services-actions.md)
- 🔁 Next prompts: /sp.write-chapter docs/module-1-ros2/rclpy.md (Chapter 1.3)
- 🧠 Reflection: Decision flowchart is particularly valuable for beginners choosing patterns

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Docusaurus build PASS, beginner-friendly PASS
- Prompt variant (if applicable): Beginner-friendly approach with analogies
- Next experiment (smallest change to try): Continue same approach for Chapter 1.3
