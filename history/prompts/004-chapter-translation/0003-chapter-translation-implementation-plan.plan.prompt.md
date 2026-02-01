---
id: "0003"
title: "Chapter Translation Implementation Plan"
stage: plan
date: 2026-01-29
surface: agent
model: claude-opus-4-5-20251101
feature: 004-chapter-translation
branch: 004-chapter-translation
user: architect
command: /sp.plan
labels: ["translation", "planning", "architecture", "implementation"]
links:
  spec: specs/004-chapter-translation/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/004-chapter-translation/plan.md
  - CLAUDE.md
tests:
  - null
---

## Prompt

/sp.plan

(User invoked the sp.plan command to execute the implementation planning workflow for the chapter translation feature.)

## Response snapshot

Generated comprehensive implementation plan for Chapter Translation Toggle feature:

**Summary**: Implement a Translate button that toggles book chapter content between English and Urdu in-place using Groq LLM (Llama 3.3 70B), with localStorage caching for instant toggle-back.

**Key Technical Decisions**:
1. Frontend: React 19 + TypeScript 5.6 + Docusaurus 3.9.2
2. Backend: FastAPI + Python 3.10+ + BeautifulSoup4 for HTML parsing
3. LLM: Existing Groq service (Llama 3.3 70B) - no new integration needed
4. Caching: localStorage with 7-day expiry, keyed by chapterId
5. RTL: CSS-based with Noto Nastaliq Urdu font

**Architecture**:
- TranslationButton component integrated via DocItem/Layout theme swizzle
- POST /api/translate endpoint with code block preservation
- Cache-first pattern: check localStorage before API call

**Constitution Check**: All 7 principles PASSED
- Docusaurus-First: Translation at DOM level, MDX unchanged
- Spec-Driven: Full spec, plan, research, data-model, contracts complete
- RAG-First: RAG operates on English source, unaffected
- Code-Content Parity: Code blocks preserved in English

**Artifacts Generated**:
- plan.md (complete)
- research.md (complete)
- data-model.md (complete)
- contracts/translate-api.yaml (complete)
- quickstart.md (complete)
- CLAUDE.md updated with new technologies

## Outcome

- Impact: High - Complete implementation plan ready for task generation
- Tests: N/A - Planning phase
- Files: 2 files modified (plan.md created, CLAUDE.md updated)
- Next prompts: /sp.tasks to generate implementation tasks
- Reflection: Phase 0 and Phase 1 artifacts were already complete from previous session; plan.md filled with architecture decisions, component diagrams, and data flow

## Evaluation notes (flywheel)

- Failure modes observed: None - all phases completed successfully
- Graders run and results (PASS/FAIL): Constitution check PASS (7/7 principles)
- Prompt variant (if applicable): Standard /sp.plan workflow
- Next experiment (smallest change to try): N/A - proceed to /sp.tasks
