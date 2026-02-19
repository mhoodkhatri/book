---
id: "0007"
title: Generate deployment tasks for 007-deploy-all-features
stage: tasks
date: 2026-02-18
surface: agent
model: claude-sonnet-4-6
feature: 007-deploy-all-features
branch: 007-deploy-all-features
user: mhoodkhatri
command: /sp.tasks
labels: ["tasks","deployment","railway","qdrant","cors","bearer-auth","github-actions"]
links:
  spec: specs/007-deploy-all-features/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/007-deploy-all-features/tasks.md
tests:
  - "none"
---

## Prompt

(empty — /sp.tasks invoked with no additional arguments)

## Response snapshot

Generated `specs/007-deploy-all-features/tasks.md` with 44 tasks across 10 phases:

- **Phase 1** (Setup — 10 tasks, all parallel): Source code changes for production — CORS wildcard fix in `backend/src/config.py` and `backend/src/main.py`, Bearer token fallback in `backend/src/middleware/auth.py`, Procfile creation for both Railway services, GitHub Actions deploy.yml update, Bearer token headers in `ChapterChat/index.tsx` and `TranslationButton.tsx`, and `.env.example` updates.
- **Phase 2** (Foundational — 7 sequential tasks): Neon DB verification, Qdrant Cloud provisioning, auth-service Railway deploy, FastAPI Railway deploy, GitHub Actions secrets + Docusaurus build, Qdrant indexing, health endpoint verification.
- **Phases 3–9** (User Stories 1–6 + 7, all individually checkpointed): Smoke tests and verification per acceptance scenario for each story.
- **Phase 10** (Polish — 4 tasks): CORS verification, security checklist, graceful degradation, final smoke test pass.

Total: 44 tasks. No test tasks (spec does not request TDD). Acceptance via smoke tests and quickstart.md checklist.

## Outcome

- ✅ Impact: Complete, immediately-executable task list for deploying all three services (Docusaurus on GitHub Pages, auth-service on Railway, FastAPI on Railway) with correct production configuration and cross-domain auth.
- 🧪 Tests: No automated test tasks — verification is manual smoke tests per quickstart.md.
- 📁 Files: specs/007-deploy-all-features/tasks.md (created)
- 🔁 Next prompts: `/sp.implement` to execute Phase 1 code changes; then follow Phase 2 deployment steps manually.
- 🧠 Reflection: The critical code changes (CORS wildcard removal, Bearer token auth) were already identified in research.md and contracts/cors-config.md — tasks.md translates them into precise, file-path-specific actions. US7 (text selection AI) was intentionally sequenced after US3 (chat) in Phase 6 since it depends on the same services being verified working first.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): none
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): Run /sp.implement on Phase 1 tasks to apply code changes before manual Railway deployment.
