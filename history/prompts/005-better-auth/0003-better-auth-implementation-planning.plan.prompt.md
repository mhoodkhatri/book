---
id: "0003"
title: "Better-Auth implementation planning"
stage: plan
date: 2026-01-30
surface: agent
model: claude-opus-4-5-20251101
feature: 005-better-auth
branch: 005-better-auth
user: mhoodkhatri
command: /sp.plan
labels: ["authentication", "better-auth", "planning", "architecture"]
links:
  spec: specs/005-better-auth/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/005-better-auth/plan.md
  - specs/005-better-auth/research.md
  - specs/005-better-auth/data-model.md
  - specs/005-better-auth/quickstart.md
  - specs/005-better-auth/contracts/auth-api.yaml
  - specs/005-better-auth/contracts/fastapi-auth-middleware.yaml
tests:
  - N/A (planning phase — no code written)
---

## Prompt

based upon the last response perform the plan

## Response snapshot

Executed `/sp.plan` for the 005-better-auth feature. Completed Phase 0 (research) and Phase 1 (design & contracts):

**Phase 0 — Research**: Investigated Better-Auth Express integration, Neon Postgres adapter (pg Pool), Brevo email verification, React client setup, user additional fields for background capture, session management configuration, FastAPI session validation strategy, and account lockout implementation. All 8 research questions resolved in `research.md`.

**Phase 1 — Design**: Produced data model (5 tables: user with additional fields, session, account, verification, auth_audit_log), API contracts (OpenAPI for auth service + FastAPI middleware), and quickstart guide. Architecture: 3-service web app (Docusaurus frontend, Better-Auth Express service, FastAPI backend) sharing Neon Postgres.

**Plan**: 6 implementation phases (A-F) covering auth service foundation, frontend auth UI, background capture, feature gating, password reset/deletion, and edge cases. All constitution gates PASS.

## Outcome

- Impact: Complete implementation plan with 6 phases, research findings, data model, API contracts, and quickstart guide
- Tests: N/A (planning phase)
- Files: 6 artifacts created in specs/005-better-auth/
- Next prompts: `/sp.tasks` to generate actionable task list from this plan
- Reflection: Better-Auth's provider-agnostic email hooks make Brevo integration straightforward. The shared-DB pattern for FastAPI session validation avoids inter-service coupling.

## Evaluation notes (flywheel)

- Failure modes observed: None — all research questions resolved, all constitution gates passed
- Graders run and results (PASS/FAIL): Constitution check PASS (all 11 gates)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Run /sp.tasks to convert phases into dependency-ordered tasks
