---
id: "0004"
title: "Deploy All Features Implementation Plan"
stage: plan
date: 2026-02-18
surface: agent
model: claude-sonnet-4-6
feature: 007-deploy-all-features
branch: 007-deploy-all-features
user: mhoodkhatri
command: /sp.plan
labels: ["deployment", "railway", "qdrant-cloud", "cors", "github-pages", "github-actions"]
links:
  spec: specs/007-deploy-all-features/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/007-deploy-all-features/plan.md
  - specs/007-deploy-all-features/research.md
  - specs/007-deploy-all-features/data-model.md
  - specs/007-deploy-all-features/quickstart.md
  - specs/007-deploy-all-features/contracts/env-vars.md
  - specs/007-deploy-all-features/contracts/health-endpoints.md
  - specs/007-deploy-all-features/contracts/cors-config.md
tests:
  - "Health check verification: curl /health on all 3 services"
  - "CORS preflight test: OPTIONS requests with Origin header"
  - "Smoke test checklist in quickstart.md (6 user flows)"
---

## Prompt

/sp.plan

## Response snapshot

Generated full Phase 0 and Phase 1 implementation plan for `007-deploy-all-features` (Full Production Deployment — RAG, Translation, Auth).

**Phase 0 (Research)** resolved all NEEDS CLARIFICATION items:
- Deployment platform: Railway (Node.js + Python, persistent runtime, free tier, auto-SSL)
- Qdrant production: Qdrant Cloud (QDRANT_URL + QDRANT_API_KEY on Railway service)
- Cross-origin cookies: SameSite=None; Secure with explicit CORS origin allowlists
- GitHub Actions: Docusaurus build with CHAT_API_URL/AUTH_API_URL as repo secrets
- FastAPI CORS bug: Remove `+ ["*"]` wildcard; add CORS_ORIGINS env var

**Phase 1 (Design)** produced:
- `plan.md`: Full technical context, constitution check (all PASS), deployment topology diagram, ordered critical path
- `research.md`: 7 research items fully resolved with decisions + rationale
- `data-model.md`: Existing Neon Postgres schema + Qdrant vector schema documented for production verification
- `contracts/env-vars.md`: All env vars per service (auth-service, FastAPI backend, Docusaurus build)
- `contracts/health-endpoints.md`: Health check contracts + Railway config + smoke test script
- `contracts/cors-config.md`: CORS and cookie config with exact values + verification curl commands
- `quickstart.md`: 6-step deployment runbook (Qdrant Cloud → auth-service → FastAPI → indexing → GitHub Actions → smoke tests)

**Key code change identified**: `backend/src/main.py` has a wildcard CORS bug (`allow_origins=settings.cors_origins + ["*"]`) that is incompatible with `allow_credentials=True`. Fix tracked in tasks.

## Outcome

- ✅ Impact: All Phase 0 and Phase 1 planning artifacts complete. Ready for `/sp.tasks` to generate implementation task list.
- 🧪 Tests: No code tests run (planning phase). Verification checklists embedded in quickstart.md and data-model.md.
- 📁 Files: 7 files created in specs/007-deploy-all-features/
- 🔁 Next prompts: `/sp.tasks` to generate ordered task list; then implement deployment config changes
- 🧠 Reflection: The most critical finding is the FastAPI CORS wildcard bug (main.py:37) that will break cross-origin cookie auth in production. Must be fixed before deploying.

## Evaluation notes (flywheel)

- Failure modes observed: setup-plan.sh script failed (bash path issue on Windows), fell back to agent-native file reading
- Graders run and results (PASS/FAIL): Constitution check — all 11 gates PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Add CORS_ORIGINS env var to config.py and fix main.py wildcard as first implementation task
