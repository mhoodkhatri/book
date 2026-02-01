---
id: "0002"
title: "Spec Clarification Better Auth"
stage: spec
date: 2026-01-30
surface: agent
model: claude-opus-4-5-20251101
feature: 005-better-auth
branch: 005-better-auth
user: mhoodkhatri
command: /sp.clarify
labels: ["clarification", "authentication", "better-auth", "spec"]
links:
  spec: specs/005-better-auth/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/005-better-auth/spec.md
tests:
  - none
---

## Prompt

/sp.clarify

## Response snapshot

Ran structured ambiguity scan on the 005-better-auth spec. Identified 5 high-impact clarification areas and resolved all via sequential Q&A:

1. **Better-Auth ↔ FastAPI integration**: Better-Auth as standalone Node.js/Express API; FastAPI validates sessions via shared Neon Postgres DB (decoupled runtimes).
2. **Auth UI location**: Dedicated Docusaurus pages via `src/pages/` (e.g., `/auth`, `/profile/settings`, `/profile/background`).
3. **Email provider**: Brevo (formerly Sendinblue) — 300 emails/day free tier, personal sender email verification.
4. **Deployment topology**: Separate origin (different port/subdomain) with CORS configured for Docusaurus domain.
5. **Background form format**: Multi-select checkboxes from predefined lists + optional "Other" free-text field.

All clarifications integrated into spec sections: Clarifications, Assumptions, Functional Requirements (FR-013, FR-018, FR-020, FR-032, FR-034, FR-036), Key Entities.

## Outcome

- ✅ Impact: 5 architectural ambiguities resolved; spec ready for `/sp.plan`
- 🧪 Tests: none (spec-only changes)
- 📁 Files: specs/005-better-auth/spec.md (updated)
- 🔁 Next prompts: /sp.plan to generate implementation plan
- 🧠 Reflection: All critical integration and deployment questions resolved before planning phase, reducing downstream rework risk

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): N/A (clarification workflow)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
