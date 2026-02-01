# Implementation Plan: Better-Auth Authentication

**Branch**: `005-better-auth` | **Date**: 2026-01-30 | **Spec**: [specs/005-better-auth/spec.md](spec.md)
**Input**: Feature specification from `/specs/005-better-auth/spec.md`

## Summary

Implement a complete authentication system using Better-Auth (Node.js/Express) with email verification via Brevo, user background capture for personalization, and gated access to RAG chatbot, Translation, and Personalization features. The system uses Neon Postgres as the shared database between the Better-Auth service and the existing FastAPI backend, with cookie-based cross-origin session management.

## Technical Context

**Language/Version**: TypeScript 5.6 (auth service + frontend), Python 3.10+ (FastAPI backend)
**Primary Dependencies**: Better-Auth v1.4.x, Express, React 19, Docusaurus 3.9.2, FastAPI, pg (Node.js), asyncpg (Python)
**Storage**: Neon Serverless Postgres (shared between Better-Auth and FastAPI)
**Testing**: Manual verification checklist (hackathon scope), Playwright E2E (stretch)
**Target Platform**: Web — Docusaurus static site (port 3000), Express auth service (port 3005), FastAPI backend (port 8000)
**Project Type**: Web (3 services: frontend + auth service + API backend)
**Performance Goals**: Sign-in < 2s, session validation < 100ms, email delivery < 30s
**Constraints**: Brevo free tier (300 emails/day), Neon free tier, cross-origin cookie auth (SameSite=None)
**Scale/Scope**: Hackathon demo — <100 users, 3 gated features, 6 auth pages/views

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Evidence |
|-----------|--------|----------|
| I. Docusaurus-First | PASS | Auth UI pages in `src/pages/`, React components in `src/components/` |
| II. Spec-Driven | PASS | Full spec exists at `specs/005-better-auth/spec.md` |
| III. RAG-First Content | N/A | Auth feature does not modify content structure |
| IV. Modular Content | N/A | Auth feature does not modify content modules |
| V. Code-Content Parity | N/A | No code examples in textbook affected |
| VI. Accessibility-First | PASS | Auth pages will use semantic HTML, proper form labels, autocomplete attributes |
| VII. Security & Data | PASS | Better-Auth with secure sessions, Neon Postgres, env vars for secrets, audit logging |
| Tech Stack: Better-Auth | PASS | Constitution mandates Better-Auth |
| Tech Stack: Neon Postgres | PASS | Constitution mandates Neon |
| Tech Stack: FastAPI | PASS | Existing backend preserved, extended with session validation |
| Smallest viable change | PASS | Only adds auth; does not refactor existing RAG/Translation logic |

**Post-Phase 1 Re-check**: All gates still PASS. The 3-service architecture (frontend, auth, backend) is the minimum needed — Better-Auth requires Node.js runtime, FastAPI is Python, and they share Neon Postgres as specified in the clarifications.

## Project Structure

### Documentation (this feature)

```text
specs/005-better-auth/
├── plan.md              # This file
├── research.md          # Phase 0 output — technology research
├── data-model.md        # Phase 1 output — database schema
├── quickstart.md        # Phase 1 output — setup guide
├── contracts/
│   ├── auth-api.yaml    # Phase 1 output — Better-Auth + custom endpoints
│   └── fastapi-auth-middleware.yaml  # Phase 1 output — protected FastAPI endpoints
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

```text
# Auth Service (NEW — Better-Auth + Express)
auth-service/
├── package.json
├── tsconfig.json
├── .env                  # DATABASE_URL, BETTER_AUTH_SECRET, BREVO_API_KEY, etc.
├── src/
│   ├── index.ts          # Express server entry point (port 3005)
│   ├── auth.ts           # Better-Auth configuration (DB, email, session, user fields)
│   ├── routes/
│   │   ├── background.ts # POST /api/auth/custom/update-background
│   │   └── account.ts    # POST /api/auth/custom/delete-account
│   ├── services/
│   │   └── brevo.ts      # Brevo email service (verification, reset, retry logic)
│   ├── middleware/
│   │   └── lockout.ts    # Account lockout check (5 attempts → 15 min lock)
│   └── lib/
│       └── audit.ts      # Auth audit log writer

# FastAPI Backend (MODIFIED — add session validation)
backend/
├── src/
│   ├── main.py           # MODIFIED: add auth middleware
│   ├── middleware/
│   │   └── auth.py       # NEW: session validation dependency (reads Neon session table)
│   ├── api/
│   │   ├── chat.py       # MODIFIED: add auth dependency
│   │   └── translate.py  # MODIFIED: add auth dependency
│   └── services/
│       └── (unchanged)
├── requirements.txt      # MODIFIED: add asyncpg

# Docusaurus Frontend (MODIFIED — add auth UI and gating)
src/
├── lib/
│   └── auth-client.ts    # NEW: Better-Auth React client
├── contexts/
│   └── AuthContext.tsx    # NEW: React context for auth state
├── components/
│   ├── Auth/
│   │   ├── AuthForm.tsx       # NEW: Sign-in / Sign-up tabbed form
│   │   ├── PasswordStrength.tsx # NEW: Real-time password strength meter
│   │   ├── AuthGuard.tsx      # NEW: Wrapper that shows login prompt for gated features
│   │   └── AuthNavbar.tsx     # NEW: Navbar auth button (Sign In / User menu)
│   ├── Profile/
│   │   ├── BackgroundForm.tsx # NEW: Software/hardware background capture
│   │   └── AccountSettings.tsx # NEW: Account management (update bg, delete)
│   ├── ChapterChat/
│   │   ├── FloatingButton.tsx # MODIFIED: wrap with AuthGuard
│   │   └── (rest unchanged)
│   └── ChapterTranslation/
│       └── TranslationButton.tsx # MODIFIED: wrap with AuthGuard
├── pages/
│   ├── auth.tsx           # NEW: /auth page (sign-in/sign-up)
│   ├── profile/
│   │   ├── settings.tsx   # NEW: /profile/settings page
│   │   └── background.tsx # NEW: /profile/background page
│   └── index.tsx          # (unchanged)
├── theme/
│   └── DocItem/Layout/
│       └── index.tsx      # MODIFIED: wrap with AuthContext provider
├── css/
│   └── custom.css         # MODIFIED: add auth component styles
└── hooks/
    └── useAuthGuard.ts    # NEW: hook for auth check + redirect
```

**Structure Decision**: 3-service web architecture. Better-Auth requires Node.js (Express), existing backend is Python (FastAPI), frontend is React (Docusaurus). Services share Neon Postgres — no inter-service HTTP calls for session validation.

## Complexity Tracking

> No constitution violations. Architecture follows mandated stack exactly.

| Decision | Justification | Simpler Alternative Rejected |
|----------|---------------|------------------------------|
| 3 services (frontend, auth, backend) | Better-Auth = Node.js, FastAPI = Python — can't merge runtimes | Single-runtime: would require rewriting FastAPI in Node.js or auth in Python |
| Shared Neon DB (no inter-service HTTP) | Per spec clarification: FastAPI validates sessions by querying DB directly | Auth-service HTTP proxy: adds latency, coupling, and failure modes |
| Custom lockout logic | Better-Auth doesn't have built-in account lockout after N failures | Skip lockout: violates FR-006 |

## Implementation Phases

### Phase A: Auth Service Foundation (P1 — Stories 1 & 2)

**Goal**: Standalone Better-Auth Express service with signup, signin, email verification, and session management.

1. **Initialize auth-service project** — `package.json`, `tsconfig.json`, dependencies
2. **Configure Better-Auth** — Database (Neon), email/password, user additional fields, session settings
3. **Implement Brevo email service** — Verification and password reset email sending with retry logic
4. **Create Express server** — Mount Better-Auth handler, CORS, custom routes
5. **Implement account lockout** — Failed login counter, 15-minute lockout after 5 attempts
6. **Implement audit logging** — Write auth events to `auth_audit_log` table
7. **Run database migrations** — Generate and apply schema to Neon Postgres
8. **Manual verification** — Test signup → email → verify → signin → session → logout flow

### Phase B: Frontend Auth UI (P1 — Stories 1 & 2)

**Goal**: Auth pages, React context, and session management in Docusaurus.

1. **Create auth client** — `src/lib/auth-client.ts` with Better-Auth React client
2. **Create AuthContext** — `src/contexts/AuthContext.tsx` with `useSession`, cross-tab sync
3. **Create AuthForm component** — Tabbed sign-in/sign-up form with validation, password strength meter
4. **Create /auth page** — `src/pages/auth.tsx` using AuthForm
5. **Create AuthNavbar component** — Sign In button / User dropdown in navbar
6. **Integrate AuthContext** — Wrap DocItem layout with auth provider
7. **Add toast notifications** — Key action feedback (signed in, signed out, email sent)
8. **Add loading states** — Skeleton/loading during auth checks (FR-042)

### Phase C: Background Capture & Profile (P2 — Story 3)

**Goal**: User background form and account settings.

1. **Create BackgroundForm component** — Multi-select checkboxes, "Other" free-text, skip option
2. **Create /profile/background page** — Post-verification redirect target
3. **Create AccountSettings component** — Update background, change password
4. **Create /profile/settings page** — Account management
5. **Wire verification redirect** — After email verify → redirect to /profile/background

### Phase D: Feature Gating (P2 — Story 4)

**Goal**: Gate RAG chatbot and Translation behind authentication.

1. **Create AuthGuard component** — Renders login prompt for unauthenticated users
2. **Create useAuthGuard hook** — Auth check with redirect-back-after-login logic
3. **Gate ChapterChat** — Wrap FloatingButton with AuthGuard
4. **Gate TranslationButton** — Wrap with AuthGuard
5. **Add FastAPI auth middleware** — Session validation via Neon Postgres query
6. **Protect /api/chat endpoint** — Add auth dependency
7. **Protect /api/translate endpoint** — Add auth dependency
8. **Test graceful degradation** — Auth service down → content readable, gated features show "unavailable" (FR-043)

### Phase E: Password Reset & Account Deletion (P3 — Stories 5 & 6)

**Goal**: Complete auth lifecycle with password reset and account deletion.

1. **Configure password reset** — Better-Auth `sendResetPassword` with Brevo, 1-hour token expiry
2. **Create reset password UI** — Request form + set new password form
3. **Implement account deletion** — Password re-confirmation, soft-delete, anonymize, invalidate sessions
4. **Test token expiry** — Verify 24h verification and 1h reset expiry enforcement

### Phase F: Polish & Edge Cases

**Goal**: Handle all edge cases from spec.

1. **Email normalization** — Ensure lowercase before storage (FR-007)
2. **Resend cooldown** — 60-second timer with visible countdown (FR-015)
3. **Cross-tab sync** — Login/logout propagation via storage events (FR-011)
4. **Session expiry during RAG query** — Preserve input, show login, restore after auth (edge case)
5. **Auth service unavailable** — Graceful degradation for all gated features (FR-043)
6. **CORS hardening** — Restrict to exact Docusaurus domain (FR-034)

## Key Architectural Decisions

1. **Better-Auth as standalone Express service** (not embedded in FastAPI) — per spec clarification, separate runtimes sharing Neon DB.
2. **Direct DB session validation in FastAPI** (not HTTP call to auth service) — per spec clarification, avoids coupling and latency.
3. **Cookie-based auth with SameSite=None** — required for cross-origin auth between Docusaurus (port 3000) and auth service (port 3005).
4. **User background stored as additional fields on Better-Auth user table** — avoids separate table, single-query profile retrieval.
5. **Brevo REST API for emails** (not SMTP) — simpler integration, better error handling, no SMTP config needed.

## Risks & Mitigations

| Risk | Impact | Mitigation |
|------|--------|------------|
| Cross-origin cookies blocked by browser | Auth fails silently | Use `SameSite=None; Secure` — requires HTTPS in production. Local dev may need `localhost` exception. |
| Brevo free tier email limit (300/day) | Signup blocked after limit | Sufficient for hackathon; add rate limiting on frontend resend button |
| Better-Auth version incompatibility | API changes break integration | Pin to exact version in `package.json`; test after install |

## Follow-ups

- **OAuth providers** (Google, GitHub): Architecture supports future addition via Better-Auth plugins — out of scope for initial implementation.
- **Personalize button**: Background data is captured but the personalization feature itself is a separate spec.
- **Production deployment**: HTTPS, domain-based CORS, Brevo domain verification, environment-specific configs.
