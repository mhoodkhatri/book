# Implementation Plan: Better-Auth Authentication System

**Branch**: `004-better-auth` | **Date**: 2026-01-26 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/004-better-auth/spec.md`

## Summary

Implement Better-Auth authentication for the Physical AI Textbook platform, enabling Sign Up/Sign In/Sign Out with user background capture. Gate RAG chatbot and translation features behind authentication. Uses hybrid architecture: Better-Auth JWT plugin for frontend auth, FastAPI middleware for JWT verification, Neon Postgres for user data storage.

## Technical Context

**Language/Version**: TypeScript 5.6.2 (frontend), Python 3.10+ (backend)
**Primary Dependencies**: Better-Auth ^1.4.6, React 19.x, FastAPI ^0.109.0, PyJWT ^2.8.0, psycopg2-binary ^2.9.9
**Storage**: Neon Serverless Postgres (users, sessions, user_profile tables)
**Testing**: Manual E2E testing, curl for API testing (pytest for unit tests if time permits)
**Target Platform**: Web (Docusaurus 3.9.2 static site + FastAPI REST API)
**Project Type**: Web (frontend + backend)
**Performance Goals**: Sign-in <10s, auth state transitions <1s, 100 concurrent users
**Constraints**: Hackathon timeline, constitution mandate for Better-Auth, Neon free tier
**Scale/Scope**: ~100 users, 6 user stories, 19 functional requirements

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| I. Docusaurus-First | PASS | Auth UI integrates with Docusaurus via swizzled components |
| II. Spec-Driven Development | PASS | Full SDD workflow followed: spec → clarify → plan → tasks |
| III. RAG-First Content Design | N/A | Auth feature, not content |
| IV. Modular Content Architecture | N/A | Auth feature, not content |
| V. Code-Content Parity | N/A | Auth feature, not code examples |
| VI. Accessibility-First | PASS | Auth forms will use semantic HTML, labels |
| VII. Security and Data Integrity | PASS | Better-Auth mandated, Neon Postgres, env vars for secrets |

**Technical Stack Compliance**:

| Component | Required | Planned | Status |
|-----------|----------|---------|--------|
| Documentation | Docusaurus | Docusaurus 3.9.2 | PASS |
| Backend | FastAPI | FastAPI (existing) | PASS |
| Database | Neon Serverless Postgres | Neon Postgres | PASS |
| Authentication | Better-Auth | Better-Auth ^1.4.6 | PASS |
| Development | Claude Code + Spec-Kit Plus | Using SDD workflow | PASS |

**Gate Result**: PASS - All constitution requirements satisfied.

## Project Structure

### Documentation (this feature)

```text
specs/004-better-auth/
├── plan.md              # This file
├── spec.md              # Feature specification
├── research.md          # Research findings (complete)
├── data-model.md        # Database schema (complete)
├── quickstart.md        # Dev setup guide (complete)
├── contracts/
│   └── auth-api.yaml    # OpenAPI spec (complete)
└── tasks.md             # Implementation tasks (next: /sp.tasks)
```

### Source Code (repository root)

```text
# Frontend (Docusaurus/React)
src/
├── components/
│   ├── Auth/
│   │   ├── SignUpForm.tsx       # New: Signup with background selection
│   │   ├── SignInForm.tsx       # New: Signin form
│   │   ├── UserMenu.tsx         # New: User dropdown (signed-in state)
│   │   └── AuthModal.tsx        # New: Modal wrapper for auth forms
│   ├── ChapterChat/
│   │   └── index.tsx            # Modify: Add auth check
│   └── TranslateButton/
│       └── index.tsx            # Modify: Add auth check
├── context/
│   └── AuthContext.tsx          # New: Auth state management
├── lib/
│   └── auth-client.ts           # New: Better-Auth client config
├── theme/
│   ├── Root.tsx                 # New: Swizzle to add AuthProvider
│   └── DocItem/Layout/
│       └── index.tsx            # Modify: Inject auth-aware components
└── types/
    └── auth.ts                  # New: TypeScript auth types

# Backend (FastAPI/Python)
backend/
├── src/
│   ├── api/
│   │   ├── auth.py              # New: Auth endpoints (signup, signin, signout)
│   │   ├── chat.py              # Modify: Add auth dependency
│   │   └── translate.py         # Modify: Add auth dependency
│   ├── middleware/
│   │   └── auth.py              # New: JWT verification middleware
│   ├── models/
│   │   └── auth.py              # New: Auth Pydantic models
│   ├── services/
│   │   ├── auth.py              # New: Auth business logic (hash, verify)
│   │   └── database.py          # New: Neon Postgres connection
│   ├── config.py                # Modify: Add DATABASE_URL, JWT_SECRET
│   └── main.py                  # Modify: Add auth router
├── requirements.txt             # Modify: Add PyJWT, psycopg2-binary
└── .env                         # Modify: Add auth env vars
```

**Structure Decision**: Web application with frontend (Docusaurus/React) and backend (FastAPI/Python). Auth components integrate via React Context and swizzled Docusaurus theme. Backend auth via FastAPI router + dependency injection for protected endpoints.

## Architecture Overview

### Authentication Flow

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Docusaurus    │    │    FastAPI      │    │  Neon Postgres  │
│   (Frontend)    │    │   (Backend)     │    │   (Database)    │
└────────┬────────┘    └────────┬────────┘    └────────┬────────┘
         │                      │                      │
         │  1. Signup/Signin    │                      │
         │─────────────────────>│  2. Hash password    │
         │                      │  3. Create user      │
         │                      │─────────────────────>│
         │                      │<─────────────────────│
         │  4. Return JWT token │                      │
         │<─────────────────────│                      │
         │                      │                      │
         │  5. Store in localStorage                   │
         │                      │                      │
         │  6. API request +    │                      │
         │     Authorization    │                      │
         │     header           │                      │
         │─────────────────────>│  7. Verify JWT       │
         │                      │  8. Execute if valid │
         │  9. Response         │                      │
         │<─────────────────────│                      │
```

### Key Integration Points

1. **Frontend Auth State**: React Context (`AuthContext.tsx`) wraps app via swizzled `Root.tsx`
2. **Auth UI**: Modal-based signup/signin triggered from Navbar or feature prompts
3. **Protected Features**: Chat and Translate components check `useAuth()` before API calls
4. **Backend Middleware**: `Depends(get_current_user)` on protected endpoints
5. **Token Storage**: localStorage (`auth_token`) with 7-day expiry, refresh on activity

## Implementation Phases

### Phase 1: Backend Auth Infrastructure
- Database connection service
- Auth models (Pydantic)
- Auth service (password hashing, JWT generation/verification)
- Auth middleware (FastAPI dependency)
- Auth API endpoints (signup, signin, signout, session)

### Phase 2: Frontend Auth Infrastructure
- TypeScript types
- Better-Auth client configuration
- AuthContext provider
- Swizzle Root component

### Phase 3: Auth UI Components
- SignUpForm with background selection
- SignInForm
- UserMenu (signed-in state)
- AuthModal wrapper

### Phase 4: Feature Gating
- Add auth checks to ChapterChat component
- Add auth checks to TranslateButton component
- Add auth dependency to /api/chat endpoint
- Add auth dependency to /api/translate endpoint
- "Please sign in" prompts with modal trigger

### Phase 5: Integration & Testing
- E2E flow testing (signup → signin → use features → signout)
- Error handling verification
- Session persistence testing

## Complexity Tracking

> No constitution violations requiring justification. Implementation follows mandated stack.

| Decision | Rationale | Alternative Rejected |
|----------|-----------|---------------------|
| JWT in header (not cookies) | Works with CORS, static site compatible | Cookies require complex CORS config |
| React Context (not Redux) | Sufficient for auth state, simpler | Redux overkill for single global state |
| Inline auth in FastAPI (not Better-Auth server) | Single process, simpler deployment | Separate Node.js process adds complexity |

## Risk Analysis

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| Neon cold start latency | Medium | Low | Connection pooling, graceful degradation |
| JWT token leak (XSS) | Low | Medium | Short expiry, clear on signout |
| Password validation gaps | Low | Medium | Server-side validation always enforced |

## Dependencies

### External Dependencies
- Neon Postgres account (free tier)
- Environment variables configured

### Internal Dependencies
- Existing ChapterChat component
- Existing TranslateButton component
- Existing FastAPI backend structure

## Success Metrics

From spec success criteria:
- SC-001: Signup flow <2 minutes ✓ (simple form)
- SC-002: Sign-in <10 seconds ✓ (JWT return)
- SC-003: 100% RAG requests blocked when unauthenticated ✓ (middleware)
- SC-004: 100% translation requests blocked when unauthenticated ✓ (middleware)
- SC-005: Session persistence across refresh ✓ (localStorage + validation)
- SC-006: Immediate feature lockout on signout ✓ (context update)

## Next Steps

1. Run `/sp.tasks` to generate implementation task list
2. Execute tasks in dependency order
3. Run `/sp.git.commit_pr` when complete
