# Implementation Plan: Better-Auth Authentication System

**Branch**: `004-better-auth` | **Date**: 2026-01-26 | **Spec**: [./spec.md](./spec.md)
**Input**: Feature specification from `/specs/004-better-auth/spec.md`

## Summary

Implement OAuth 2.0 authentication using Better-Auth library to gate RAG chatbot and translation features behind user authentication. The system will capture user background during signup (stored in Neon Postgres), manage secure sessions, and modify both backend (FastAPI) and frontend (React/Docusaurus) to enforce authentication checks.

## Technical Context

**Language/Version**: Python 3.10+ (backend), TypeScript 5.6.2 (frontend)
**Primary Dependencies**: FastAPI, Better-Auth (JS), PyJWT (Python), React 19.x, Docusaurus 3.9.2
**Storage**: Neon Serverless Postgres (users, profiles, sessions)
**Testing**: Manual testing (hackathon scope)
**Target Platform**: Web (GitHub Pages frontend, backend API server)
**Project Type**: Web (frontend + backend)
**Performance Goals**: Sign-in within 10 seconds, 100 concurrent sessions, session persistence 99.5%
**Constraints**: Free tier databases (Neon, Qdrant), HTTPS required for production session cookies
**Scale/Scope**: ~1,000 users, ~50 concurrent for hackathon demo

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| I. Docusaurus-First | PASS | Auth UI integrates via React components, no modification to MDX content |
| II. Spec-Driven Development | PASS | Following SDD workflow with spec -> plan -> tasks |
| III. RAG-First Content Design | N/A | Auth feature does not modify content structure |
| IV. Modular Content Architecture | N/A | Auth feature does not modify content modules |
| V. Code-Content Parity | N/A | Auth feature does not add code examples to book |
| VI. Accessibility-First Design | PASS | Auth UI will use semantic HTML, clear labels |
| VII. Security and Data Integrity | PASS | Better-Auth required per constitution, Neon Postgres mandated |

**Technical Stack Compliance**:
- Authentication: Better-Auth (constitution mandated)
- Database: Neon Serverless Postgres (constitution mandated)
- Backend: FastAPI (existing, constitution approved)
- Frontend: Docusaurus/React (existing, constitution approved)

## Project Structure

### Documentation (this feature)

```text
specs/004-better-auth/
├── plan.md              # This file
├── research.md          # Phase 0 output - authentication strategy research
├── data-model.md        # Phase 1 output - database schema design
├── quickstart.md        # Phase 1 output - development setup guide
├── contracts/           # Phase 1 output - API contracts
│   └── auth-api.yaml    # OpenAPI spec for auth endpoints
└── tasks.md             # Phase 2 output (via /sp.tasks)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── main.py                    # [MODIFY] Add auth router
│   ├── config.py                  # [MODIFY] Add Neon DB config
│   ├── api/
│   │   ├── chat.py                # [MODIFY] Add auth dependency
│   │   ├── translate.py           # [MODIFY] Add auth dependency
│   │   └── auth.py                # [NEW] Auth endpoints (proxy to Better-Auth or local JWT verify)
│   ├── models/
│   │   ├── auth.py                # [NEW] User, UserProfile, Session models
│   │   └── ...existing models
│   ├── services/
│   │   ├── auth.py                # [NEW] Auth service (JWT verification, session management)
│   │   ├── database.py            # [NEW] Neon Postgres connection
│   │   └── ...existing services
│   └── middleware/
│       └── auth.py                # [NEW] FastAPI auth dependency for route protection
└── requirements.txt               # [MODIFY] Add psycopg2-binary, PyJWT

src/
├── components/
│   ├── ChapterChat/
│   │   ├── index.tsx              # [MODIFY] Add auth check before chat
│   │   └── FloatingButton.tsx     # [MODIFY] Show auth prompt if not signed in
│   ├── TranslateButton/
│   │   └── index.tsx              # [MODIFY] Add auth check before translation
│   └── Auth/                      # [NEW] Auth UI components
│       ├── index.tsx              # Auth context provider
│       ├── SignUpForm.tsx         # Sign up with background capture
│       ├── SignInForm.tsx         # Sign in form
│       ├── UserMenu.tsx           # User menu dropdown (shows when signed in)
│       ├── AuthModal.tsx          # Modal wrapper for auth forms
│       └── styles.module.css      # Auth component styles
├── context/
│   └── AuthContext.tsx            # [NEW] React context for auth state
└── theme/
    └── DocItem/
        └── Layout/
            └── index.tsx          # [MODIFY] Add AuthProvider wrapper

package.json                       # [MODIFY] Add better-auth client
```

**Structure Decision**: Web application structure (existing frontend + backend) with new auth components integrated into Docusaurus theme via swizzled Layout and navbar customization. Auth state managed via React Context.

## Complexity Tracking

> No complexity violations. Implementation follows constitution requirements.

| Item | Justification |
|------|---------------|
| Better-Auth library | Constitution mandated (Section VII) |
| Neon Postgres | Constitution mandated (Technical Stack Constraints) |
| JWT verification in Python | Better-Auth is JS-native; Python backend requires JWT plugin + PyJWT verification |

## Architecture Decisions

### AD-001: Better-Auth with FastAPI Python Backend

**Decision**: Use Better-Auth as the authentication server (Node.js) and verify JWT tokens in FastAPI using PyJWT.

**Rationale**: Better-Auth is a JavaScript/TypeScript library without native Python support. Per constitution mandate, we must use Better-Auth. The recommended approach is:
1. Use Better-Auth's JWT plugin to issue signed JWTs
2. Verify JWTs in Python using PyJWT with the same secret
3. Store users/sessions in Neon Postgres (shared between Better-Auth and FastAPI)

**Alternatives Rejected**:
- Pure Python auth (e.g., FastAPI-Users): Violates constitution mandate for Better-Auth
- Better-Auth as separate microservice: Over-engineering for hackathon scope

### AD-002: Session Storage in Neon Postgres

**Decision**: Store all authentication data (users, profiles, sessions) in Neon Postgres.

**Rationale**: Constitution mandates Neon Postgres. Better-Auth supports PostgreSQL as a database adapter. This provides branch-compatible authentication data per Neon Auth's architecture.

### AD-003: Frontend Auth State via React Context

**Decision**: Manage auth state using React Context with localStorage persistence.

**Rationale**:
- Docusaurus is a static site generator; no server-side session management
- React Context provides component-level auth state access
- localStorage persists session token across page refreshes
- Better-Auth React client provides `useSession` hook compatible with this approach

## Integration Points

### Backend Protection (FastAPI)

```python
# src/middleware/auth.py
from fastapi import Depends, HTTPException, Header
from src.services.auth import verify_token

async def require_auth(authorization: str = Header(...)):
    """Dependency that requires valid JWT token."""
    if not authorization.startswith("Bearer "):
        raise HTTPException(status_code=401, detail="Missing bearer token")

    token = authorization.replace("Bearer ", "")
    user = verify_token(token)

    if not user:
        raise HTTPException(status_code=401, detail="Invalid or expired token")

    return user

# Usage in protected routes:
@router.post("/api/chat")
async def chat(request: ChatRequest, user = Depends(require_auth)):
    # Only authenticated users reach here
    ...
```

### Frontend Protection (React)

```typescript
// src/components/ChapterChat/index.tsx
import { useAuth } from '@site/src/context/AuthContext';

export function ChapterChat({ ... }) {
  const { isAuthenticated, showSignIn } = useAuth();

  if (!isAuthenticated) {
    return (
      <div className={styles.authPrompt}>
        <p>Please sign in first to use this feature.</p>
        <button onClick={showSignIn}>Sign In</button>
      </div>
    );
  }

  // Existing chat implementation
  ...
}
```

## Data Flow

### Sign Up Flow
```
User -> SignUpForm -> Better-Auth /signup -> Neon Postgres (insert user, profile)
                   -> JWT issued -> localStorage -> AuthContext updated
                   -> UI reflects signed-in state
```

### Protected API Call Flow
```
User clicks Chat -> Check isAuthenticated (Context)
                 -> If not auth: Show "Please sign in" message
                 -> If auth: Include JWT in Authorization header
                 -> FastAPI middleware verifies JWT
                 -> If valid: Process request, return response
                 -> If invalid: 401 -> Show "Session expired" + sign-in link
```

### Sign Out Flow
```
User clicks Sign Out -> Better-Auth /signout -> Clear localStorage
                     -> AuthContext updated -> UI reflects signed-out state
                     -> Protected features become inaccessible
```

## Environment Variables

### Backend (.env additions)
```
# Neon Postgres (new)
DATABASE_URL=postgresql://user:pass@ep-xxx.neon.tech/dbname?sslmode=require

# Auth (new)
BETTER_AUTH_SECRET=your-secret-key-min-32-chars
JWT_ALGORITHM=HS256
```

### Frontend (docusaurus.config.ts additions)
```typescript
customFields: {
  chatApiUrl: process.env.CHAT_API_URL || 'http://localhost:8000',
  authApiUrl: process.env.AUTH_API_URL || 'http://localhost:8000', // new
}
```

## Testing Strategy

### Manual Test Cases (Hackathon Scope)

1. **Sign Up Flow**
   - Navigate to site, click Sign Up
   - Fill email, password, select background
   - Submit -> Verify account created, auto-signed-in
   - Verify chat and translate features accessible

2. **Sign In Flow**
   - Sign out, click Sign In
   - Enter valid credentials -> Verify signed in
   - Enter invalid credentials -> Verify error message shown

3. **Protected Feature Access**
   - As unauthenticated user, click chatbot -> See "Please sign in" message
   - As unauthenticated user, click translate -> See "Please sign in" message
   - Sign in, retry -> Features work normally

4. **Session Persistence**
   - Sign in, refresh page -> Still signed in
   - Sign in, close tab, reopen -> Still signed in (within session expiry)

5. **Sign Out**
   - Click Sign Out -> Verify signed out
   - Verify chat and translate show "Please sign in"

## Risks and Mitigations

| Risk | Impact | Mitigation |
|------|--------|------------|
| Better-Auth + FastAPI integration complexity | High | Use JWT plugin approach, verify tokens in Python |
| Session cookie security on HTTP (dev) | Medium | Use JWT in Authorization header, not cookies |
| Neon Postgres cold starts | Low | Free tier has some latency; acceptable for hackathon |
| Better-Auth version compatibility | Medium | Pin to stable version (1.4.6 tested with Neon) |

## Phase Outputs Checklist

- [x] **plan.md** - This document
- [x] **research.md** - Better-Auth + FastAPI integration research
- [x] **data-model.md** - Database schema (users, profiles, sessions)
- [x] **contracts/auth-api.yaml** - OpenAPI spec for auth endpoints
- [x] **quickstart.md** - Local development setup guide
- [ ] **tasks.md** - Generated via /sp.tasks (Phase 2)
