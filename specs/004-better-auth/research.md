# Research: Better-Auth Authentication Integration

**Feature**: 004-better-auth | **Date**: 2026-01-26 | **Status**: Complete

## Research Questions

### RQ-001: How to integrate Better-Auth with FastAPI Python backend?

**Context**: Better-Auth is a JavaScript/TypeScript library. Our backend is FastAPI (Python). Need to bridge authentication between these stacks.

**Findings**:

1. **Better-Auth JWT Plugin Approach** (Recommended)
   - Better-Auth provides a JWT plugin that issues signed JWTs
   - Python backend verifies these JWTs using PyJWT with shared secret
   - Database tables are shared via Neon Postgres

2. **Implementation Pattern**:
   ```
   Frontend (React) <-> Better-Auth (Node.js auth server)
                    \-> FastAPI (Python API) verifies JWT
   ```

3. **Token Verification in Python**:
   ```python
   import jwt
   from datetime import datetime

   def verify_token(token: str, secret: str) -> dict | None:
       try:
           payload = jwt.decode(token, secret, algorithms=["HS256"])
           if payload["exp"] < datetime.utcnow().timestamp():
               return None
           return payload
       except jwt.InvalidTokenError:
           return None
   ```

**Decision**: Use Better-Auth with JWT plugin. Frontend handles auth via Better-Auth client. Backend verifies JWTs in FastAPI middleware.

**Sources**:
- [Better Auth with Different Backend (FastAPI)](https://www.answeroverflow.com/m/1404248316824518656)
- [FastAPI JWT Authentication](https://fastapi.tiangolo.com/tutorial/security/oauth2-jwt/)

---

### RQ-002: Better-Auth React client setup for Docusaurus

**Context**: Need to integrate Better-Auth client into a Docusaurus 3.x site using React 19.

**Findings**:

1. **Client Installation**:
   ```bash
   npm install better-auth
   ```

2. **Client Setup**:
   ```typescript
   // src/lib/auth-client.ts
   import { createAuthClient } from "better-auth/react";

   export const authClient = createAuthClient({
     baseURL: "http://localhost:8000/api/auth"
   });
   ```

3. **React Hooks Available**:
   - `authClient.useSession()` - Returns session state (user, loading, error)
   - `authClient.signUp.email()` - Email/password signup
   - `authClient.signIn.email()` - Email/password signin
   - `authClient.signOut()` - Sign out current user

4. **Session State Pattern**:
   ```typescript
   const { data: session, isPending, error } = authClient.useSession();

   if (isPending) return <Loading />;
   if (!session) return <SignInPrompt />;
   return <AuthenticatedContent user={session.user} />;
   ```

**Decision**: Use Better-Auth React client with `createAuthClient`. Wrap Docusaurus in custom AuthProvider using React Context.

**Sources**:
- [Better Auth Client Docs](https://www.better-auth.com/docs/concepts/client)
- [Better Auth React Integration](https://better-auth-ui.com/integrations/react)

---

### RQ-003: Neon Postgres schema for Better-Auth

**Context**: Better-Auth requires specific database tables. Need to understand schema requirements for Neon Postgres.

**Findings**:

1. **Better-Auth Core Tables** (auto-created):
   - `user` - User accounts
   - `session` - Active sessions
   - `account` - OAuth provider accounts (optional)
   - `verification` - Email verification tokens (optional)

2. **Custom Extension for User Background**:
   ```sql
   -- Better-Auth user table (auto-created)
   CREATE TABLE "user" (
     id TEXT PRIMARY KEY,
     email TEXT UNIQUE NOT NULL,
     email_verified BOOLEAN DEFAULT FALSE,
     name TEXT,
     image TEXT,
     created_at TIMESTAMP DEFAULT NOW(),
     updated_at TIMESTAMP DEFAULT NOW()
   );

   -- Custom: User profile with background (our addition)
   CREATE TABLE user_profile (
     id TEXT PRIMARY KEY,
     user_id TEXT REFERENCES "user"(id) ON DELETE CASCADE,
     background_type TEXT NOT NULL,  -- 'beginner_robotics' | 'experienced_programmer' | 'ai_ml_background' | 'hardware_electronics'
     created_at TIMESTAMP DEFAULT NOW(),
     updated_at TIMESTAMP DEFAULT NOW()
   );

   -- Better-Auth session table (auto-created)
   CREATE TABLE session (
     id TEXT PRIMARY KEY,
     user_id TEXT REFERENCES "user"(id) ON DELETE CASCADE,
     token TEXT UNIQUE NOT NULL,
     expires_at TIMESTAMP NOT NULL,
     ip_address TEXT,
     user_agent TEXT,
     created_at TIMESTAMP DEFAULT NOW(),
     updated_at TIMESTAMP DEFAULT NOW()
   );
   ```

3. **Background Type Enum Values** (from spec FR-003):
   - `beginner_robotics` - "Beginner in robotics"
   - `experienced_programmer` - "Experienced programmer"
   - `ai_ml_background` - "AI/ML background"
   - `hardware_electronics` - "Hardware/electronics background"

**Decision**: Use Better-Auth's auto-created tables plus custom `user_profile` table for background data.

**Sources**:
- [Neon Auth with Better Auth](https://neon.tech/docs/guides/neon-auth)
- [Better Auth Database Setup](https://www.better-auth.com/docs/installation)

---

### RQ-004: Simplified auth approach for hackathon

**Context**: Full Better-Auth server requires Node.js runtime. For hackathon simplicity, explore alternatives.

**Findings**:

1. **Option A: Full Better-Auth Server** (Recommended for compliance)
   - Requires Node.js process alongside Python backend
   - Better-Auth handles all auth logic
   - FastAPI only verifies tokens
   - Pros: Full Better-Auth feature set, constitution compliant
   - Cons: Two backend processes

2. **Option B: Better-Auth-Compatible Python Implementation**
   - Implement Better-Auth-compatible endpoints in FastAPI
   - Use same database schema
   - Use PyJWT for token generation
   - Pros: Single backend process
   - Cons: Not "using" Better-Auth per se, may not satisfy constitution

3. **Option C: Hybrid Approach** (Selected)
   - Install Better-Auth as npm dependency
   - Create minimal Node.js auth server (auth-server.js)
   - Run via `concurrently` with FastAPI
   - FastAPI focuses on API, Node.js handles auth only
   - Pros: Constitution compliant, clear separation
   - Cons: Slight complexity

**Decision**: Option C - Hybrid approach. Minimal Node.js Better-Auth server for auth endpoints, FastAPI for protected APIs. Both share Neon Postgres.

---

### RQ-005: Auth header vs cookie-based sessions

**Context**: Need to pass authentication state from frontend to FastAPI backend.

**Findings**:

1. **Cookie-Based Sessions**:
   - Better-Auth default: HTTP-only session cookies
   - Pros: Automatic inclusion in requests, XSS-resistant
   - Cons: CORS complexity, requires HTTPS for Secure flag, doesn't work well with static site + separate API

2. **JWT in Authorization Header** (Recommended for our architecture):
   - Use Better-Auth JWT plugin
   - Store JWT in localStorage (or memory for extra security)
   - Include `Authorization: Bearer <token>` in API requests
   - Pros: Works with CORS, simple implementation, static site compatible
   - Cons: XSS risk if localStorage used

3. **Mitigation for localStorage JWT**:
   - Short token expiry (e.g., 1 hour)
   - Refresh token rotation
   - Clear on sign-out

**Decision**: Use JWT in Authorization header. Store in localStorage for persistence. Accept XSS tradeoff given hackathon scope.

---

### RQ-006: Frontend auth state management in Docusaurus

**Context**: Docusaurus is a static site generator. Need to manage auth state across page navigations.

**Findings**:

1. **React Context + localStorage Pattern**:
   ```typescript
   // AuthContext.tsx
   const AuthContext = createContext<AuthState | null>(null);

   export function AuthProvider({ children }) {
     const [user, setUser] = useState<User | null>(null);
     const [isLoading, setIsLoading] = useState(true);

     useEffect(() => {
       // Check localStorage on mount
       const token = localStorage.getItem('auth_token');
       if (token) {
         validateToken(token).then(setUser).finally(() => setIsLoading(false));
       } else {
         setIsLoading(false);
       }
     }, []);

     return (
       <AuthContext.Provider value={{ user, isLoading, signIn, signOut }}>
         {children}
       </AuthContext.Provider>
     );
   }
   ```

2. **Docusaurus Root Component Swizzle**:
   - Swizzle `Root` component to wrap app in AuthProvider
   - Or use `clientModules` in docusaurus.config.ts

3. **Auth State Flow**:
   ```
   App Mount -> Check localStorage -> Validate token with API
            -> If valid: Set user state
            -> If invalid: Clear storage, show signed-out UI
   ```

**Decision**: Use React Context with localStorage. Swizzle Docusaurus Root component to inject AuthProvider.

---

## Summary of Decisions

| Research Question | Decision |
|-------------------|----------|
| RQ-001: Better-Auth + FastAPI | JWT plugin approach, verify in Python |
| RQ-002: React client setup | Better-Auth React client with createAuthClient |
| RQ-003: Neon Postgres schema | Better-Auth tables + custom user_profile |
| RQ-004: Simplified approach | Hybrid: Node.js auth server + FastAPI API |
| RQ-005: Auth header vs cookie | JWT in Authorization header |
| RQ-006: Frontend state | React Context + localStorage |

## Technology Versions

| Technology | Version | Notes |
|------------|---------|-------|
| Better-Auth | ^1.4.6 | Tested with Neon Auth |
| PyJWT | ^2.8.0 | Python JWT verification |
| psycopg2-binary | ^2.9.9 | Neon Postgres Python driver |
| React | ^19.0.0 | Existing frontend |
| FastAPI | ^0.109.0 | Existing backend |

## Open Questions (None)

All research questions resolved. Ready for Phase 1: Design & Contracts.
