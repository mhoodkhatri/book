# Research: Better-Auth Authentication

**Feature**: 005-better-auth | **Date**: 2026-01-30 | **Branch**: `005-better-auth`

## Research Questions & Findings

### RQ-1: How does Better-Auth integrate with Express.js?

**Decision**: Mount Better-Auth via `toNodeHandler` on a catch-all Express route.

**Rationale**: Better-Auth provides first-class Express support via `better-auth/node`. The handler processes all auth routes (`/api/auth/*`) automatically — signup, signin, verification, password reset, session management — without manual route definitions.

**Key Details**:
- Install: `npm i better-auth`
- Mount: `app.all("/api/auth/*", toNodeHandler(auth))` (Express v4 syntax)
- **Critical**: Do NOT apply `express.json()` before the Better-Auth handler — it breaks the client API. Apply it only to non-auth routes.
- Session retrieval: `auth.api.getSession({ headers: fromNodeHeaders(req.headers) })`
- CORS with `credentials: true` required for cross-origin cookie auth.

**Alternatives Considered**:
- Neon Auth (managed Better-Auth): Not chosen — doesn't support custom plugins or server-side handlers yet; limited to Better-Auth v1.4.6.
- Auth.js: Not chosen — constitution mandates Better-Auth specifically.

**Sources**: [Express Integration](https://www.better-auth.com/docs/integrations/express), [Installation](https://www.better-auth.com/docs/installation)

---

### RQ-2: How does Better-Auth connect to Neon Postgres?

**Decision**: Use `pg` Pool adapter with Neon connection string directly. No ORM.

**Rationale**: Simplest integration path. Better-Auth accepts a `pg.Pool` instance directly — no need for Drizzle or Prisma unless already in the stack. Neon Postgres is standard PostgreSQL, so the `pg` driver works out of the box. Schema is auto-generated via Better-Auth CLI.

**Key Details**:
```ts
import { Pool } from "pg";
export const auth = betterAuth({
  database: new Pool({
    connectionString: process.env.DATABASE_URL,
  }),
});
```
- CLI: `npx @better-auth/cli generate` → outputs SQL schema
- CLI: `npx @better-auth/cli migrate` → applies migrations
- Tables auto-created: `user`, `session`, `account`, `verification`

**Alternatives Considered**:
- Drizzle ORM adapter: Not chosen — adds unnecessary dependency for this project. Direct `pg` Pool is simpler and sufficient.
- Neon serverless driver (`@neondatabase/serverless`): Could be used if deploying to edge/serverless; not needed for Express on Node.js runtime.

**Sources**: [PostgreSQL Adapter](https://www.better-auth.com/docs/adapters/postgresql), [Database Concepts](https://www.better-auth.com/docs/concepts/database)

---

### RQ-3: How does email verification work with Brevo?

**Decision**: Use Better-Auth's `sendVerificationEmail` callback with Brevo's Transactional Email API (`POST https://api.brevo.com/v3/smtp/email`).

**Rationale**: Better-Auth is email-provider-agnostic — it provides hooks (`sendVerificationEmail`, `sendResetPassword`) where you supply your own email-sending logic. Brevo's REST API is straightforward (single POST with API key header). Free tier: 300 emails/day — sufficient for hackathon.

**Key Details**:
- Brevo API: `POST https://api.brevo.com/v3/smtp/email`
- Auth header: `api-key: <BREVO_API_KEY>`
- Better-Auth config:
  - `emailVerification.sendOnSignUp: true` — auto-send on signup
  - `emailVerification.autoSignInAfterVerification: true` — auto-login after verify
  - `emailAndPassword.requireEmailVerification: true` — block login until verified
- Sender: Personal verified email in Brevo (no custom domain needed for free tier)
- Token expiry: Configurable in Better-Auth (set 24h for verification, 1h for password reset)

**Alternatives Considered**:
- Resend: Popular but requires domain verification — overhead for hackathon.
- SendGrid: Free tier limited; Brevo offers more generous free plan.
- Nodemailer + Gmail SMTP: Unreliable for production; Google blocks automated sends.

**Sources**: [Email Concepts](https://www.better-auth.com/docs/concepts/email), [Email & Password](https://www.better-auth.com/docs/authentication/email-password)

---

### RQ-4: How to set up the React client for Better-Auth in Docusaurus?

**Decision**: Use `createAuthClient` from `better-auth/react` with `baseURL` pointing to the Express auth service.

**Rationale**: Better-Auth provides a React-specific client with hooks (`useSession`) that integrate naturally with React's state model. Docusaurus is React-based, so this works directly. The client handles cookie-based session management, request states, and TypeScript inference.

**Key Details**:
```ts
import { createAuthClient } from "better-auth/react";
export const authClient = createAuthClient({
  baseURL: "http://localhost:3005", // Better-Auth Express service
});
```
- Hooks: `useSession()` returns `{ data, isPending, error, refetch }`
- Methods: `authClient.signUp.email()`, `authClient.signIn.email()`, `authClient.signOut()`
- Password reset: `authClient.requestPasswordReset()`, `authClient.resetPassword()`
- Session: Cookie-based, automatically sent with credentials
- Cross-origin: Requires `credentials: "include"` on fetch (handled by client)

**Alternatives Considered**:
- Vanilla client (`better-auth/client`): Works but loses React hooks; less ergonomic in Docusaurus components.
- Custom fetch wrapper: Unnecessary — Better-Auth client handles all auth flows.

**Sources**: [Client Concepts](https://www.better-auth.com/docs/concepts/client)

---

### RQ-5: How to store user background (software/hardware) in Better-Auth?

**Decision**: Use Better-Auth's `user.additionalFields` to extend the user table with `softwareBackground` and `hardwareBackground` JSON array fields, plus `backgroundCompleted` boolean.

**Rationale**: Better-Auth natively supports extending the user schema with additional fields. Storing background data directly on the user table avoids an extra join and keeps the profile retrieval in a single query. JSON arrays are well-supported in PostgreSQL.

**Key Details**:
```ts
user: {
  additionalFields: {
    softwareBackground: {
      type: "string[]",
      required: false,
      defaultValue: [],
      input: true,
    },
    hardwareBackground: {
      type: "string[]",
      required: false,
      defaultValue: [],
      input: true,
    },
    softwareOther: {
      type: "string",
      required: false,
    },
    hardwareOther: {
      type: "string",
      required: false,
    },
    backgroundCompleted: {
      type: "boolean",
      required: false,
      defaultValue: false,
    },
  },
}
```
- CLI `generate`/`migrate` will add these columns automatically
- Client can update via `authClient.updateUser()`
- TypeScript types auto-inferred with `inferAdditionalFields` plugin

**Alternatives Considered**:
- Separate `user_profile` table: More normalized but adds join complexity; unnecessary for this scale.
- localStorage-only: Would lose data across devices; spec requires server-side storage.

**Sources**: [Users & Accounts](https://www.better-auth.com/docs/concepts/users-accounts), [Database](https://www.better-auth.com/docs/concepts/database)

---

### RQ-6: Session management configuration for the spec requirements

**Decision**: Configure Better-Auth sessions with 30-minute idle expiry (default), 30-day "remember me" expiry, and cookie caching for performance.

**Rationale**: Spec requires FR-009 (30 min idle / 30 day remember-me). Better-Auth's session system supports `expiresIn` and `updateAge` configuration. The `rememberMe` flag on `signIn.email()` can be used to set extended session duration. Cross-tab sync via `storage` events in the React client.

**Key Details**:
- Default session: `expiresIn: 30 * 60` (30 minutes)
- Remember-me: Handled by setting longer expiry when `rememberMe: true`
- Cookie settings: `httpOnly: true`, `secure: true`, `sameSite: "none"` (cross-origin)
- Session rotation: Better-Auth handles token rotation on login by default
- Cross-tab sync: `useSession()` hook + `BroadcastChannel` or `storage` event listener
- Multi-device logout: `revokeOtherSessions()` after password change

**Alternatives Considered**:
- JWT stateless sessions: More scalable but harder to revoke; cookie-based preferred for security.
- Redis session store: Overkill for hackathon scale; Postgres is sufficient.

**Sources**: [Session Management](https://www.better-auth.com/docs/concepts/session-management)

---

### RQ-7: How should FastAPI validate Better-Auth sessions?

**Decision**: FastAPI reads the `better-auth.session_token` cookie from incoming requests and queries the Neon Postgres `session` table directly to validate.

**Rationale**: Per spec clarification, FastAPI and Better-Auth are decoupled runtimes sharing a database. FastAPI doesn't call Better-Auth's API — it queries the shared `session` table. This avoids inter-service HTTP calls and keeps FastAPI independent.

**Key Details**:
- FastAPI dependency: `asyncpg` or `psycopg2` to query Neon Postgres
- Validation query: `SELECT * FROM session WHERE token = $1 AND expires_at > NOW()`
- Cookie name: `better-auth.session_token` (Better-Auth default)
- Middleware pattern: FastAPI `Depends()` for protected endpoints
- Returns user_id for downstream use (e.g., attaching to RAG context)

**Alternatives Considered**:
- HTTP call to Better-Auth `/api/auth/session`: Adds latency and coupling between services.
- JWT validation: Better-Auth uses opaque session tokens by default, not JWTs.
- Shared Redis: Unnecessary infrastructure for this scale.

---

### RQ-8: Account lockout and rate limiting strategy

**Decision**: Use Better-Auth's built-in rate limiting and implement account lockout via a custom `denyList` check or plugin.

**Rationale**: Better-Auth doesn't have built-in account lockout after N failed attempts. We'll implement this with a `failed_login_attempts` counter on the user table (via additionalFields) and a `lockoutUntil` timestamp. Check these before allowing sign-in via a `before` hook or custom endpoint wrapper.

**Key Details**:
- Add `failedLoginAttempts` (number, default 0) and `lockoutUntil` (string/date, nullable) to user additionalFields
- On failed login: increment counter; if >= 5, set `lockoutUntil = now + 15 minutes`
- On successful login: reset counter to 0
- Check lockout in a `before` authentication hook or custom middleware
- Better-Auth hooks: `onRequest`, `after` callbacks on auth configuration

**Alternatives Considered**:
- IP-based rate limiting: Spec says per-account, not per-IP.
- Redis-based counter: Overkill; database counter is sufficient for hackathon scale.

---

## Summary of Technology Choices

| Component | Choice | Package/Service |
|-----------|--------|-----------------|
| Auth Framework | Better-Auth v1.4.x | `better-auth` |
| Auth Runtime | Node.js + Express | `express` |
| Database | Neon Serverless Postgres | `pg` |
| Email Provider | Brevo (Sendinblue) | REST API |
| Frontend Client | Better-Auth React | `better-auth/react` |
| Session Storage | Postgres (cookie-based) | Built-in |
| Password Hashing | scrypt (Better-Auth default) | Built-in |
| Schema Management | Better-Auth CLI | `@better-auth/cli` |
