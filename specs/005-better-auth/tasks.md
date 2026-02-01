# Tasks: Better-Auth Authentication

**Input**: Design documents from `/specs/005-better-auth/`
**Prerequisites**: plan.md (required), spec.md (required), research.md, data-model.md, contracts/auth-api.yaml, contracts/fastapi-auth-middleware.yaml, quickstart.md

**Tests**: Not explicitly requested — test tasks omitted. Manual verification checklists provided per phase.

**Organization**: Tasks grouped by user story for independent implementation and testing.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2)
- Exact file paths included in every task description

## Path Conventions

- **Auth Service (new)**: `auth-service/src/`
- **FastAPI Backend (existing)**: `backend/src/`
- **Docusaurus Frontend (existing)**: `src/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Initialize auth-service project and install frontend dependencies

- [x] T001 Create `auth-service/` directory with `package.json` (name: `auth-service`, scripts: `dev: tsx src/index.ts`, `generate: better-auth generate`, `migrate: better-auth migrate`) and install dependencies: `better-auth`, `express`, `cors`, `dotenv`, `pg`
- [x] T002 Create `auth-service/tsconfig.json` with `target: ES2022`, `module: NodeNext`, `moduleResolution: NodeNext`, `outDir: dist`, `strict: true`, `esModuleInterop: true`
- [x] T003 [P] Install dev dependencies in `auth-service/`: `typescript`, `@types/express`, `@types/cors`, `@better-auth/cli`, `tsx`, `@types/pg`
- [x] T004 [P] Create `auth-service/.env.example` with placeholders for `DATABASE_URL`, `BETTER_AUTH_SECRET`, `BETTER_AUTH_URL`, `BREVO_API_KEY`, `BREVO_SENDER_EMAIL`, `BREVO_SENDER_NAME`, `FRONTEND_URL`
- [x] T005 [P] Install `better-auth` in the Docusaurus frontend root `package.json` (needed for `better-auth/react` client)
- [x] T006 [P] Add `asyncpg` to `backend/requirements.txt` and add `DATABASE_URL` placeholder to `backend/.env.example` (or existing `.env`)

**Checkpoint**: Both projects have all dependencies installed. No code yet.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core auth configuration and database schema — MUST complete before any user story

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T007 Create Better-Auth configuration in `auth-service/src/auth.ts`: configure `betterAuth()` with `pg.Pool` database adapter (`process.env.DATABASE_URL`), `emailAndPassword` (enabled, requireEmailVerification, minPasswordLength: 8, maxPasswordLength: 128), `emailVerification` (sendOnSignUp: true, autoSignInAfterVerification: true), session config (expiresIn: 30*60, updateAge: 5*60), and `user.additionalFields` for `softwareBackground`, `hardwareBackground`, `softwareOther`, `hardwareOther`, `backgroundCompleted`, `failedLoginAttempts`, `lockoutUntil` per data-model.md
- [x] T008 Create Brevo email service in `auth-service/src/services/brevo.ts`: implement `sendEmail(to, subject, htmlContent)` function using Brevo REST API (`POST https://api.brevo.com/v3/smtp/email`) with `api-key` header from `process.env.BREVO_API_KEY`, sender from env vars, and retry logic (3 attempts with exponential backoff per FR-014). Export `sendVerificationEmail(user, url)` and `sendResetPasswordEmail(user, url)` wrappers
- [x] T009 Wire Brevo into Better-Auth config in `auth-service/src/auth.ts`: set `emailVerification.sendVerificationEmail` to call `sendVerificationEmail` from `brevo.ts`, set `emailAndPassword.sendResetPassword` to call `sendResetPasswordEmail` from `brevo.ts`
- [x] T010 Create Express server in `auth-service/src/index.ts`: import `dotenv/config`, create Express app, configure CORS with `origin: process.env.FRONTEND_URL`, `credentials: true`. Mount Better-Auth via `app.all("/api/auth/*", toNodeHandler(auth))` BEFORE `express.json()`. Add `express.json()` after for custom routes. Listen on port 3005
- [x] T011 Create audit log service in `auth-service/src/lib/audit.ts`: implement `logAuthEvent(pool, { userId, eventType, ipAddress, userAgent, success, metadata })` that inserts into `auth_audit_log` table. Event types: signup, login, logout, password_reset, account_delete, email_verify, login_failed, lockout
- [x] T012 Create `auth_audit_log` table migration SQL in `auth-service/src/migrations/001-audit-log.sql`: CREATE TABLE with columns per data-model.md (id SERIAL PK, userId VARCHAR(36) FK, eventType VARCHAR(50), ipAddress VARCHAR(45), userAgent TEXT, success BOOLEAN, metadata JSONB, createdAt TIMESTAMP DEFAULT NOW()), plus index on (userId, eventType, createdAt)
- [x] T013 Run Better-Auth schema generation (`npx @better-auth/cli generate` in `auth-service/`) and apply migrations (`npx @better-auth/cli migrate`) to create `user`, `session`, `account`, `verification` tables in Neon Postgres. Then apply custom `001-audit-log.sql` migration
- [x] T014 Create Better-Auth React client in `src/lib/auth-client.ts`: import `createAuthClient` from `better-auth/react`, export `authClient` with `baseURL` set to `http://localhost:3005` (or `process.env.REACT_APP_AUTH_URL`). Add `inferAdditionalFields` plugin for TypeScript support of custom user fields
- [x] T015 Create AuthContext provider in `src/contexts/AuthContext.tsx`: use `authClient.useSession()` hook, expose `{ user, session, isPending, error, signOut, refetch }` via React context. Add cross-tab sync via `BroadcastChannel` or `storage` event listener that calls `refetch()` on auth state change

**Checkpoint**: Foundation ready — auth service starts, database schema exists, frontend client configured, context provider available.

---

## Phase 3: User Story 1 — New User Signup with Email Verification (Priority: P1) 🎯 MVP

**Goal**: A visitor can sign up, receive a verification email, click the link, and become a verified user.

**Independent Test**: Create a new account → receive email → click verification link → confirm account is verified and auto-signed-in.

### Implementation for User Story 1

- [x] T016 [P] [US1] Create `PasswordStrength` component in `src/components/Auth/PasswordStrength.tsx`: real-time password strength meter (FR-038) that evaluates length, uppercase, lowercase, numbers, special chars. Display strength bar (weak/fair/strong/very-strong) with color coding. Export as default
- [x] T017 [P] [US1] Create `AuthForm` component in `src/components/Auth/AuthForm.tsx`: tabbed sign-up / sign-in form with tab toggle (FR-036). Sign-up tab: name, email, password fields with inline validation (email format, password strength via PasswordStrength component). Password visibility toggle (FR-037). Proper `autocomplete` attributes (FR-039). Submit button disabled while in-flight (FR-040). Uses `authClient.signUp.email()` on submit. Shows success message: "Verification email sent — check your inbox"
- [x] T018 [US1] Create `/auth` page in `src/pages/auth.tsx`: full-page Docusaurus page using `Layout` wrapper. Renders `AuthForm` component. Reads `?tab=signup|signin` query param for initial tab. Reads `?redirect=` param for post-auth redirect
- [x] T019 [US1] Add toast notification system in `src/components/Auth/Toast.tsx`: simple toast component for key action feedback (FR-041). Support success/error/info variants. Auto-dismiss after 5 seconds. Export `useToast()` hook and `ToastContainer` component
- [x] T020 [US1] Wire email verification redirect in `auth-service/src/auth.ts`: set `emailVerification.sendVerificationEmail` callback URL to include `callbackURL` pointing to `/profile/background` (post-verification redirect per US3). Add audit log entry for `email_verify` event on successful verification via `auth.api` hooks
- [x] T021 [US1] Add signup audit logging in `auth-service/src/index.ts` or `auth-service/src/auth.ts`: hook into Better-Auth's `after` callback for sign-up to call `logAuthEvent` with eventType `signup`, capture IP and user agent

**Checkpoint**: Users can sign up, receive verification email via Brevo, click link to verify. Auth form is functional with validation and password strength meter.

---

## Phase 4: User Story 2 — User Sign-In and Session Management (Priority: P1)

**Goal**: A returning user can sign in, maintain session across pages, use "Remember me", and log out.

**Independent Test**: Sign in with valid credentials → navigate between pages (session persists) → log out → session invalidated.

### Implementation for User Story 2

- [x] T022 [US2] Add sign-in logic to `AuthForm` sign-in tab in `src/components/Auth/AuthForm.tsx`: email + password fields, "Remember me" checkbox, `authClient.signIn.email({ email, password, rememberMe })` on submit. Generic error message on failure: "Invalid email or password" (FR-005). Show "Please verify your email first" for unverified accounts with "Resend verification" link
- [x] T023 [US2] Implement account lockout middleware in `auth-service/src/middleware/lockout.ts`: before sign-in, check `failedLoginAttempts >= 5` AND `lockoutUntil > now`. If locked, return 403 with lockout message and remaining time. On failed login: increment `failedLoginAttempts`, if >= 5 set `lockoutUntil = now + 15 min`. On successful login: reset `failedLoginAttempts` to 0 and clear `lockoutUntil`. Log `login_failed` and `lockout` events to audit log
- [x] T024 [US2] Wire lockout middleware into Better-Auth hooks in `auth-service/src/auth.ts`: use `before` or `onRequest` hook to run lockout check before sign-in attempts. Wire `after` hook for successful/failed login to update counters
- [x] T025 [P] [US2] Create `AuthNavbar` component in `src/components/Auth/AuthNavbar.tsx`: when unauthenticated show "Sign In" button linking to `/auth`. When authenticated show user dropdown with name, "Account Settings" link to `/profile/settings`, and "Sign Out" button. Sign out calls `authClient.signOut()` and shows toast
- [x] T026 [US2] Integrate `AuthNavbar` into Docusaurus navbar via `src/theme/Navbar/Content/index.tsx` or navbar `customItems` in `docusaurus.config.ts`: swizzle navbar if needed, add `AuthNavbar` to right side of navbar
- [x] T027 [US2] Wrap root layout with `AuthContext` provider in `src/theme/Root.tsx` (Docusaurus root wrapper): create or modify `src/theme/Root.tsx` to wrap children with `AuthContextProvider` and `ToastContainer`. This ensures auth state is available on every page
- [x] T028 [US2] Add loading/skeleton state in `src/components/Auth/AuthNavbar.tsx`: while `isPending` from `useSession()`, show a skeleton placeholder instead of sign-in button or user dropdown (FR-042). Prevent flash of incorrect auth state
- [x] T029 [US2] Add login/logout audit logging in `auth-service/src/auth.ts`: hook into Better-Auth `after` callbacks for sign-in (eventType: `login`) and sign-out (eventType: `logout`). Capture IP and user agent

**Checkpoint**: Users can sign in, see their name in navbar, navigate without losing session, use "Remember me", and sign out. Account locks after 5 failures.

---

## Phase 5: User Story 3 — User Background Capture (Priority: P2)

**Goal**: After email verification, user completes a background form with software/hardware experience. Can skip and complete later.

**Independent Test**: Complete signup → verify email → redirected to background form → select skills → save → confirm data persists in profile.

### Implementation for User Story 3

- [x] T030 [P] [US3] Create `BackgroundForm` component in `src/components/Profile/BackgroundForm.tsx`: multi-select checkboxes for software (Python, ROS 2, C++, JavaScript, MATLAB, Bash/Shell) and hardware (Jetson Orin, Desktop Workstation, Laptop, Raspberry Pi, Cloud/VM) per FR-018. Include "Other" free-text fields for each. "Skip for now" button (FR-019). "Save" button calls custom endpoint. Submit disabled while in-flight
- [x] T031 [US3] Create custom background update route in `auth-service/src/routes/background.ts`: `POST /api/auth/custom/update-background` — validate session (get user from `auth.api.getSession`), validate input (softwareBackground array from predefined list, hardwareBackground array, optional other fields), update user via `auth.api.updateUser` or direct DB query, set `backgroundCompleted: true`. Return updated user profile
- [x] T032 [US3] Mount background route in `auth-service/src/index.ts`: import background router and mount under `express.json()` section (after Better-Auth handler). Add route: `app.post("/api/auth/custom/update-background", backgroundHandler)`
- [x] T033 [US3] Create `/profile/background` page in `src/pages/profile/background.tsx`: renders `BackgroundForm`. If user already has `backgroundCompleted: true`, show pre-filled form for editing. On successful save, redirect to home page or the page they came from. On "Skip", redirect to home
- [x] T034 [P] [US3] Create `AccountSettings` component in `src/components/Profile/AccountSettings.tsx`: display current user info (name, email, verification status). Show `BackgroundForm` for updating background (FR-020). Include "Delete Account" button (wired in US6)
- [x] T035 [US3] Create `/profile/settings` page in `src/pages/profile/settings.tsx`: requires authentication (redirect to `/auth?redirect=/profile/settings` if not logged in). Renders `AccountSettings` component

**Checkpoint**: Post-verification redirect to background form works. Users can fill, skip, and later update their background from account settings.

---

## Phase 6: User Story 4 — Gated Access to RAG, Translation, and Personalization (Priority: P2)

**Goal**: Unauthenticated users can read content but cannot access RAG chatbot, Translation, or Personalization. Login prompt appears instead.

**Independent Test**: Visit chapter page without logging in → try RAG chatbot → see login prompt → sign in → chatbot now accessible.

### Implementation for User Story 4

- [x] T036 [P] [US4] Create `AuthGuard` component in `src/components/Auth/AuthGuard.tsx`: wrapper that checks auth state from `AuthContext`. If authenticated, render children. If not authenticated, render a login prompt card with "Sign in to use this feature" message and "Sign In" / "Sign Up" buttons linking to `/auth?redirect={currentPath}` (FR-024)
- [x] T037 [P] [US4] Create `useAuthGuard` hook in `src/hooks/useAuthGuard.ts`: returns `{ isAuthenticated, isPending, user, loginUrl }` where `loginUrl` includes current path as redirect parameter. Convenience hook for components that need auth check logic
- [x] T038 [US4] Gate ChapterChat in `src/components/ChapterChat/FloatingButton.tsx`: wrap the floating chat button with `AuthGuard`. When unauthenticated, clicking the button shows the auth prompt instead of opening the chat (FR-021)
- [x] T039 [US4] Gate TranslationButton in `src/components/ChapterTranslation/TranslationButton.tsx`: wrap the translation trigger with `AuthGuard`. When unauthenticated, clicking shows auth prompt instead of triggering translation (FR-022)
- [x] T040 [US4] Create FastAPI auth middleware in `backend/src/middleware/auth.py`: implement `get_current_user` dependency that reads `better-auth.session_token` cookie from request, queries Neon Postgres session table (`SELECT s.*, u.* FROM session s JOIN "user" u ON s."userId" = u.id WHERE s.token = $1 AND s."expiresAt" > NOW()`), returns user dict or raises `HTTPException(401, "Authentication required")`. Use `asyncpg` connection pool initialized at startup
- [x] T041 [US4] Initialize asyncpg connection pool in `backend/src/main.py`: add startup event to create `asyncpg.Pool` with `DATABASE_URL` env var. Store on `app.state`. Add shutdown event to close pool
- [x] T042 [US4] Protect `/api/chat` endpoint in `backend/src/api/chat.py`: add `Depends(get_current_user)` to the chat endpoint function signature. Return 401 JSON error if unauthenticated
- [x] T043 [US4] Protect `/api/translate` endpoint in `backend/src/api/translate.py`: add `Depends(get_current_user)` to the translate endpoint function signature. Return 401 JSON error if unauthenticated
- [x] T044 [US4] Handle auth service unavailable gracefully in `src/components/Auth/AuthGuard.tsx`: if `AuthContext` reports connection error (auth service down), render "Service temporarily unavailable — please try again later" instead of login prompt. Textbook content remains readable (FR-043)
- [x] T045 [US4] Update frontend API calls in `src/components/ChapterChat/` and `src/components/ChapterTranslation/`: ensure `fetch` calls to `/api/chat` and `/api/translate` include `credentials: "include"` so the session cookie is sent cross-origin. Handle 401 responses by showing login prompt

**Checkpoint**: All three gated features require authentication. Unauthenticated users see login prompts. Textbook content is always accessible. Backend rejects unauthenticated API calls.

---

## Phase 7: User Story 5 — Password Reset Flow (Priority: P3)

**Goal**: User can request a password reset, receive an email, click the link, set a new password, and all sessions are invalidated.

**Independent Test**: Click "Forgot password?" → enter email → receive reset email → click link → set new password → old sessions invalidated.

### Implementation for User Story 5

- [x] T046 [US5] Configure password reset in `auth-service/src/auth.ts`: set `emailAndPassword.sendResetPassword` callback to use `sendResetPasswordEmail` from `brevo.ts`. Set token expiry to 1 hour (FR-016). Ensure all sessions are revoked on password reset (FR-028). Add audit log for `password_reset` event
- [x] T047 [US5] Add "Forgot password?" link to sign-in tab in `src/components/Auth/AuthForm.tsx`: link below the sign-in form that opens a reset request form. Reset request form: email field + submit button. Calls `authClient.forgetPassword({ email, redirectTo: "/auth?tab=reset" })`. Shows "If an account exists, a reset email has been sent" (FR-026, no account enumeration)
- [x] T048 [US5] Create password reset form in `src/components/Auth/ResetPasswordForm.tsx`: reads `token` from URL query params. Two fields: new password + confirm password with `PasswordStrength` meter. Submit calls `authClient.resetPassword({ newPassword, token })`. On success: show "Password reset — please sign in" and redirect to `/auth?tab=signin`
- [x] T049 [US5] Add reset password route to `/auth` page in `src/pages/auth.tsx`: handle `?tab=reset&token=xxx` query params. When `tab=reset`, render `ResetPasswordForm` instead of `AuthForm`

**Checkpoint**: Full password reset flow works end-to-end via Brevo email.

---

## Phase 8: User Story 6 — Account Deletion (Priority: P3)

**Goal**: User can delete their account with password confirmation. Data is anonymized, sessions invalidated.

**Independent Test**: Go to account settings → click "Delete Account" → confirm with password → account is soft-deleted and sessions invalidated.

### Implementation for User Story 6

- [x] T050 [US6] Create custom account deletion route in `auth-service/src/routes/account.ts`: `POST /api/auth/custom/delete-account` — validate session, verify password by checking hashed password (use Better-Auth's internal password verification or `scrypt`), soft-delete user (set email to `deleted-{uuid}@anonymized.local`, clear name, clear background fields), revoke all sessions for user, log `account_delete` audit event. Return 200 on success
- [x] T051 [US6] Mount account deletion route in `auth-service/src/index.ts`: import account router and mount: `app.post("/api/auth/custom/delete-account", deleteAccountHandler)`
- [x] T052 [US6] Add delete account UI to `AccountSettings` component in `src/components/Profile/AccountSettings.tsx`: "Delete Account" button that opens a confirmation modal. Modal requires password re-entry (FR-029). On confirm, calls `/api/auth/custom/delete-account` with password. On success, clear auth state and redirect to home with toast "Account deleted"

**Checkpoint**: Account deletion with password confirmation, data anonymization, and session invalidation works.

---

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: Edge cases and hardening from spec

- [x] T053 [P] Implement email normalization in `auth-service/src/auth.ts`: ensure email is lowercased before storage in Better-Auth's `before` hook for sign-up and sign-in (FR-007)
- [x] T054 [P] Implement resend verification cooldown in `src/components/Auth/AuthForm.tsx`: after clicking "Resend verification", disable the button for 60 seconds with visible countdown timer (FR-015). Cooldown tracked in component state
- [x] T055 [P] Implement cross-tab auth sync in `src/contexts/AuthContext.tsx`: listen for `storage` events or use `BroadcastChannel` to detect login/logout in other tabs. On event, call `refetch()` to update auth state in all tabs within 2 seconds (FR-011)
- [x] T056 [P] Handle session expiry during RAG query in `src/components/ChapterChat/`: if a 401 response is received while composing/sending a query, preserve the query text in component state, show auth prompt overlay, and after re-authentication restore the query text (spec edge case)
- [x] T057 [P] Add auth component styles in `src/css/custom.css`: styles for auth forms (inputs, tabs, buttons, strength meter, toast notifications, auth guard prompt, skeleton loading states). Ensure responsive design and consistent with Docusaurus theme
- [x] T058 [P] Add CORS hardening in `auth-service/src/index.ts`: restrict CORS `origin` to exact `FRONTEND_URL` value (no wildcards). Ensure `credentials: true` and appropriate `Access-Control-Allow-Headers` (FR-034)
- [x] T059 Add `.gitignore` entries for `auth-service/node_modules/`, `auth-service/.env`, `auth-service/dist/` to repo root `.gitignore`
- [x] T060 Validate complete flow against `specs/005-better-auth/quickstart.md` verification checklist: Neon DB connected, Brevo sends emails, auth service on 3005, tables created, signup/verify/signin/session/CORS all working

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Setup)**: No dependencies — start immediately
- **Phase 2 (Foundational)**: Depends on Phase 1 — BLOCKS all user stories
- **Phase 3 (US1 - Signup)**: Depends on Phase 2
- **Phase 4 (US2 - Sign-in)**: Depends on Phase 2. Sign-in tab shares `AuthForm` with US1 (T017), so US1 should complete first or T017 is designed for both tabs
- **Phase 5 (US3 - Background)**: Depends on Phase 2. Background redirect wired in US1 (T020)
- **Phase 6 (US4 - Gating)**: Depends on Phase 2 + Phase 4 (needs AuthContext/AuthGuard). Can proceed independently on backend (T040-T043) after Phase 2
- **Phase 7 (US5 - Password Reset)**: Depends on Phase 2 + Phase 3 (uses AuthForm, Brevo)
- **Phase 8 (US6 - Account Deletion)**: Depends on Phase 5 (uses AccountSettings from US3)
- **Phase 9 (Polish)**: Depends on all desired user stories being complete

### User Story Dependencies

```
Phase 1 (Setup)
    └──▶ Phase 2 (Foundation)
              ├──▶ Phase 3 (US1: Signup + Verify) 🎯 MVP
              │         ├──▶ Phase 4 (US2: Sign-in + Sessions)
              │         │         ├──▶ Phase 6 (US4: Gating)
              │         │         └──▶ Phase 7 (US5: Password Reset)
              │         └──▶ Phase 5 (US3: Background Capture)
              │                   └──▶ Phase 8 (US6: Account Deletion)
              └──▶ Phase 9 (Polish) — after all stories
```

### Within Each User Story

- Models/services before UI components
- Backend endpoints before frontend integration
- Core implementation before edge cases

### Parallel Opportunities

**Phase 1**: T003, T004, T005, T006 can all run in parallel
**Phase 2**: T008, T011, T12 can run in parallel (independent files). T07 first, then T09 (depends on T07+T08)
**Phase 3 (US1)**: T016 and T019 can run in parallel (independent components)
**Phase 4 (US2)**: T025 can run in parallel with T023/T024
**Phase 5 (US3)**: T030 and T034 can run in parallel (independent components)
**Phase 6 (US4)**: T036, T037 parallel. T040-T043 (backend) parallel with T038-T039 (frontend)
**Phase 9**: T053-T058 all different files, all parallel

---

## Parallel Example: User Story 4 (Gating)

```bash
# Frontend components (parallel — different files):
Task: "Create AuthGuard component in src/components/Auth/AuthGuard.tsx"
Task: "Create useAuthGuard hook in src/hooks/useAuthGuard.ts"

# Backend middleware (parallel — different files):
Task: "Create FastAPI auth middleware in backend/src/middleware/auth.py"
Task: "Initialize asyncpg pool in backend/src/main.py"

# Then sequentially: wire up gating in existing components
Task: "Gate ChapterChat FloatingButton"
Task: "Gate TranslationButton"
Task: "Protect /api/chat endpoint"
Task: "Protect /api/translate endpoint"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001–T006)
2. Complete Phase 2: Foundation (T007–T015)
3. Complete Phase 3: User Story 1 — Signup + Email Verification (T016–T021)
4. **STOP and VALIDATE**: Test signup → email → verify flow end-to-end
5. This alone demonstrates Better-Auth + Brevo integration for hackathon

### Incremental Delivery

1. Setup + Foundation → Auth service running, DB ready
2. Add US1 (Signup) → First user can create account (MVP!)
3. Add US2 (Sign-in) → Returning users, session management, navbar
4. Add US3 (Background) → Profile capture for personalization
5. Add US4 (Gating) → RAG/Translation require auth
6. Add US5 (Password Reset) → Complete auth lifecycle
7. Add US6 (Account Deletion) → Privacy compliance
8. Polish → Edge cases, cross-tab sync, graceful degradation

### Suggested Stop Points

- **After Phase 3 (US1)**: Minimum viable auth — signup + verify works
- **After Phase 4 (US2)**: Complete login experience — most valuable demo
- **After Phase 6 (US4)**: Full gating — hackathon requirement met
- **After Phase 9**: Production-quality auth system

---

## Summary

| Metric | Value |
|--------|-------|
| **Total Tasks** | 60 |
| **Phase 1 (Setup)** | 6 tasks |
| **Phase 2 (Foundation)** | 9 tasks |
| **Phase 3 (US1 - Signup)** | 6 tasks |
| **Phase 4 (US2 - Sign-in)** | 8 tasks |
| **Phase 5 (US3 - Background)** | 6 tasks |
| **Phase 6 (US4 - Gating)** | 10 tasks |
| **Phase 7 (US5 - Password Reset)** | 4 tasks |
| **Phase 8 (US6 - Deletion)** | 3 tasks |
| **Phase 9 (Polish)** | 8 tasks |
| **Parallel opportunities** | 22 tasks marked [P] |
| **MVP scope** | Phases 1–3 (21 tasks) |

## Notes

- [P] tasks = different files, no dependencies on incomplete tasks
- [Story] label maps task to specific user story for traceability
- Each user story is independently testable at its checkpoint
- Commit after each task or logical group
- Stop at any checkpoint to validate independently
- No test tasks generated (not requested in spec)
