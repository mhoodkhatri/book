---
id: "0011"
title: E2E Test Failures Diagnosis and Fix
stage: green
date: 2026-02-02
surface: agent
model: claude-opus-4-5-20251101
feature: 005-better-auth
branch: main
user: developer
command: implement
labels: ["e2e", "playwright", "bugfix", "auth"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - e2e/helpers/db-helpers.ts
  - src/components/Auth/AuthForm.tsx
  - e2e/tests/account-lockout.spec.ts
tests:
  - e2e/tests/account-lockout.spec.ts
  - e2e/tests/auth-guard.spec.ts
  - e2e/tests/auth-navbar.spec.ts
  - e2e/tests/email-verification.spec.ts
  - e2e/tests/password-reset.spec.ts
  - e2e/tests/password-strength.spec.ts
  - e2e/tests/signin.spec.ts
  - e2e/tests/signout.spec.ts
  - e2e/tests/signup.spec.ts
  - e2e/tests/tab-switching.spec.ts
---

## Prompt

Implement the following plan:

# Run E2E Tests and Fix Failures

Run `npm run test:e2e:headed` against the live auth-service + frontend, diagnose failures, and fix them. All 37 E2E Playwright tests across 10 spec files covering auth tab switching, password strength, signup, signin, email verification, password reset, auth navbar, signout, auth guard, and account lockout.

## Response snapshot

Ran full E2E suite (37 tests). Initial run: 24 passed, 13 failed. Diagnosed 3 root causes and applied fixes:

1. **Missing Origin header** in `createUserViaAPI()` — Better-Auth requires `Origin` header for CORS validation. Added `Origin: http://localhost:3000` to the fetch call in `e2e/helpers/db-helpers.ts`. This fixed 10 of 13 failures (all tests using `createTestUser` fixture or `authenticatedPage` fixture).

2. **Unverified user detection bug** in `AuthForm.tsx` — Better-Auth returns "Email not verified" but the frontend checked for substring "verify" (not in "verified") and "verification" (not in "verified"). Changed to check for "verif" which matches all variants. Also added passthrough for lockout error messages containing "locked" or "try again in".

3. **Account lockout test timeout** — 5+1 failed login attempts against remote Neon DB took ~35s, exceeding the default 30s test timeout. Added `test.describe.configure({ timeout: 60_000 })`.

Final run: **37/37 tests pass**.

## Outcome

- Impact: All 37 E2E auth tests now pass consistently
- Tests: 37 passed, 0 failed across 10 spec files
- Files: 3 files modified (db-helpers.ts, AuthForm.tsx, account-lockout.spec.ts)
- Next prompts: Run tests in CI, add retry logic for flaky network conditions
- Reflection: The Origin header issue was a server-side CORS requirement not accounted for in test helpers. The "verif" substring matching is a more robust approach for Better-Auth error detection.

## Evaluation notes (flywheel)

- Failure modes observed: Missing Origin header on API calls, substring mismatch for "verified" vs "verify", test timeout exceeded for slow remote DB
- Graders run and results (PASS/FAIL): Playwright full suite PASS (37/37)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Add CI workflow for E2E tests with auth-service
