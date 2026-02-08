---
id: "0010"
title: E2E Auth Playwright Testing
stage: green
date: 2026-02-01
surface: agent
model: claude-opus-4-5-20251101
feature: 005-better-auth
branch: main
user: user
command: implement
labels: ["e2e", "playwright", "auth", "testing"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - playwright.config.ts
  - package.json
  - e2e/global-setup.ts
  - e2e/global-teardown.ts
  - e2e/helpers/constants.ts
  - e2e/helpers/db-helpers.ts
  - e2e/fixtures/auth.fixture.ts
  - e2e/tests/tab-switching.spec.ts
  - e2e/tests/password-strength.spec.ts
  - e2e/tests/signup.spec.ts
  - e2e/tests/signin.spec.ts
  - e2e/tests/email-verification.spec.ts
  - e2e/tests/password-reset.spec.ts
  - e2e/tests/auth-navbar.spec.ts
  - e2e/tests/signout.spec.ts
  - e2e/tests/auth-guard.spec.ts
  - e2e/tests/account-lockout.spec.ts
  - src/components/Auth/AuthForm.tsx
  - src/components/Auth/AuthGuard.tsx
  - src/components/Auth/AuthNavbar.tsx
  - src/components/Auth/PasswordStrength.tsx
  - src/components/Auth/ResetPasswordForm.tsx
tests:
  - e2e/tests/tab-switching.spec.ts (4 tests)
  - e2e/tests/password-strength.spec.ts (4 tests)
  - e2e/tests/signup.spec.ts (4 tests)
  - e2e/tests/signin.spec.ts (5 tests)
  - e2e/tests/email-verification.spec.ts (4 tests)
  - e2e/tests/password-reset.spec.ts (5 tests)
  - e2e/tests/auth-navbar.spec.ts (4 tests)
  - e2e/tests/signout.spec.ts (2 tests)
  - e2e/tests/auth-guard.spec.ts (3 tests)
  - e2e/tests/account-lockout.spec.ts (2 tests)
---

## Prompt

Implement the following plan:

# E2E Auth Testing Plan — Playwright + Real Backend

Add Playwright E2E tests for the full auth system (signup, signin, email verification UI, password reset, lockout, signout, AuthGuard, navbar state, password strength meter, tab switching). Tests run against the real auth-service backend on localhost:3005 with Neon Postgres.

## Response snapshot

Implemented the full E2E auth testing suite with Playwright:

1. **Dependencies**: Installed `@playwright/test`, `pg`, `@types/pg`, `dotenv` as devDependencies. Added `test:e2e`, `test:e2e:headed`, `test:e2e:report` npm scripts.

2. **Config**: Created `playwright.config.ts` with dual webServer entries (auth-service on 3005, frontend on 3000), serial workers, Chromium-only, global setup/teardown.

3. **Infrastructure**: Created DB helpers (`verifyUserEmail`, `deleteUser`, `deleteAllTestUsers`, `resetLockout`, `createUserViaAPI`, `healthCheck`), test constants, custom fixtures (`createTestUser` with auto-cleanup, `authenticatedPage`), global setup (health check + seed verified user), global teardown (clean test users + close pool).

4. **Data-testid attributes**: Added 9 `data-testid` attributes across 5 component files (AuthForm, AuthGuard, AuthNavbar, PasswordStrength, ResetPasswordForm).

5. **Test specs**: 10 spec files with 37 total tests covering tab switching, password strength, signup, signin, email verification, password reset, auth navbar, signout, auth guard, and account lockout.

Adapted auth-guard tests to match actual architecture (AuthGuard component not mounted at a route; tests use navbar auth state on docs pages instead).

## Outcome

- ✅ Impact: Full E2E auth testing coverage with 37 tests across 10 spec files
- 🧪 Tests: 37 Playwright E2E tests, all recognized by `npx playwright test --list`
- 📁 Files: 15 new files created, 5 existing files modified (data-testid additions), 1 file updated (package.json)
- 🔁 Next prompts: Run tests with `npm run test:e2e`, fix any failures, add CI integration
- 🧠 Reflection: AuthGuard tests adapted to actual architecture — component isn't route-mounted, so tested via navbar auth state instead

## Evaluation notes (flywheel)

- Failure modes observed: None during setup; tests need running against live services to verify
- Graders run and results (PASS/FAIL): Config validation PASS (npx playwright test --list shows all 37 tests)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Run full suite against live auth-service and frontend
