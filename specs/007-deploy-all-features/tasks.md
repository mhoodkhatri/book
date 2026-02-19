# Tasks: Full Production Deployment (RAG, Translation, Auth)

**Input**: Design documents from `/specs/007-deploy-all-features/`
**Branch**: `007-deploy-all-features`
**Date**: 2026-02-18
**Prerequisites**: plan.md ✓, spec.md ✓, research.md ✓, data-model.md ✓, contracts/ ✓, quickstart.md ✓

**Organization**: Tasks grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story?] Description with file path`

- **[P]**: Can run in parallel (different files, no incomplete-task dependencies)
- **[Story]**: Which user story this task belongs to (maps exactly to spec.md)
- Setup + Foundational phases have NO story label
- File paths are exact

---

## Phase 1: Setup — Source Code Changes for Production

**Purpose**: Apply all code changes required before any Railway or GitHub Pages deployment will function correctly. These changes fix the CORS wildcard bug, enable cross-domain Bearer token auth, and wire CI/CD secrets into the build.

**⚠️ CRITICAL**: Complete all Phase 1 tasks before deploying any service. The CORS wildcard bug in `backend/src/main.py` blocks all production chat and translation requests.

- [x] T001 [P] Fix `cors_origins` to read from `CORS_ORIGINS` env var in `backend/src/config.py` — replace the hardcoded localhost list with `[o.strip() for o in os.getenv("CORS_ORIGINS","").split(",") if o.strip()] or ["http://localhost:3000","http://localhost:3001","http://127.0.0.1:3000"]`
- [x] T002 Fix CORS wildcard and enhance `/health` endpoint in `backend/src/main.py` — (a) change `allow_origins=settings.cors_origins + ["*"]` to `allow_origins=settings.cors_origins`; (b) update `GET /health` to check Qdrant connectivity and DB connectivity and return actual status instead of hardcoded `"configured"` per `specs/007-deploy-all-features/contracts/health-endpoints.md`
- [x] T003 [P] Add `Authorization: Bearer <token>` fallback to `backend/src/middleware/auth.py` — after the cookie check on line 52, if `raw_token` is None, check `request.headers.get("Authorization","")` and extract `raw_token` if it starts with `"Bearer "`; token parsing logic remains the same (split on `.`, take first part)
- [x] T004 [P] Create `backend/Procfile` with Railway start command: `web: uvicorn src.main:app --host 0.0.0.0 --port $PORT`
- [x] T005 [P] Create `auth-service/Procfile` with Railway start command: `web: npm run build && node dist/index.js`
- [x] T006 [P] Update `.github/workflows/deploy.yml` to inject `CHAT_API_URL` and `AUTH_API_URL` build-time env vars — add `env: CHAT_API_URL: ${{ secrets.CHAT_API_URL }}` and `env: AUTH_API_URL: ${{ secrets.AUTH_API_URL }}` to the "Build website" step so Docusaurus embeds production URLs at build time
- [x] T007 [P] Add `Authorization: Bearer <token>` header to the chat fetch call in `src/components/ChapterChat/index.tsx` — import `useAuth` from `@site/src/contexts/AuthContext`; extract `session` from `useAuth()`; add `...(session?.token ? { "Authorization": \`Bearer \${session.token as string}\` } : {})` to the fetch headers inside `sendMessage`
- [x] T008 [P] Add `Authorization: Bearer <token>` header to the translate fetch call in `src/components/ChapterTranslation/TranslationButton.tsx` — extract `session` from existing `useAuth()` call chain (currently uses `useAuthGuard` which calls `useAuth`; add direct `const { session } = useAuth()` import); add Bearer header to `fetch(\`\${apiUrl}/api/translate\`, ...)` headers alongside existing `Content-Type`
- [x] T009 [P] Update `backend/.env.example` to document all production env vars per `specs/007-deploy-all-features/contracts/env-vars.md` — add `CORS_ORIGINS`, `QDRANT_URL`, `QDRANT_API_KEY`, `GROQ_API_KEY`, `DATABASE_URL` with comments and example values
- [x] T010 [P] Update `auth-service/.env.example` to document all production env vars per `specs/007-deploy-all-features/contracts/env-vars.md` — add `BETTER_AUTH_SECRET`, `BETTER_AUTH_URL`, `FRONTEND_URL`, `BREVO_API_KEY`, `BREVO_FROM_EMAIL` with comments and example values

**Checkpoint**: All source code changes committed. Run `npm run build` locally to verify frontend still compiles cleanly.

---

## Phase 2: Foundational — Infrastructure Provisioning & Deployment

**Purpose**: Deploy all three services to production in the correct dependency order. Each step blocks the next.

**⚠️ CRITICAL**: Complete steps in order — each depends on the previous completing successfully.

- [ ] T011 Verify Neon Postgres schema and connectivity — confirm tables `user`, `session`, `account`, `verification`, `auth_audit_log` exist per `specs/007-deploy-all-features/data-model.md`; confirm `DATABASE_URL` includes `sslmode=require`
- [ ] T012 Provision Qdrant Cloud free-tier cluster at cloud.qdrant.io — create cluster, generate API key, record `QDRANT_URL` (format: `https://xxx.cloud.qdrant.io`) and `QDRANT_API_KEY` for use in T014
- [ ] T013 Deploy auth-service to Railway — new service with root directory `auth-service`, set env vars (`DATABASE_URL`, `BETTER_AUTH_SECRET`, `BETTER_AUTH_URL`, `FRONTEND_URL=https://mhoodkhatri.github.io`, `BREVO_API_KEY`, `BREVO_FROM_EMAIL`), set start command from `auth-service/Procfile`, configure health check path `/health` with start period 60s; record assigned URL (e.g., `https://auth-xxx.railway.app`)
- [ ] T014 Deploy FastAPI backend to Railway — new service with root directory `backend`, set env vars (`GROQ_API_KEY`, `QDRANT_URL`, `QDRANT_API_KEY` from T012, `DATABASE_URL`, `CORS_ORIGINS=https://mhoodkhatri.github.io`), set start command from `backend/Procfile`, configure health check path `/health` with start period 120s (fastembed ONNX model download); record assigned URL (e.g., `https://backend-xxx.railway.app`)
- [ ] T015 Configure GitHub Actions secrets and trigger Docusaurus deployment — go to `github.com/mhoodkhatri/book/settings/secrets/actions`; add `CHAT_API_URL` (backend URL from T014) and `AUTH_API_URL` (auth URL from T013); trigger build via `git push main` or workflow_dispatch; verify build succeeds and site is accessible at `https://mhoodkhatri.github.io/book/`
- [ ] T016 Trigger Qdrant chapter indexing on deployed backend — run `curl -X POST https://<backend-url>/api/index -H "Content-Type: application/json"`; monitor Railway backend logs for indexing progress; confirm completion (1-3 minutes); verify at least one chunk is queryable per module
- [ ] T017 [P] Verify health endpoints for all three services — run smoke test from `specs/007-deploy-all-features/contracts/health-endpoints.md`: `GET /health` on auth-service returns `{"status":"healthy","service":"auth-service"}`; `GET /health` on backend returns `{"status":"healthy"}` with `qdrant: "ok"`; `GET https://mhoodkhatri.github.io/book/` returns HTTP 200

**Checkpoint**: All three services are live and healthy. Qdrant index is populated. Ready to verify user stories.

---

## Phase 3: User Story 1 — Public Visitor Reads the Book (Priority: P1) 🎯 MVP

**Goal**: Any anonymous visitor can read all chapters at the public URL without an account, and sees clear sign-in invitations (not errors) for gated features.

**Independent Test**: Open `https://mhoodkhatri.github.io/book/` in an incognito browser and navigate to any chapter — all content loads, no login prompt, and AI Chat / Translate buttons show a sign-in invitation rather than a broken state.

- [ ] T018 [P] [US1] Verify public chapter access: open `https://mhoodkhatri.github.io/book/` in incognito, navigate to at least one chapter from each of the 4 modules, confirm all content loads without any login prompt or error per FR-001
- [ ] T019 [P] [US1] Verify gated-feature sign-in invitations for anonymous users: on a chapter page, click "Ask AI" → confirm sign-in modal appears (not a 401 error); click "Translate" → confirm `LockedFeatureModal` appears per FR-013
- [ ] T020 [US1] Verify all navigation, sidebar, and module index links work correctly at production URL — no 404s, no broken asset paths, no console errors per SC-001

**Checkpoint**: User Story 1 fully functional. Anonymous visitors can read the complete book.

---

## Phase 4: User Story 2 — New Reader Signs Up and Verifies Email (Priority: P1)

**Goal**: A new visitor completes signup, receives a verification email within 60 seconds, clicks the link, and arrives at the book in an authenticated state.

**Independent Test**: Use a fresh email address, complete the signup form on the live site, confirm verification email arrives and clicking the link activates the account.

- [ ] T021 [P] [US2] Smoke test signup flow: navigate to sign-up page on production, submit a valid email + password, confirm verification email arrives within 60 seconds in the inbox per FR-002 and FR-003
- [ ] T022 [P] [US2] Smoke test email verification: click the verification link from T021, confirm browser redirects to the book in a signed-in state with the user's name shown in the navbar per FR-003
- [ ] T023 [US2] Verify expired-link edge case: if a 24h-old link is unavailable to test live, verify the auth-service code handles the `verification.expiresAt` check and the UI displays a clear message with a resend option per spec edge cases

**Checkpoint**: User Story 2 fully functional. New users can create and verify accounts.

---

## Phase 5: User Story 3 — Authenticated User Uses Ask AI (Priority: P2)

**Goal**: A signed-in reader asks a question on a chapter page and receives a contextually relevant streaming response within 5 seconds.

**Independent Test**: Sign in, navigate to a Module 1 chapter, open AI Chat, ask "What is a ROS 2 node?" — a relevant response begins streaming within 5 seconds.

- [ ] T024 [P] [US3] Smoke test AI chat with authentication: sign in, navigate to a chapter, open chat panel, ask a chapter-relevant question, confirm streaming response starts within 5 seconds per FR-004 and FR-005 and SC-003
- [ ] T025 [P] [US3] Verify multi-turn conversation context: ask a follow-up question in the same chat session, confirm the response references prior messages (not isolated answers) per spec acceptance scenario 2
- [ ] T026 [US3] Verify auth gate on AI Chat: sign out, navigate to a chapter, click AI Chat → confirm sign-in prompt appears rather than a chat response or 401 error per FR-013

**Checkpoint**: User Story 3 fully functional. AI chat works end-to-end in production with Bearer token auth.

---

## Phase 6: User Story 7 — Reader Selects Text and Asks AI Inline (Priority: P2)

**Goal**: A signed-in reader highlights text on a chapter page, an "Ask AI" button appears, clicking it opens the chat with a response already generating about the selected passage.

**Independent Test**: Sign in, highlight a passage in any chapter, confirm the "Ask AI" popup appears, click it and verify the chat opens with a contextual response — no typing required.

- [ ] T027 [P] [US7] Smoke test desktop text selection: sign in, highlight a meaningful passage (5-50 words) in a chapter, confirm `TextSelectionPopup` appears near the selection within a moment per FR-016
- [ ] T028 [P] [US7] Smoke test selection-to-chat flow: after popup appears, click "Ask AI", confirm `ChapterChat` opens and immediately begins streaming a response about the selected text without any user typing per FR-017
- [ ] T029 [P] [US7] Smoke test mobile text selection: open the site on a mobile device or emulator at 375px, long-press and select text using native handles, confirm the "Ask AI" button appears below the selection per spec scenario 3
- [ ] T030 [US7] Verify edge cases and auth gate: (a) select a single character — popup should NOT appear; (b) sign out and select text — clicking "Ask AI" shows sign-in prompt, not a chat response per FR-018 and spec edge case

**Checkpoint**: User Story 7 fully functional. Text-selection AI flow works on both desktop and mobile.

---

## Phase 7: User Story 4 — Authenticated User Translates a Chapter (Priority: P2)

**Goal**: A signed-in reader clicks "Translate" and receives Urdu content within 30 seconds; subsequent requests for the same chapter load from cache significantly faster.

**Independent Test**: Sign in, navigate to any chapter, click "Translate to Urdu", confirm Urdu content appears within 30 seconds, then click "Show Original" and confirm English is restored.

- [ ] T031 [P] [US4] Smoke test translation: sign in, click "Translate to Urdu" on a chapter, confirm Urdu content loads within 30 seconds with correct RTL rendering per FR-006 and SC-004
- [ ] T032 [P] [US4] Verify "Show Original" toggle: after translation, click "Show Original", confirm English content is immediately restored and no page reload occurs per FR-007
- [ ] T033 [US4] Verify translation cache: translate the same chapter a second time, confirm the response returns significantly faster than the first request (localStorage cache hit) per FR-006 and SC-004

**Checkpoint**: User Story 4 fully functional. Translation works with Bearer token auth in production.

---

## Phase 8: User Story 5 — Returning User Signs In (Priority: P2)

**Goal**: A returning reader with a verified account signs in with email and password and regains immediate access to all gated features, with session persistence across browser close.

**Independent Test**: Sign in with a verified account created in Phase 4, confirm AI Chat and Translate are accessible; close and reopen the browser, confirm the session persists.

- [ ] T034 [P] [US5] Smoke test sign-in: enter valid credentials, click Sign In, confirm redirect to the book in authenticated state and gated features (AI Chat, Translate) are immediately accessible per FR-001 and spec scenario 1
- [ ] T035 [P] [US5] Verify "Remember Me" session persistence: sign in with "Remember Me" checked, close the browser tab, reopen and navigate to the site, confirm the user is still signed in per FR-019
- [ ] T036 [US5] Verify account lockout: attempt to sign in with incorrect credentials 5 consecutive times, confirm the account is temporarily locked and a clear message explaining the lockout is shown per FR-008

**Checkpoint**: User Story 5 fully functional. Returning users have reliable, persistent access.

---

## Phase 9: User Story 6 — Mobile Reader Uses All Features (Priority: P2)

**Goal**: All book features (reading, sign-in, AI chat, translation) work correctly and without layout issues on mobile screen widths starting at 375px.

**Independent Test**: Open the live site on a physical phone or browser emulator at 375px width, sign in, use AI chat, and trigger translation — all without horizontal scrolling or inaccessible controls.

- [ ] T037 [P] [US6] Verify mobile reading layout: open site at 375px, navigate chapter pages, confirm no horizontal scrolling and all text is readable per FR-015 and SC-010
- [ ] T038 [P] [US6] Verify mobile AI chat: sign in on mobile, open AI chat panel on a chapter page, confirm panel fits the screen and the text input field is usable without forced zooming per spec scenario 2
- [ ] T039 [P] [US6] Verify mobile translation: sign in on mobile, click "Translate to Urdu", confirm translated content displays correctly in the mobile layout per spec scenario 3
- [ ] T040 [US6] Verify mobile sign-in and sign-up forms: open sign-in and sign-up pages at 375px, confirm all form fields are reachable, labels are readable, and the submit button is accessible without layout issues per spec scenario 4

**Checkpoint**: User Story 6 fully functional. All features usable on mobile.

---

## Phase 10: Polish & Cross-Cutting Concerns

**Purpose**: Security verification, CORS validation, graceful degradation checks, and final documentation.

- [ ] T041 [P] Run CORS verification tests from `specs/007-deploy-all-features/contracts/cors-config.md` — run both `curl -I -X OPTIONS` preflight tests against auth-service and FastAPI backend; confirm `Access-Control-Allow-Origin: https://mhoodkhatri.github.io` and `Access-Control-Allow-Credentials: true` in response; confirm no wildcard `*` in production responses
- [ ] T042 [P] Run security checklist from `specs/007-deploy-all-features/contracts/env-vars.md` — verify no `.env` files committed (`git log --all -- "*.env"`); confirm `BETTER_AUTH_SECRET` differs between dev and prod; confirm `CORS_ORIGINS` has no wildcard; confirm `DATABASE_URL` includes `sslmode=require`
- [ ] T043 Verify graceful degradation edge cases — (a) sign out and attempt AI chat → sign-in prompt shown, not error page per FR-013; (b) navigate to chapters with backend temporarily unreachable → book content remains readable per SC-006
- [ ] T044 Run complete smoke test checklist from `specs/007-deploy-all-features/quickstart.md` sections 6.1–6.7 as a final end-to-end validation pass

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Setup)**: No dependencies — can start immediately. All T001–T010 are independent and parallel.
- **Phase 2 (Foundational)**: Depends on Phase 1 committed to `main`. Steps T011–T017 must run sequentially (each depends on the previous).
- **Phase 3 (US1)**: Depends on Phase 2 complete (Docusaurus deployed).
- **Phase 4 (US2)**: Depends on Phase 2 complete (auth-service deployed + Brevo configured).
- **Phase 5 (US3)**: Depends on Phase 2 complete (backend deployed + Qdrant indexed + Bearer token code from Phase 1).
- **Phase 6 (US7)**: Depends on Phase 5 complete (same services; builds on AI chat functionality).
- **Phase 7 (US4)**: Depends on Phase 2 complete (backend deployed + Bearer token code from Phase 1).
- **Phase 8 (US5)**: Depends on Phase 4 complete (verified account created).
- **Phase 9 (US6)**: Depends on Phases 3–8 complete (all features must be working before mobile verification).
- **Phase 10 (Polish)**: Depends on all user story phases complete.

### Critical Path (Minimum to Launch)

```
T001→T002→T003 (Phase 1 code changes)
      ↓
T011→T012→T013→T014→T015→T016→T017 (Phase 2 sequential deployment)
      ↓
Phases 3 & 4 (P1 user stories) — MUST pass before launch
      ↓
Phases 5–9 (P2 user stories)
      ↓
Phase 10 (Polish & security verification)
```

### Parallel Opportunities

- **Phase 1**: All 10 tasks (T001–T010) are fully parallel — each touches a different file.
- **Phase 2**: T011–T017 must run sequentially (dependency chain). No parallelism available.
- **Phases 3–8**: After Phase 2, US1 (Phase 3), US2 (Phase 4), and US4 (Phase 7) can be verified in parallel since they test different service paths. US3 (Phase 5) must come before US7 (Phase 6).
- **Phase 9**: All mobile verification tasks (T037–T040) are parallel.
- **Phase 10**: T041, T042 are parallel; T043, T044 run after.

---

## Implementation Strategy

### MVP First (P1 Stories Only — Phases 1–4)

1. Complete Phase 1: All 10 code changes committed
2. Complete Phase 2: All services deployed and healthy
3. Complete Phase 3: Public reading verified (US1)
4. Complete Phase 4: Signup + email verification verified (US2)
5. **STOP and VALIDATE**: Core book experience is live. Anyone can read and create an account.

### Incremental Delivery

- **Deploy P1** → Book is publicly accessible with auth ✓
- **Add US3 (AI Chat)** → Signed-in users can ask questions ✓
- **Add US7 (Text Selection)** → Inline AI prompts work ✓
- **Add US4 (Translation)** → Urdu translation available ✓
- **Add US5 (Sign-In)** → Returning users confirmed working ✓
- **Add US6 (Mobile)** → Mobile experience verified ✓
- **Phase 10** → Security + polish ✓

### Key Deployment Notes

- Railway auto-deploys on push to `main` branch after initial service creation
- `BETTER_AUTH_URL` must be updated after Railway assigns the auth-service URL (requires a redeploy)
- fastembed ONNX model downloads on first backend startup (~70MB, ~60s) — Railway start period must be 120s
- Qdrant indexing (T016) must run AFTER backend is healthy (T014)
- GitHub Pages deployment (T015) must use the Railway URLs from T013 and T014

---

## Notes

- [P] tasks = different files, no incomplete-task dependencies — safe to implement in parallel
- Story labels map exactly to spec.md user story numbers (US1, US2, US3, US4, US5, US6, US7)
- No test tasks generated — spec.md does not request TDD; acceptance is via smoke tests and manual verification per quickstart.md
- Commit Phase 1 changes as a single commit before starting Phase 2 deployment
- Stop at each Checkpoint to validate the story independently before proceeding
- Reference `specs/007-deploy-all-features/quickstart.md` for step-by-step deployment commands
