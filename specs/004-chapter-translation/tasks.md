# Tasks: Chapter Translation Toggle

**Input**: Design documents from `/specs/004-chapter-translation/`
**Prerequisites**: plan.md (required), spec.md (required), research.md, data-model.md, contracts/translate-api.yaml

**Tests**: Not explicitly requested in the feature specification. Test tasks are excluded.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Frontend**: `src/` (Docusaurus root)
- **Backend**: `backend/src/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Add new dependencies and create directory structure for the translation feature

- [X] T001 Add `beautifulsoup4>=4.12.0` and `lxml>=5.0.0` to `backend/requirements.txt`
- [X] T002 [P] Create directory `src/components/ChapterTranslation/` for frontend translation components
- [X] T003 [P] Add Noto Nastaliq Urdu font import and RTL/Urdu CSS styles to `src/css/custom.css` (font-family, direction, line-height, text-align, code block LTR overrides)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Backend translation infrastructure that ALL user stories depend on

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T004 Create Pydantic models `TranslationRequest` and `TranslationResponse` in `backend/src/models/translation.py` per data-model.md and contracts/translate-api.yaml
- [X] T005 Implement `TranslatorService` in `backend/src/services/translator.py` — parse HTML with BeautifulSoup, extract translatable text (skip `<pre>` and `<code>` tags), build translation prompt with technical term preservation list, call existing Groq LLM service, reassemble HTML with translated text
- [X] T006 Create `/api/translate` POST endpoint in `backend/src/api/translate.py` — accept `TranslationRequest`, call `TranslatorService`, return `TranslationResponse`, handle 400/500/504 errors per contract
- [X] T007 Register the translate router in `backend/src/main.py` by importing `backend/src/api/translate.py` and calling `app.include_router(translate.router)`

**Checkpoint**: Backend translation endpoint is functional — can be tested with curl/Postman

---

## Phase 3: User Story 1 — Translate Current Chapter to Urdu (Priority: P1) 🎯 MVP

**Goal**: Reader clicks a "Translate" button on any chapter page, and the chapter content is translated to Urdu in-place via the backend API, with loading state shown during translation.

**Independent Test**: Navigate to any chapter, click the Translate button, verify all visible content changes from English to Urdu while maintaining page layout and scroll position.

### Implementation for User Story 1

- [X] T008 [P] [US1] Create localStorage cache utilities in `src/components/ChapterTranslation/cache.ts` — implement `ChapterTranslationCache` interface, `cacheTranslation()`, `getCachedTranslation()` (with 7-day expiry check), and `clearTranslationCache()` per data-model.md cache strategy
- [X] T009 [P] [US1] Create `TranslationButton` component in `src/components/ChapterTranslation/TranslationButton.tsx` — manage `TranslationState` (language, isTranslating, error), render button with label "Translate to Urdu" / "Translating..." / "Show Original", disable during translation, show error state
- [X] T010 [P] [US1] Create component styles in `src/components/ChapterTranslation/styles.module.css` — button positioning (fixed or sticky near top), loading spinner animation, disabled state styling, error notification styling
- [X] T011 [US1] Wire translation logic in `TranslationButton.tsx` — on click: extract `.markdown` container innerHTML, check cache first, if cache miss POST to `/api/translate`, on success replace `.markdown` innerHTML with translated HTML, add `dir="rtl"` and `urdu-content` class to container, save both english and urdu to localStorage cache
- [X] T012 [US1] Add scroll position preservation in `TranslationButton.tsx` — before DOM update capture scroll percentage, after DOM update restore scroll position via `requestAnimationFrame` per research.md pattern
- [X] T013 [US1] Integrate `TranslationButton` into `src/theme/DocItem/Layout/index.tsx` — import component, extract `metadata.id` and `metadata.title` from `useDoc()`, render `<TranslationButton chapterId={chapterId} chapterTitle={chapterTitle} />` alongside existing `FloatingChatButton`

**Checkpoint**: User Story 1 fully functional — can translate any chapter to Urdu with loading indicator, cached results, and preserved scroll position

---

## Phase 4: User Story 2 — Toggle Back to English (Priority: P1)

**Goal**: Reader viewing Urdu content clicks the same button to instantly restore the original English content from cache, with no API call or loading state.

**Independent Test**: Translate a chapter to Urdu, click the button again, verify original English content is restored exactly with no delay.

### Implementation for User Story 2

- [X] T014 [US2] Implement toggle-back logic in `TranslationButton.tsx` — when language state is `urdu`, on click: read `english` content from localStorage cache, replace `.markdown` innerHTML, remove `dir="rtl"` and `urdu-content` class, restore `dir="ltr"`, update state to `english` — no API call, no loading state
- [X] T015 [US2] Verify content integrity in toggle-back — ensure restored English HTML is identical to pre-translation content (no mutation from cache serialization/deserialization)

**Checkpoint**: User Story 2 fully functional — instant toggle-back to English from cached content, button label reflects current language state

---

## Phase 5: User Story 3 — Preserve Formatting and Structure (Priority: P2)

**Goal**: Translated Urdu content preserves all formatting: headings (H1-H4), bullet/numbered lists, bold/italic emphasis, tables, and code blocks remain untranslated in English.

**Independent Test**: Translate a chapter with diverse formatting elements and verify each element type is preserved correctly in the Urdu output.

### Implementation for User Story 3

- [X] T016 [US3] Refine translation prompt in `backend/src/services/translator.py` — ensure prompt explicitly instructs LLM to preserve ALL HTML tags, heading hierarchy, list structure, emphasis tags (`<strong>`, `<em>`), table structure, and class/id attributes exactly
- [X] T017 [US3] Enhance BeautifulSoup HTML parsing in `backend/src/services/translator.py` — mark `<pre>`, `<code>`, `<img>`, `<video>`, `<iframe>` elements as skip-translation, extract only text nodes from remaining elements, rebuild HTML with translated text preserving tag structure
- [X] T018 [US3] Add technical term preservation list in `backend/src/services/translator.py` — maintain list of terms (ROS, ROS2, Gazebo, Isaac Sim, NVIDIA, URDF, SLAM, Nav2, lidar, LiDAR, IMU, API, SDK, Python, C++) that must remain in English within Urdu text

**Checkpoint**: Formatting preservation verified — headings, lists, emphasis, tables, and code blocks all render correctly in Urdu mode

---

## Phase 6: User Story 4 — Visual Feedback During Translation (Priority: P3)

**Goal**: Clear visual feedback while translation is in progress — loading indicator on button, disabled state, and no loading shown for cached restores.

**Independent Test**: Click Translate on a long chapter, observe loading indicator appears, then disappears when translation completes. Toggle back shows no loading.

### Implementation for User Story 4

- [X] T019 [US4] Enhance loading state in `TranslationButton.tsx` — add CSS spinner animation to button during `isTranslating: true`, change button text to "Translating...", ensure button is visually disabled (opacity, cursor)
- [X] T020 [US4] Add error handling UI in `TranslationButton.tsx` — on translation failure display inline error message below button (or toast notification), auto-dismiss after 5 seconds, revert button to "Translate to Urdu" state, keep original English content displayed
- [X] T021 [US4] Handle edge case: navigation during translation in `TranslationButton.tsx` — use `useEffect` cleanup to abort in-flight fetch request via `AbortController` when component unmounts (user navigates away)

**Checkpoint**: Visual feedback complete — loading, error, and edge case states all handled gracefully

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [X] T022 [P] Validate that existing RAG chatbot (`src/components/ChapterChat/`) continues functioning identically when translation feature is active — verify no DOM conflicts or event handler interference
- [X] T023 [P] Handle edge case: offline translation attempt — if no cache exists and fetch fails due to network, show user-friendly "You are offline" error message in `TranslationButton.tsx`
- [X] T024 Run quickstart.md validation — follow all steps in `specs/004-chapter-translation/quickstart.md` to verify end-to-end developer setup

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies — can start immediately
- **Foundational (Phase 2)**: Depends on T001 (backend dependencies) — BLOCKS all user stories
- **User Story 1 (Phase 3)**: Depends on Phase 2 completion (backend endpoint ready)
- **User Story 2 (Phase 4)**: Depends on Phase 3 completion (needs cache populated by US1)
- **User Story 3 (Phase 5)**: Depends on Phase 2 completion (backend service refinement) — can run in parallel with US1/US2 frontend work
- **User Story 4 (Phase 6)**: Depends on Phase 3 completion (needs TranslationButton component)
- **Polish (Phase 7)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Depends on Foundational (Phase 2) — no dependencies on other stories
- **User Story 2 (P1)**: Depends on US1 (cache and toggle state must exist) — sequential after US1
- **User Story 3 (P2)**: Depends on Foundational (Phase 2) — backend-only, can parallel with US1 frontend
- **User Story 4 (P3)**: Depends on US1 (TranslationButton must exist) — enhances existing component

### Within Each User Story

- Models/interfaces before services
- Services before endpoints/components
- Cache utilities before component logic
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- T002 and T003 can run in parallel (different files)
- T008, T009, T010 can run in parallel (different files)
- T022 and T023 can run in parallel (different concerns)
- US3 backend refinement (T016-T018) can overlap with US1 frontend work (T008-T013) since they modify different files

---

## Parallel Example: User Story 1

```bash
# Launch cache, component, and styles in parallel (different files):
Task: "Create cache utilities in src/components/ChapterTranslation/cache.ts"
Task: "Create TranslationButton component in src/components/ChapterTranslation/TranslationButton.tsx"
Task: "Create component styles in src/components/ChapterTranslation/styles.module.css"

# Then sequentially:
Task: "Wire translation logic in TranslationButton.tsx"
Task: "Add scroll position preservation in TranslationButton.tsx"
Task: "Integrate into DocItem/Layout/index.tsx"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T003)
2. Complete Phase 2: Foundational backend (T004-T007)
3. Complete Phase 3: User Story 1 (T008-T013)
4. **STOP and VALIDATE**: Test translation end-to-end on a real chapter
5. Deploy/demo if ready — users can translate chapters to Urdu

### Incremental Delivery

1. Setup + Foundational → Backend endpoint ready
2. Add User Story 1 → Translate to Urdu works → Deploy (MVP!)
3. Add User Story 2 → Instant toggle-back works → Deploy
4. Add User Story 3 → Formatting preserved perfectly → Deploy
5. Add User Story 4 → Loading/error UX polished → Deploy
6. Each story adds value without breaking previous stories

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- No test tasks included — tests were not explicitly requested in the feature spec
