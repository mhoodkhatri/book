# Tasks: Context-Aware RAG Chatbot

**Input**: Design documents from `/specs/003-context-rag-chatbot/`
**Prerequisites**: plan.md (complete), spec.md (complete)

**Stack**: Gemini 2.0 Flash (LLM), Vercel AI SDK (frontend streaming), Google text-embedding-004, Qdrant Cloud, FastAPI

**Tests**: Not explicitly requested in specification. Tests are OPTIONAL - included as integration validation only.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions (from plan.md)

- **Backend**: `backend/src/`, `backend/scripts/`
- **Frontend**: `src/components/`, `src/theme/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization, external services setup, and basic structure

- [x] T001 Create backend directory structure per plan.md in `backend/`
- [x] T002 [P] Initialize Python project with pyproject.toml in `backend/pyproject.toml`
- [x] T003 [P] Create requirements.txt with FastAPI, qdrant-client, google-generativeai, uvicorn, python-dotenv in `backend/requirements.txt`
- [x] T004 [P] Create .env.example with GOOGLE_API_KEY, QDRANT_URL, QDRANT_API_KEY in `backend/.env.example`
- [x] T005 [P] Install Vercel AI SDK dependencies via npm: `ai` and `@ai-sdk/google`
- [x] T006 [P] Add CHAT_API_URL to docusaurus.config.ts customFields
- [ ] T007 Create Qdrant Cloud collection `textbook_chunks` with 768-dim vectors (manual/script)
- [x] T008 [P] Update .gitignore with Python and Node.js patterns

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**CRITICAL**: No user story work can begin until this phase is complete

### Backend Core Services

- [x] T009 Create config module with environment variable loading in `backend/src/config.py`
- [x] T010 [P] Create Qdrant client wrapper with connection management in `backend/src/services/qdrant.py`
- [x] T011 [P] Implement EmbeddingService with Google text-embedding-004 in `backend/src/services/embedding.py`
- [x] T012 [P] Implement GeminiService with streaming for Gemini 2.0 Flash in `backend/src/services/gemini.py`
- [x] T013 Create FastAPI app entry point with CORS configuration in `backend/src/main.py`
- [x] T014 [P] Create Pydantic models for chat request/response in `backend/src/models/chat.py`
- [x] T015 [P] Create Pydantic models for Chapter metadata in `backend/src/models/chapter.py`

### Content Indexing Pipeline

- [x] T016 Implement ContentIndexer with markdown chunking logic in `backend/src/services/indexer.py`
- [x] T017 Create index_chapters.py CLI script for batch indexing in `backend/scripts/index_chapters.py`
- [x] T018 [P] Create test_connection.py script to verify external services in `backend/scripts/test_connection.py`
- [ ] T019 Run index_chapters.py to populate Qdrant with existing chapter content

### API Scaffolding

- [x] T020 Create API router structure with __init__.py in `backend/src/api/__init__.py`
- [x] T021 [P] Implement GET /api/chapters endpoint in `backend/src/api/chapters.py`
- [x] T022 [P] Implement POST /api/index admin endpoint in `backend/src/api/index.py`

**Checkpoint**: Foundation ready - Backend can start, external services connected, content indexed

---

## Phase 3: User Story 1 - Chapter-Scoped Q&A (Priority: P1) MVP

**Goal**: Students can ask questions and receive answers using ONLY content from the current chapter, with proper refusal for out-of-scope questions.

**Independent Test**: Load any chapter page, ask a question about that chapter's content, verify:
1. Response uses only that chapter's content
2. Response includes citations [§section]
3. Out-of-scope questions are politely refused

### Backend Implementation for US1

- [x] T023 [US1] Implement RAGService with chapter-filtered Qdrant search in `backend/src/services/rag.py`
- [x] T024 [US1] Create prompt builder with chapter context and citations in `backend/src/services/prompts.py`
- [x] T025 [US1] Implement POST /api/chat streaming endpoint with SSE in `backend/src/api/chat.py`
- [x] T026 [US1] Add chapter boundary validation logic in RAGService in `backend/src/services/rag.py`
- [x] T027 [US1] Implement refusal responses for out-of-scope questions in `backend/src/services/prompts.py`

### Frontend Implementation for US1

- [x] T028 [US1] Create ChapterChat component with streaming in `src/components/ChapterChat/index.tsx`
- [x] T029 [US1] Create FloatingChatButton component with open/close state in `src/components/ChapterChat/FloatingButton.tsx`
- [x] T030 [US1] Create chat component styles in `src/components/ChapterChat/styles.module.css`
- [x] T031 [US1] Swizzle DocItem/Layout to inject FloatingChatButton in `src/theme/DocItem/Layout/index.tsx`
- [x] T032 [US1] Add context badge showing current chapter title in `src/components/ChapterChat/index.tsx`

### US1 Integration

- [ ] T033 [US1] Test chapter-scoped Q&A end-to-end with sample questions
- [ ] T034 [US1] Verify citation format displays correctly in responses
- [ ] T035 [US1] Test out-of-scope refusal with questions from other chapters

**Checkpoint**: MVP Complete - Users can ask questions about current chapter and receive chapter-scoped answers

---

## Phase 4: User Story 2 - Selected Text Context (Priority: P2)

**Goal**: Students can highlight text on the page and ask follow-up questions that use the selected text as additional context.

**Independent Test**: Select a paragraph on a chapter page, ask "explain this in simpler terms", verify:
1. Response references the selected text
2. Response still respects chapter boundaries
3. Selection is captured at submission time

### Backend Implementation for US2

- [ ] T036 [US2] Add selected_text parameter to chat request model in `backend/src/models/chat.py`
- [ ] T037 [US2] Update prompt builder to incorporate selected text context in `backend/src/services/prompts.py`
- [ ] T038 [US2] Modify RAGService to boost relevance of chunks containing selected text in `backend/src/services/rag.py`

### Frontend Implementation for US2

- [ ] T039 [US2] Add text selection capture with window.getSelection() in `src/components/ChapterChat/index.tsx`
- [ ] T040 [US2] Add selection state management to ChapterChat component in `src/components/ChapterChat/index.tsx`
- [ ] T041 [US2] Pass selected_text in useChat body configuration in `src/components/ChapterChat/index.tsx`
- [ ] T042 [US2] Add selected text preview indicator in UI in `src/components/ChapterChat/index.tsx`

### US2 Integration

- [ ] T043 [US2] Test selected text context with various paragraph selections
- [ ] T044 [US2] Verify selection is captured at submission (not affected by deselection while typing)

**Checkpoint**: US2 Complete - Text selection enhances context while maintaining chapter boundaries

---

## Phase 5: User Story 3 - Chat History Within Session (Priority: P3)

**Goal**: Students can have multi-turn conversations with follow-up questions that reference previous exchanges.

**Independent Test**: Ask a question, then ask a follow-up using pronouns ("it", "that"), verify:
1. Chatbot understands the pronoun reference
2. Context is maintained within session
3. Chapter boundary still respected

### Backend Implementation for US3

- [ ] T045 [US3] Ensure conversation history is passed in chat request in `backend/src/api/chat.py`
- [ ] T046 [US3] Update Gemini prompt to include conversation history in `backend/src/services/gemini.py`
- [ ] T047 [US3] Add conversation context handling in prompt builder in `backend/src/services/prompts.py`

### Frontend Implementation for US3

- [ ] T048 [US3] Verify useChat maintains message history automatically
- [ ] T049 [US3] Handle chapter navigation - clear chat when chapter changes in `src/components/ChapterChat/index.tsx`
- [ ] T050 [US3] Add chapter change detection using useDoc hook in `src/components/ChapterChat/index.tsx`

### US3 Integration

- [ ] T051 [US3] Test 5-message conversation with pronoun references
- [ ] T052 [US3] Test chapter navigation clears chat appropriately

**Checkpoint**: US3 Complete - Multi-turn conversations work within session boundaries

---

## Phase 6: User Story 4 - Clear Context Indicators (Priority: P3)

**Goal**: Users see clear visual indicators showing the current chapter context and any selected text.

**Independent Test**: Open chat on different chapters, verify:
1. Chapter title displayed prominently
2. Selected text indicator appears when text selected
3. Refusal messages explain the context limitation

### Frontend Implementation for US4

- [ ] T053 [P] [US4] Style context badge for prominence in `src/components/ChapterChat/styles.module.css`
- [ ] T054 [P] [US4] Style selected text preview indicator in `src/components/ChapterChat/styles.module.css`
- [ ] T055 [US4] Add clear selection button to selected text indicator in `src/components/ChapterChat/index.tsx`

### Backend Implementation for US4

- [ ] T056 [US4] Ensure refusal messages include chapter name in `backend/src/services/prompts.py`
- [ ] T057 [US4] Add chapter suggestion in refusal when topic found elsewhere in `backend/src/services/rag.py`

### US4 Integration

- [ ] T058 [US4] Test context badge displays correct chapter on each page
- [ ] T059 [US4] Verify refusal messages clearly explain context limitations

**Checkpoint**: US4 Complete - Users understand why questions are answered or refused

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

### Error Handling & Edge Cases

- [ ] T060 [P] Add graceful degradation when backend unavailable in `src/components/ChapterChat/index.tsx`
- [ ] T061 [P] Add loading spinner during streaming in `src/components/ChapterChat/index.tsx`
- [ ] T062 Add rate limiting handling for Gemini API in `backend/src/services/gemini.py`
- [ ] T063 [P] Add Qdrant connection retry logic in `backend/src/services/qdrant.py`

### Performance & Optimization

- [ ] T064 Add embedding caching for repeated queries in `backend/src/services/embedding.py`
- [ ] T065 [P] Optimize chunk retrieval with appropriate top_k values in `backend/src/services/rag.py`

### Documentation & Deployment

- [ ] T066 [P] Create Dockerfile for backend deployment in `backend/Dockerfile`
- [ ] T067 [P] Create backend README with setup instructions in `backend/README.md`
- [ ] T068 Configure deployment for backend (Railway/Render)

### Validation

- [ ] T069 Run full end-to-end test across all user stories
- [ ] T070 Validate mobile responsiveness of chat widget
- [ ] T071 Test accessibility (keyboard navigation, screen reader)

---

## Dependencies & Execution Order

### Phase Dependencies

```
Phase 1: Setup
    │
    ▼
Phase 2: Foundational (BLOCKS ALL USER STORIES)
    │
    ├─────────────────┬─────────────────┬─────────────────┐
    ▼                 ▼                 ▼                 ▼
Phase 3: US1      Phase 4: US2      Phase 5: US3      Phase 6: US4
(MVP)             (depends on       (depends on       (can parallel
                  US1 backend)      US1 frontend)     with US2/US3)
    │                 │                 │                 │
    └─────────────────┴─────────────────┴─────────────────┘
                              │
                              ▼
                    Phase 7: Polish
```

### User Story Dependencies

| Story | Can Start After | Dependencies on Other Stories |
|-------|-----------------|-------------------------------|
| US1 (P1) | Phase 2 complete | None - MVP |
| US2 (P2) | Phase 2 complete | US1 backend (T023-T027) for RAG |
| US3 (P3) | Phase 2 complete | US1 frontend (T028-T032) for chat |
| US4 (P3) | Phase 2 complete | US1 components exist |

### Parallel Opportunities

**Phase 1 (all [P] tasks):**
```
T002, T003, T004, T005, T006, T008 can run in parallel
```

**Phase 2 (after T009):**
```
T010, T011, T012 can run in parallel (different service files)
T014, T015 can run in parallel (different model files)
T021, T022 can run in parallel (different API files)
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T008)
2. Complete Phase 2: Foundational (T009-T022)
3. Complete Phase 3: User Story 1 (T023-T035)
4. **STOP and VALIDATE**: Test chapter-scoped Q&A independently
5. Deploy backend, test on live Docusaurus site

### Task Counts

| Phase | Task Count | Parallelizable |
|-------|------------|----------------|
| Phase 1: Setup | 8 | 6 |
| Phase 2: Foundational | 14 | 8 |
| Phase 3: US1 | 13 | 0 (sequential) |
| Phase 4: US2 | 9 | 0 (sequential) |
| Phase 5: US3 | 8 | 0 (sequential) |
| Phase 6: US4 | 7 | 2 |
| Phase 7: Polish | 12 | 6 |
| **TOTAL** | **71** | **22** |

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Vercel AI SDK handles streaming via useChat hook
- Gemini 2.0 Flash provides fast, capable responses
- MVP (US1) is the minimum viable product for hackathon demo
