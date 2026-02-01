---
id: 0006
title: Implement Chapter Translation Feature
stage: green
date: 2026-01-29
surface: agent
model: claude-opus-4-5-20251101
feature: 004-chapter-translation
branch: 004-chapter-translation
user: architect
command: /sp.implement
labels: ["implementation", "translation", "urdu", "frontend", "backend"]
links:
  spec: specs/004-chapter-translation/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - backend/requirements.txt
  - backend/src/models/translation.py
  - backend/src/services/translator.py
  - backend/src/api/translate.py
  - backend/src/main.py
  - src/css/custom.css
  - src/components/ChapterTranslation/cache.ts
  - src/components/ChapterTranslation/TranslationButton.tsx
  - src/components/ChapterTranslation/styles.module.css
  - src/theme/DocItem/Layout/index.tsx
  - specs/004-chapter-translation/tasks.md
tests:
  - TypeScript type check passed (no new errors)
---

## Prompt

/sp.implement

## Response snapshot

Implemented the complete Chapter Translation Toggle feature across 7 phases (24 tasks, all completed):

**Backend (Python/FastAPI)**:
- `TranslationRequest` and `TranslationResponse` Pydantic models with validation
- `TranslatorService` with BeautifulSoup HTML parsing, placeholder-based code block preservation, comprehensive translation prompt with 40+ technical terms, and Groq LLM integration
- `/api/translate` POST endpoint with 400/500/504 error handling and 60s timeout
- Router registered in main.py

**Frontend (TypeScript/React)**:
- `cache.ts` — localStorage cache with 7-day expiry, LRU eviction on full storage
- `TranslationButton.tsx` — Full state machine (english/translating/urdu/error), scroll position preservation via percentage-based approach with requestAnimationFrame, AbortController for navigation cleanup, offline detection, auto-dismiss error notifications
- `styles.module.css` — Fixed positioning (bottom-left), spinner animation, error toast, responsive breakpoints
- Integrated into DocItem/Layout alongside existing FloatingChatButton

**Infrastructure**:
- Added beautifulsoup4 and lxml to backend/requirements.txt
- Added Noto Nastaliq Urdu Google Font and RTL CSS styles to custom.css
- Verified no z-index or DOM conflicts with existing RAG chatbot

## Outcome

- ✅ Impact: Complete translation feature — translate any chapter to Urdu with one click, instant toggle-back from cache, preserved scroll position, loading states, error handling, offline support
- 🧪 Tests: TypeScript type check passed (0 new errors; 3 pre-existing JSX namespace errors in unrelated components)
- 📁 Files: 10 files modified/created (4 backend, 5 frontend, 1 CSS)
- 🔁 Next prompts: Run /sp.git.commit_pr to commit and create PR; manual E2E testing with running backend
- 🧠 Reflection: All 24 tasks completed in single pass. TranslationButton component consolidated US1-US4 work since they all modify the same file. Backend uses placeholder approach for code blocks rather than per-node translation for better LLM output quality.

## Evaluation notes (flywheel)

- Failure modes observed: None — all phases completed successfully
- Graders run and results (PASS/FAIL): TypeScript typecheck PASS (no new errors)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Test with actual Groq API call on a real chapter to validate translation quality and prompt effectiveness
