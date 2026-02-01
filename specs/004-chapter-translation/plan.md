# Implementation Plan: Chapter Translation Toggle

**Branch**: `004-chapter-translation` | **Date**: 2026-01-29 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/004-chapter-translation/spec.md`

## Summary

Implement a Translate button that toggles book chapter content between English and Urdu in-place. The feature uses the existing Groq LLM service (Llama 3.3 70B) for translation, preserves code blocks in English, supports RTL rendering for Urdu text, and caches both original and translated content in localStorage for instant toggle-back. Integration is achieved by adding a TranslationButton component to the existing DocItem Layout theme swizzle.

## Technical Context

**Language/Version**: TypeScript 5.6 (frontend), Python 3.10+ (backend)
**Primary Dependencies**: React 19, Docusaurus 3.9.2, FastAPI, Groq SDK, BeautifulSoup4
**Storage**: localStorage (browser-side caching), no backend persistence
**Testing**: Jest (frontend), pytest (backend)
**Target Platform**: Web browsers (Chrome, Firefox, Safari, Edge), Docusaurus site
**Project Type**: Web application (frontend + backend)
**Performance Goals**: <100ms toggle-back latency (cached), <30s initial translation
**Constraints**: 500KB max content size, 7-day cache expiry, RTL support required
**Scale/Scope**: ~25 chapters, 1 target language (Urdu), single-user localStorage

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Requirement | Status | Notes |
|-----------|-------------|--------|-------|
| I. Docusaurus-First | Content in MDX, deployable to GitHub Pages | ✅ PASS | Translation is DOM-level, MDX source unchanged |
| II. Spec-Driven Development | Feature has spec, plan, tasks | ✅ PASS | spec.md complete, plan.md in progress |
| III. RAG-First Content | Content optimized for RAG retrieval | ✅ PASS | RAG operates on English source, unaffected |
| IV. Modular Content | 4-module structure maintained | ✅ PASS | No content structure changes |
| V. Code-Content Parity | Code examples executable | ✅ PASS | Code blocks preserved in English |
| VI. Accessibility-First | Translation button required | ✅ PASS | This feature implements that requirement |
| VII. Security | No secrets committed, sanitized inputs | ✅ PASS | Content sanitized via BeautifulSoup |

**Pre-Phase 0 Gate**: ✅ PASSED
**Post-Phase 1 Gate**: ✅ PASSED (no violations introduced)

## Project Structure

### Documentation (this feature)

```text
specs/004-chapter-translation/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output - COMPLETE
├── data-model.md        # Phase 1 output - COMPLETE
├── quickstart.md        # Phase 1 output - COMPLETE
├── contracts/           # Phase 1 output - COMPLETE
│   └── translate-api.yaml
├── checklists/
│   └── requirements.md  # Requirements checklist
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── api/
│   │   ├── chat.py           # Existing RAG chat endpoint
│   │   └── translate.py      # NEW: Translation endpoint
│   ├── models/
│   │   ├── chat.py           # Existing chat models
│   │   └── translation.py    # NEW: TranslationRequest/Response
│   ├── services/
│   │   ├── groq_llm.py       # Existing Groq service (reused)
│   │   └── translator.py     # NEW: Translation service with HTML parsing
│   └── main.py               # Register translate router
├── tests/
│   └── test_translate.py     # NEW: Translation endpoint tests
└── requirements.txt          # Add beautifulsoup4, lxml

src/                          # Docusaurus frontend
├── components/
│   ├── ChapterChat/          # Existing RAG chatbot
│   └── ChapterTranslation/   # NEW: Translation feature
│       ├── TranslationButton.tsx
│       ├── styles.module.css
│       └── cache.ts          # localStorage cache utilities
├── theme/
│   └── DocItem/Layout/
│       └── index.tsx         # Modified: Add TranslationButton
└── css/
    └── custom.css            # Modified: Add Urdu font, RTL styles
```

**Structure Decision**: Web application (Option 2) - extends existing backend/frontend structure with new translation-specific modules. No new top-level directories required; all changes fit within existing architecture.

## Component Architecture

### Frontend Components

```
┌─────────────────────────────────────────────────────────────┐
│                    DocItem/Layout                            │
│  (Theme swizzle - integration point)                         │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────────┐  ┌──────────────────────────────────┐  │
│  │ TranslationButton│  │       FloatingChatButton         │  │
│  │                 │  │       (existing)                 │  │
│  └────────┬────────┘  └──────────────────────────────────┘  │
└───────────┼─────────────────────────────────────────────────┘
            │
            ▼
┌─────────────────────────────────────────────────────────────┐
│              TranslationButton (new component)               │
├─────────────────────────────────────────────────────────────┤
│  Props: chapterId, chapterTitle                             │
│  State: language, isTranslating, error                      │
│                                                             │
│  ┌─────────────┐    ┌──────────────┐    ┌───────────────┐  │
│  │ useTranslate│───►│ cache.ts     │───►│ localStorage  │  │
│  │ hook        │    │ utilities    │    │               │  │
│  └─────────────┘    └──────────────┘    └───────────────┘  │
│         │                                                   │
│         ▼                                                   │
│  ┌─────────────────────────────────────────────────────┐   │
│  │            POST /api/translate                       │   │
│  │            (cache miss only)                         │   │
│  └─────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

### Backend Services

```
┌─────────────────────────────────────────────────────────────┐
│                 POST /api/translate                          │
│                 (translate.py router)                        │
├─────────────────────────────────────────────────────────────┤
│                          │                                   │
│                          ▼                                   │
│  ┌─────────────────────────────────────────────────────┐   │
│  │              TranslatorService                       │   │
│  │              (translator.py)                         │   │
│  ├─────────────────────────────────────────────────────┤   │
│  │  1. Parse HTML with BeautifulSoup                    │   │
│  │  2. Extract translatable text (skip <pre>, <code>)   │   │
│  │  3. Build translation prompt                         │   │
│  │  4. Call Groq LLM                                    │   │
│  │  5. Reassemble HTML with translated text             │   │
│  │  6. Return TranslationResponse                       │   │
│  └────────────────────────┬────────────────────────────┘   │
│                           │                                  │
│                           ▼                                  │
│  ┌─────────────────────────────────────────────────────┐   │
│  │              GroqService (existing)                  │   │
│  │              groq_llm.py                             │   │
│  └─────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

## Data Flow

### Translation Request Flow

```
User clicks "Translate to Urdu"
         │
         ▼
┌──────────────────────────┐
│ Check localStorage cache │
│ key: translation_cache_  │
│      ${chapterId}        │
└────────────┬─────────────┘
             │
    ┌────────┴────────┐
    │                 │
  Cache Hit        Cache Miss
    │                 │
    ▼                 ▼
┌──────────┐   ┌─────────────────────┐
│ Return   │   │ Extract DOM content │
│ cached   │   │ .querySelector      │
│ urdu     │   │ ('.markdown')       │
└────┬─────┘   └──────────┬──────────┘
     │                    │
     │                    ▼
     │         ┌─────────────────────┐
     │         │ POST /api/translate │
     │         │ {chapter_id,        │
     │         │  chapter_title,     │
     │         │  content}           │
     │         └──────────┬──────────┘
     │                    │
     │                    ▼
     │         ┌─────────────────────┐
     │         │ Save to localStorage│
     │         │ (english + urdu)    │
     │         └──────────┬──────────┘
     │                    │
     └────────┬───────────┘
              │
              ▼
┌─────────────────────────────┐
│ Update DOM with translated  │
│ content + RTL attributes    │
└─────────────────────────────┘
```

### Toggle Back Flow (Instant)

```
User clicks "Show Original"
         │
         ▼
┌──────────────────────────┐
│ Read localStorage cache  │
│ Get cached.english       │
└────────────┬─────────────┘
             │
             ▼
┌─────────────────────────────┐
│ Update DOM with original    │
│ content + LTR attributes    │
│ (NO API call)               │
└─────────────────────────────┘
```

## Key Implementation Details

### 1. Translation Prompt (Backend)

```python
TRANSLATION_PROMPT = """
Translate the following HTML content from English to Urdu.

CRITICAL RULES:
1. Preserve ALL HTML tags exactly as they are
2. Do NOT translate text inside <pre> or <code> tags
3. Keep these technical terms in English: ROS, ROS2, Gazebo, Isaac Sim,
   NVIDIA, URDF, SLAM, Nav2, lidar, LiDAR, IMU, API, SDK, Python, C++
4. Maintain the same heading hierarchy (h1, h2, h3, etc.)
5. Preserve all class names, id attributes, and other HTML attributes
6. Return ONLY the translated HTML, no explanations

Content to translate:
{content}
"""
```

### 2. Cache Schema (Frontend)

```typescript
interface ChapterTranslationCache {
  chapterId: string;
  english: string;
  urdu: string | null;
  translatedAt: number | null;
}

// Key: `translation_cache_${chapterId}`
// Expiry: 7 days (604800000 ms)
```

### 3. RTL Styling

```css
.urdu-content {
  direction: rtl;
  font-family: 'Noto Nastaliq Urdu', serif;
  line-height: 2;
  text-align: right;
}

.urdu-content pre,
.urdu-content code {
  direction: ltr;
  text-align: left;
}
```

## Complexity Tracking

> No constitution violations. Feature uses smallest viable implementation.

| Aspect | Decision | Rationale |
|--------|----------|-----------|
| Caching | localStorage only | No backend persistence needed; client-side sufficient |
| LLM | Reuse Groq | Already integrated; no new service required |
| Streaming | Not implemented | 30s timeout acceptable for MVP; can add later |
| Languages | Urdu only | Spec requires only Urdu; extensible pattern established |

## Artifacts Generated

| Artifact | Path | Status |
|----------|------|--------|
| Research | `specs/004-chapter-translation/research.md` | ✅ Complete |
| Data Model | `specs/004-chapter-translation/data-model.md` | ✅ Complete |
| API Contract | `specs/004-chapter-translation/contracts/translate-api.yaml` | ✅ Complete |
| Quickstart | `specs/004-chapter-translation/quickstart.md` | ✅ Complete |
| Plan | `specs/004-chapter-translation/plan.md` | ✅ Complete |
| Tasks | `specs/004-chapter-translation/tasks.md` | ⏳ Next: `/sp.tasks` |

## Next Steps

1. Run `/sp.tasks` to generate implementation tasks from this plan
2. Implement backend translation service (translator.py)
3. Implement frontend TranslationButton component
4. Integrate into DocItem/Layout theme swizzle
5. Test across all chapter types
6. Run `/sp.git.commit_pr` to commit and create PR
