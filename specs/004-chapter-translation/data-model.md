# Data Model: Chapter Translation Toggle

**Feature**: 004-chapter-translation
**Date**: 2026-01-29

## Entity Definitions

### Frontend Entities (TypeScript)

#### TranslationState

Represents the current translation status of a chapter.

```typescript
interface TranslationState {
  /** Current display language */
  language: 'english' | 'urdu';

  /** Whether translation is in progress */
  isTranslating: boolean;

  /** Error message if translation failed, null otherwise */
  error: string | null;

  /** Timestamp of last translation request */
  lastUpdated: number | null;
}
```

**State Transitions**:
```
initial → { language: 'english', isTranslating: false, error: null }
     ↓ (click translate)
translating → { language: 'english', isTranslating: true, error: null }
     ↓ (success)
translated → { language: 'urdu', isTranslating: false, error: null }
     ↓ (click restore)
english → { language: 'english', isTranslating: false, error: null }

// Error path
translating → { language: 'english', isTranslating: false, error: "message" }
```

#### ChapterTranslationCache

Stores original English content and translated Urdu content per chapter.

```typescript
interface ChapterTranslationCache {
  /** Unique chapter identifier (matches Docusaurus metadata.id) */
  chapterId: string;

  /** Original English HTML content */
  english: string;

  /** Translated Urdu HTML content (null if not yet translated) */
  urdu: string | null;

  /** Unix timestamp when translation was created */
  translatedAt: number | null;

  /** Content version hash (optional, for cache invalidation) */
  contentHash?: string;
}
```

**localStorage Key Pattern**: `translation_cache_${chapterId}`

**Example**:
```json
{
  "chapterId": "module-1-ros2/nodes-topics",
  "english": "<h1>Nodes and Topics</h1><p>In ROS 2...</p>",
  "urdu": "<h1>نوڈز اور ٹاپکس</h1><p>ROS 2 میں...</p>",
  "translatedAt": 1706534400000,
  "contentHash": "a1b2c3d4"
}
```

### Backend Entities (Python/Pydantic)

#### TranslationRequest

Request body for the translation endpoint.

```python
from pydantic import BaseModel, Field

class TranslationRequest(BaseModel):
    """Request to translate chapter content."""

    chapter_id: str = Field(
        ...,
        description="Unique chapter identifier",
        examples=["module-1-ros2/nodes-topics"]
    )

    chapter_title: str = Field(
        ...,
        description="Human-readable chapter title",
        examples=["Nodes and Topics"]
    )

    content: str = Field(
        ...,
        description="HTML content to translate",
        min_length=1,
        max_length=500000  # ~500KB max
    )

    target_language: str = Field(
        default="urdu",
        description="Target language code",
        pattern="^(urdu|ur)$"
    )

    class Config:
        populate_by_name = True
```

#### TranslationResponse

Response body from the translation endpoint.

```python
class TranslationResponse(BaseModel):
    """Response containing translated content."""

    success: bool = Field(
        ...,
        description="Whether translation succeeded"
    )

    chapter_id: str = Field(
        ...,
        description="Chapter identifier (echoed from request)"
    )

    translated_content: str | None = Field(
        default=None,
        description="Translated HTML content"
    )

    source_language: str = Field(
        default="english",
        description="Source language of content"
    )

    target_language: str = Field(
        default="urdu",
        description="Target language of translation"
    )

    error: str | None = Field(
        default=None,
        description="Error message if translation failed"
    )

    metadata: dict | None = Field(
        default=None,
        description="Additional metadata (tokens used, latency, etc.)"
    )
```

#### TranslationChunk (for streaming, optional)

```python
class TranslationChunk(BaseModel):
    """Chunk of translated content for streaming response."""

    chunk_index: int
    content: str
    is_final: bool = False
```

## Validation Rules

### TranslationRequest Validation

| Field | Rule | Error Message |
|-------|------|---------------|
| chapter_id | Non-empty string | "Chapter ID is required" |
| content | 1-500,000 characters | "Content must be between 1 and 500,000 characters" |
| target_language | Must be "urdu" or "ur" | "Only Urdu translation is supported" |

### ChapterTranslationCache Validation (Frontend)

| Field | Rule | Error Message |
|-------|------|---------------|
| chapterId | Matches current page | Cache miss, fetch fresh |
| english | Non-empty | Invalid cache, clear and refetch |
| translatedAt | Within 7 days | Cache expired, suggest re-translate |

## Relationships

```
┌─────────────────────┐
│  DocItem/Layout     │
│  (Docusaurus)       │
└──────────┬──────────┘
           │ provides chapterId, chapterTitle
           ▼
┌─────────────────────┐      ┌─────────────────────┐
│ TranslationButton   │◄────►│ TranslationState    │
│ (React Component)   │      │ (React State)       │
└──────────┬──────────┘      └─────────────────────┘
           │ reads/writes
           ▼
┌─────────────────────┐
│ ChapterTranslation  │
│ Cache (localStorage)│
└──────────┬──────────┘
           │ cache miss → API call
           ▼
┌─────────────────────┐      ┌─────────────────────┐
│ /api/translate      │─────►│ TranslatorService   │
│ (FastAPI endpoint)  │      │ (Groq/Gemini)       │
└─────────────────────┘      └─────────────────────┘
```

## State Machine

### Button State Machine

```
                    ┌──────────────┐
                    │              │
         ┌──────────►   ENGLISH    ├──────────┐
         │          │              │          │
         │          └──────────────┘          │
         │                 │                  │
         │                 │ click            │
         │                 │ (no cache)       │
         │                 ▼                  │
         │          ┌──────────────┐          │
         │          │              │          │
         │          │ TRANSLATING  │          │
         │          │              │          │
         │          └──────┬───────┘          │
         │                 │                  │
         │       ┌─────────┴─────────┐        │
         │       │                   │        │
         │    success              error      │
         │       │                   │        │
         │       ▼                   │        │
         │ ┌──────────────┐         │        │
         │ │              │         │        │
  click  │ │    URDU      │         │        │
(cached) │ │              │         │        │
         │ └──────────────┘         │        │
         │       │                  │        │
         │       │ click            │        │
         │       │                  │        │
         └───────┴──────────────────┴────────┘
```

## Cache Strategy

### Cache Operations

```typescript
// WRITE: After successful translation
function cacheTranslation(
  chapterId: string,
  english: string,
  urdu: string
): void {
  const cache: ChapterTranslationCache = {
    chapterId,
    english,
    urdu,
    translatedAt: Date.now(),
  };
  localStorage.setItem(
    `translation_cache_${chapterId}`,
    JSON.stringify(cache)
  );
}

// READ: Before making API call
function getCachedTranslation(
  chapterId: string
): ChapterTranslationCache | null {
  const raw = localStorage.getItem(`translation_cache_${chapterId}`);
  if (!raw) return null;

  try {
    const cache = JSON.parse(raw) as ChapterTranslationCache;
    // Validate cache is not expired (7 days)
    const maxAge = 7 * 24 * 60 * 60 * 1000;
    if (cache.translatedAt && Date.now() - cache.translatedAt > maxAge) {
      localStorage.removeItem(`translation_cache_${chapterId}`);
      return null;
    }
    return cache;
  } catch {
    return null;
  }
}

// CLEAR: On cache invalidation
function clearTranslationCache(chapterId?: string): void {
  if (chapterId) {
    localStorage.removeItem(`translation_cache_${chapterId}`);
  } else {
    // Clear all translation caches
    Object.keys(localStorage)
      .filter(key => key.startsWith('translation_cache_'))
      .forEach(key => localStorage.removeItem(key));
  }
}
```

## Database Schema

> **Note**: This feature does not require database persistence. All caching is client-side via localStorage. Backend is stateless.

If future requirements need server-side caching:

```sql
-- Optional: For server-side translation cache
CREATE TABLE chapter_translations (
  id SERIAL PRIMARY KEY,
  chapter_id VARCHAR(255) NOT NULL,
  content_hash VARCHAR(64) NOT NULL,
  source_language VARCHAR(10) DEFAULT 'english',
  target_language VARCHAR(10) NOT NULL,
  translated_content TEXT NOT NULL,
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  UNIQUE(chapter_id, content_hash, target_language)
);

CREATE INDEX idx_chapter_translations_lookup
ON chapter_translations(chapter_id, target_language);
```
