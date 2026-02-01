# Quickstart: Chapter Translation Toggle

**Feature**: 004-chapter-translation
**Estimated Setup Time**: 15 minutes

## Prerequisites

- Node.js 18+ installed
- Python 3.10+ installed
- Groq API key (already configured for RAG chatbot)
- Project cloned and dependencies installed

## Quick Setup

### 1. Verify Existing Dependencies

The translation feature reuses existing infrastructure. Verify these are working:

```bash
# Frontend (Docusaurus)
cd "C:\Users\Dell\Desktop\New folder (2)\book-rag\book"
npm run start  # Should start Docusaurus dev server on :3000

# Backend (FastAPI)
cd backend
pip install -r requirements.txt  # If not already done
python -m uvicorn src.main:app --reload  # Should start on :8000
```

### 2. Add BeautifulSoup4 (if not installed)

```bash
cd backend
pip install beautifulsoup4 lxml
```

Add to `requirements.txt`:
```
beautifulsoup4>=4.12.0
lxml>=5.0.0
```

### 3. Add Urdu Font to Docusaurus

Add to `src/css/custom.css`:

```css
/* Urdu font support */
@import url('https://fonts.googleapis.com/css2?family=Noto+Nastaliq+Urdu:wght@400;700&display=swap');

/* RTL support for translated content */
.urdu-content {
  direction: rtl;
  font-family: 'Noto Nastaliq Urdu', 'Jameel Noori Nastaleeq', serif;
  line-height: 2;
  text-align: right;
}

/* Keep code blocks LTR */
.urdu-content pre,
.urdu-content code {
  direction: ltr;
  text-align: left;
  font-family: var(--ifm-font-family-monospace);
}
```

### 4. Create Translation Component

Create `src/components/ChapterTranslation/TranslationButton.tsx`:

```typescript
import React, { useState, useCallback } from 'react';
import styles from './styles.module.css';

interface Props {
  chapterId: string;
  chapterTitle: string;
}

export default function TranslationButton({ chapterId, chapterTitle }: Props) {
  const [language, setLanguage] = useState<'english' | 'urdu'>('english');
  const [isTranslating, setIsTranslating] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const handleToggle = useCallback(async () => {
    // Implementation will go here
    console.log('Toggle translation for:', chapterId);
  }, [chapterId]);

  return (
    <button
      className={styles.translationButton}
      onClick={handleToggle}
      disabled={isTranslating}
      aria-label={language === 'english' ? 'Translate to Urdu' : 'Show Original'}
    >
      {isTranslating ? 'Translating...' : (
        language === 'english' ? '🌐 Translate to Urdu' : '🔄 Show Original'
      )}
    </button>
  );
}
```

### 5. Add Translation Button to DocItem Layout

Modify `src/theme/DocItem/Layout/index.tsx`:

```typescript
import React from 'react';
import Layout from '@theme-original/DocItem/Layout';
import type LayoutType from '@theme/DocItem/Layout';
import type { WrapperProps } from '@docusaurus/types';
import { useDoc } from '@docusaurus/plugin-content-docs/client';
import FloatingChatButton from '@site/src/components/ChapterChat/FloatingButton';
import TranslationButton from '@site/src/components/ChapterTranslation/TranslationButton';

type Props = WrapperProps<typeof LayoutType>;

export default function LayoutWrapper(props: Props): React.JSX.Element {
  const { metadata } = useDoc();
  const chapterId = metadata.id;
  const chapterTitle = metadata.title;

  return (
    <>
      <Layout {...props} />
      <TranslationButton chapterId={chapterId} chapterTitle={chapterTitle} />
      <FloatingChatButton chapterId={chapterId} chapterTitle={chapterTitle} />
    </>
  );
}
```

### 6. Create Backend Endpoint

Create `backend/src/api/translate.py`:

```python
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field

router = APIRouter(prefix="/api", tags=["translation"])

class TranslationRequest(BaseModel):
    chapter_id: str
    chapter_title: str
    content: str = Field(..., min_length=1, max_length=500000)
    target_language: str = "urdu"

class TranslationResponse(BaseModel):
    success: bool
    chapter_id: str
    translated_content: str | None = None
    error: str | None = None

@router.post("/translate", response_model=TranslationResponse)
async def translate_chapter(request: TranslationRequest):
    # Implementation will go here
    return TranslationResponse(
        success=True,
        chapter_id=request.chapter_id,
        translated_content="<p>Translation placeholder</p>"
    )
```

Register in `backend/src/main.py`:

```python
from src.api import translate

app.include_router(translate.router)
```

### 7. Verify Setup

```bash
# Test backend endpoint
curl -X POST http://localhost:8000/api/translate \
  -H "Content-Type: application/json" \
  -d '{"chapter_id": "test", "chapter_title": "Test", "content": "<p>Hello</p>"}'

# Expected response:
# {"success": true, "chapter_id": "test", "translated_content": "<p>Translation placeholder</p>", "error": null}
```

## Development Workflow

### Running Both Servers

```bash
# Terminal 1: Frontend
npm run start

# Terminal 2: Backend
cd backend && python -m uvicorn src.main:app --reload
```

### Testing Translation Flow

1. Open http://localhost:3000/docs/module-1-ros2/nodes-topics
2. Click the "Translate to Urdu" button
3. Verify loading state appears
4. Verify content updates to Urdu
5. Click "Show Original" to restore English
6. Verify instant restore (no loading)

## Configuration

### Environment Variables

No new environment variables required. Uses existing:

```env
# Already configured for RAG chatbot
GROQ_API_KEY=your-groq-key
```

### Optional Configuration

Add to `.env` if needed:

```env
# Translation-specific (optional)
TRANSLATION_TIMEOUT_MS=30000
TRANSLATION_MAX_TOKENS=8000
```

## Troubleshooting

### "Translation failed" error

1. Check Groq API key is valid
2. Check backend logs for detailed error
3. Verify content length is under 500KB

### RTL text not displaying correctly

1. Verify Noto Nastaliq Urdu font loaded (Network tab)
2. Check `urdu-content` class is applied
3. Verify `dir="rtl"` attribute on container

### Cache not working

1. Check localStorage in DevTools → Application
2. Verify key format: `translation_cache_${chapterId}`
3. Clear cache and retry: `localStorage.clear()`

## Next Steps

After quickstart setup:

1. Implement full translation logic in backend
2. Add proper error handling and loading states
3. Implement localStorage caching
4. Add scroll position preservation
5. Test across all chapter types
