# Implementation Plan: Context-Aware RAG Chatbot

**Branch**: `003-context-rag-chatbot` | **Date**: 2026-01-25 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/003-context-rag-chatbot/spec.md`

## Summary

Build a context-aware RAG chatbot for the "Physical AI & Humanoid Robotics" textbook that enforces strict chapter boundaries when answering student questions. The system uses **Gemini 2.0 Flash** for LLM responses, **Vercel AI SDK** (`@ai-sdk/google`) for the chat UI with streaming, Qdrant Cloud for vector storage with chapter-level metadata filtering, FastAPI backend for RAG orchestration, and Google's text-embedding-004 for embeddings.

## Technical Context

**Language/Version**: Python 3.11+ (backend), TypeScript 5.x (frontend)
**Primary Dependencies**: FastAPI 0.109+, google-generativeai, Qdrant Client 1.7+, ai (Vercel AI SDK), @ai-sdk/google
**Storage**: Qdrant Cloud (vectors), Neon Serverless Postgres (metadata/sessions)
**Testing**: pytest + pytest-asyncio (backend), Vitest (frontend)
**Target Platform**: GitHub Pages (static frontend), Cloud-hosted FastAPI (backend)
**Project Type**: Web application (Vercel AI SDK frontend + FastAPI backend)
**Performance Goals**: <2s time-to-first-token, 100 concurrent sessions
**Constraints**: Free-tier compatible (Qdrant, Neon), chapter-scoped retrieval only
**Scale/Scope**: ~50 chapters, ~500 content chunks, hackathon evaluation

## Constitution Check

*GATE: All checks passed*

| Principle | Status | Notes |
|-----------|--------|-------|
| I. Docusaurus-First | ✅ Pass | Chat UI embedded as React component in Docusaurus |
| II. Spec-Driven Development | ✅ Pass | Following `/sp.specify` → `/sp.plan` → `/sp.tasks` workflow |
| III. RAG-First Content | ✅ Pass | Core feature - chapters already structured for retrieval |
| IV. Modular Architecture | ✅ Pass | Chapter metadata preserves module structure |
| V. Code-Content Parity | ✅ Pass | Backend code will be tested, Vercel AI SDK provides validated streaming |
| VI. Accessibility-First | ✅ Pass | Custom accessible UI with ARIA labels |
| VII. Security | ✅ Pass | Session-only storage, sanitized queries, no persistent user data |

**Stack Compliance**:
- Docusaurus ✅ (existing)
- FastAPI ✅ (backend)
- Neon Postgres ✅ (metadata storage)
- Qdrant Cloud ✅ (vector storage)
- Gemini 2.0 Flash ✅ (LLM for chat responses)
- Vercel AI SDK ✅ (streaming chat UI)

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         DOCUSAURUS STATIC SITE                          │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │  Chapter Page (e.g., /module-1-ros2/nodes-topics)               │   │
│  │  ┌──────────────────────┐  ┌─────────────────────────────────┐  │   │
│  │  │   Chapter Content    │  │   Vercel AI SDK Chat Widget     │  │   │
│  │  │   (MDX rendered)     │  │  ┌───────────────────────────┐  │  │   │
│  │  │                      │  │  │ 📖 Chapter 1.1 Context    │  │  │   │
│  │  │   [User selects      │  │  │ ─────────────────────────│  │  │   │
│  │  │    text here]        │──┼──│ User: What is a node?    │  │  │   │
│  │  │                      │  │  │ Bot: A node is... [§1.2] │  │  │   │
│  │  │                      │  │  │ [Streaming via useChat]  │  │  │   │
│  │  └──────────────────────┘  │  └───────────────────────────┘  │  │   │
│  └─────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────┘
                                      │
                                      │ POST /api/chat (streaming)
                                      ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                           FASTAPI BACKEND                               │
│  ┌─────────────────────┐  ┌─────────────────────────────────────────┐  │
│  │  /api/chat          │  │         RAG Pipeline                    │  │
│  │  (SSE streaming)    │  │  ┌─────────────────────────────────┐    │  │
│  │                     │  │  │ 1. Embed query (Google)         │    │  │
│  └─────────────────────┘  │  │ 2. Search Qdrant (chapter filter)│   │  │
│                           │  │ 3. Build context prompt          │    │  │
│  ┌─────────────────────┐  │  │ 4. Stream from Gemini 2.0 Flash │    │  │
│  │  Gemini 2.0 Flash   │──│  └─────────────────────────────────┘    │  │
│  │  (LLM Generation)   │  │                                         │  │
│  └─────────────────────┘  │                                         │  │
└─────────────────────────────────────────────────────────────────────────┘
         │                       │                      │
         ▼                       ▼                      ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────────────┐
│  Neon Postgres  │    │  Google AI API  │    │     Qdrant Cloud        │
│  - Chapter meta │    │  - Embeddings   │    │  - Chunk vectors        │
│                 │    │  - Gemini LLM   │    │  - Chapter metadata     │
└─────────────────┘    └─────────────────┘    └─────────────────────────┘
```

## Why Gemini + Vercel AI SDK?

| Feature | Benefit |
|---------|---------|
| Gemini 2.0 Flash | Fast, capable LLM with excellent instruction following |
| Single Google API | Both embeddings AND LLM from same provider - simpler setup |
| Vercel AI SDK | `useChat` hook handles streaming, loading states, error handling |
| @ai-sdk/google | Native Gemini support with streaming |
| Custom UI control | Full control over chat appearance to match Docusaurus theme |

**Trade-off**: More frontend code than ChatKit, but full control and single API key.

## Data Model

### Qdrant Collection: `textbook_chunks`

```python
# Vector collection configuration
collection_config = {
    "name": "textbook_chunks",
    "vectors": {
        "size": 768,  # Google text-embedding-004 dimension
        "distance": "Cosine"
    }
}

# Point structure
point = {
    "id": "uuid-v4",
    "vector": [0.1, 0.2, ...],  # 768-dimensional embedding
    "payload": {
        "chapter_id": "module-1-ros2/nodes-topics",  # URL slug, filterable
        "chapter_title": "ROS 2 Nodes and Topics",
        "module": "module-1-ros2",
        "section_heading": "Understanding Publisher Nodes",
        "content": "A publisher node in ROS 2...",
        "chunk_index": 3,  # Order within chapter
        "word_count": 487,
        "url": "/docs/module-1-ros2/nodes-topics#understanding-publisher-nodes"
    }
}
```

### Neon Postgres Schema

```sql
-- Chapter metadata for fast lookup
CREATE TABLE chapters (
    id TEXT PRIMARY KEY,              -- URL slug (e.g., "module-1-ros2/nodes-topics")
    title TEXT NOT NULL,
    module TEXT NOT NULL,
    chapter_order INTEGER NOT NULL,
    section_count INTEGER DEFAULT 0,
    chunk_count INTEGER DEFAULT 0,
    last_indexed_at TIMESTAMP,
    created_at TIMESTAMP DEFAULT NOW()
);

-- Index for chapter lookup
CREATE INDEX idx_chapters_module ON chapters(module);
```

### TypeScript Interfaces (Frontend)

```typescript
// Chat message structure (Vercel AI SDK compatible)
interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
}

// Chat request with chapter context
interface ChatRequestBody {
  messages: Message[];
  chapterId: string;
  chapterTitle: string;
  selectedText?: string;
}

// Props for the chat component
interface ChapterChatProps {
  chapterId: string;
  chapterTitle: string;
}
```

## API Contracts

### POST /api/chat (Server-Sent Events)

Streams chat responses with RAG context.

**Request**:
```json
{
  "messages": [
    {"role": "user", "content": "What is a ROS 2 node?"}
  ],
  "chapterId": "module-1-ros2/nodes-topics",
  "chapterTitle": "ROS 2 Nodes and Topics",
  "selectedText": null
}
```

**Response** (SSE stream - Vercel AI SDK format):
```
0:"A "
0:"ROS "
0:"2 "
0:"node "
0:"is..."
e:{"finishReason":"stop"}
d:{"finishReason":"stop"}
```

### GET /api/chapters

**Response**:
```json
{
  "chapters": [
    {
      "id": "module-1-ros2/nodes-topics",
      "title": "ROS 2 Nodes and Topics",
      "module": "module-1-ros2",
      "chunkCount": 12
    }
  ]
}
```

### POST /api/index (Admin - Indexing Endpoint)

**Request**:
```json
{
  "chapterId": "module-1-ros2/nodes-topics",
  "content": "# ROS 2 Nodes and Topics\n\n...",
  "reindex": false
}
```

**Response**:
```json
{
  "success": true,
  "chunksCreated": 12,
  "chapterId": "module-1-ros2/nodes-topics"
}
```

## Component Design

### Backend Components

#### 1. Chat Endpoint (`backend/src/api/chat.py`)

```python
from fastapi import APIRouter
from fastapi.responses import StreamingResponse
import google.generativeai as genai

router = APIRouter()

@router.post("/api/chat")
async def chat(request: ChatRequest):
    """
    RAG chat endpoint with Gemini streaming.
    1. Retrieve relevant chunks from Qdrant (chapter-filtered)
    2. Build context prompt with citations
    3. Stream response from Gemini 2.0 Flash
    """
    # Get relevant context
    chunks = await rag_service.retrieve(
        query=request.messages[-1].content,
        chapter_id=request.chapter_id
    )

    # Build system prompt with context
    system_prompt = build_chapter_prompt(
        chapter_title=request.chapter_title,
        context_chunks=chunks,
        selected_text=request.selected_text
    )

    # Stream from Gemini
    return StreamingResponse(
        stream_gemini_response(system_prompt, request.messages),
        media_type="text/event-stream"
    )
```

#### 2. RAG Service (`backend/src/services/rag.py`)

```python
class RAGService:
    """Retrieval-augmented generation with chapter filtering."""

    async def retrieve(
        self,
        query: str,
        chapter_id: str,
        top_k: int = 5
    ) -> list[RetrievedChunk]:
        """
        Retrieve relevant chunks from ONLY the specified chapter.
        Uses Qdrant filtered search with chapter_id payload filter.
        """
        # Embed query
        embedding = await self.embedding_service.embed_text(query)

        # Search with chapter filter
        results = await self.qdrant.search(
            collection_name="textbook_chunks",
            query_vector=embedding,
            query_filter=Filter(
                must=[FieldCondition(
                    key="chapter_id",
                    match=MatchValue(value=chapter_id)
                )]
            ),
            limit=top_k
        )

        return [self._format_chunk(hit) for hit in results]
```

#### 3. Gemini Service (`backend/src/services/gemini.py`)

```python
import google.generativeai as genai

class GeminiService:
    """Gemini 2.0 Flash integration for chat responses."""

    MODEL = "gemini-2.0-flash-exp"

    def __init__(self):
        genai.configure(api_key=os.environ["GOOGLE_API_KEY"])
        self.model = genai.GenerativeModel(self.MODEL)

    async def stream_response(
        self,
        system_prompt: str,
        messages: list[dict]
    ) -> AsyncGenerator[str, None]:
        """Stream response tokens from Gemini."""
        chat = self.model.start_chat(history=self._format_history(messages))
        response = await chat.send_message_async(
            messages[-1]["content"],
            stream=True
        )

        async for chunk in response:
            if chunk.text:
                yield chunk.text
```

#### 4. Embedding Service (`backend/src/services/embedding.py`)

```python
import google.generativeai as genai

class EmbeddingService:
    """Google text-embedding-004 integration."""

    MODEL = "text-embedding-004"
    DIMENSION = 768

    def __init__(self):
        genai.configure(api_key=os.environ["GOOGLE_API_KEY"])

    async def embed_text(self, text: str) -> list[float]:
        """Generate embedding for a single text."""
        result = genai.embed_content(
            model=f"models/{self.MODEL}",
            content=text,
            task_type="retrieval_query"
        )
        return result["embedding"]

    async def embed_batch(self, texts: list[str]) -> list[list[float]]:
        """Batch embedding for indexing."""
        result = genai.embed_content(
            model=f"models/{self.MODEL}",
            content=texts,
            task_type="retrieval_document"
        )
        return result["embedding"]
```

### Frontend Components (Vercel AI SDK)

#### 1. ChapterChat (`src/components/ChapterChat/index.tsx`)

```typescript
import { useChat } from 'ai/react';
import styles from './styles.module.css';

interface ChapterChatProps {
  chapterId: string;
  chapterTitle: string;
}

export function ChapterChat({ chapterId, chapterTitle }: ChapterChatProps) {
  const [selectedText, setSelectedText] = useState<string | null>(null);

  const { messages, input, handleInputChange, handleSubmit, isLoading } = useChat({
    api: process.env.CHAT_API_URL + '/api/chat',
    body: {
      chapterId,
      chapterTitle,
      selectedText,
    },
  });

  return (
    <div className={styles.chatContainer}>
      <div className={styles.contextBadge}>
        📖 {chapterTitle}
      </div>

      <div className={styles.messages}>
        {messages.map((m) => (
          <div key={m.id} className={styles[m.role]}>
            {m.content}
          </div>
        ))}
      </div>

      <form onSubmit={handleSubmit} className={styles.inputForm}>
        <input
          value={input}
          onChange={handleInputChange}
          placeholder="Ask about this chapter..."
          disabled={isLoading}
        />
        <button type="submit" disabled={isLoading}>
          {isLoading ? '...' : 'Send'}
        </button>
      </form>
    </div>
  );
}
```

#### 2. FloatingChatButton (`src/components/ChapterChat/FloatingButton.tsx`)

```typescript
import { useState } from 'react';
import { ChapterChat } from './index';
import styles from './styles.module.css';

interface FloatingChatButtonProps {
  chapterId: string;
  chapterTitle: string;
}

export function FloatingChatButton({ chapterId, chapterTitle }: FloatingChatButtonProps) {
  const [isOpen, setIsOpen] = useState(false);

  return (
    <>
      <button
        className={styles.floatingButton}
        onClick={() => setIsOpen(!isOpen)}
        aria-label={isOpen ? "Close chat" : "Open chapter assistant"}
      >
        {isOpen ? '✕' : '💬'}
      </button>

      {isOpen && (
        <div className={styles.chatPanel}>
          <ChapterChat chapterId={chapterId} chapterTitle={chapterTitle} />
        </div>
      )}
    </>
  );
}
```

#### 3. Styles (`src/components/ChapterChat/styles.module.css`)

```css
.chatContainer {
  display: flex;
  flex-direction: column;
  height: 100%;
  background: var(--ifm-background-color);
}

.contextBadge {
  padding: 12px 16px;
  background: var(--ifm-color-primary-lighter);
  border-bottom: 1px solid var(--ifm-color-emphasis-300);
  font-weight: 600;
  font-size: 0.875rem;
}

.messages {
  flex: 1;
  overflow-y: auto;
  padding: 16px;
  display: flex;
  flex-direction: column;
  gap: 12px;
}

.user {
  align-self: flex-end;
  background: var(--ifm-color-primary);
  color: white;
  padding: 8px 12px;
  border-radius: 12px 12px 0 12px;
  max-width: 80%;
}

.assistant {
  align-self: flex-start;
  background: var(--ifm-color-emphasis-200);
  padding: 8px 12px;
  border-radius: 12px 12px 12px 0;
  max-width: 80%;
}

.inputForm {
  display: flex;
  padding: 12px;
  gap: 8px;
  border-top: 1px solid var(--ifm-color-emphasis-300);
}

.inputForm input {
  flex: 1;
  padding: 8px 12px;
  border: 1px solid var(--ifm-color-emphasis-400);
  border-radius: 8px;
  font-size: 0.875rem;
}

.inputForm button {
  padding: 8px 16px;
  background: var(--ifm-color-primary);
  color: white;
  border: none;
  border-radius: 8px;
  cursor: pointer;
}

.floatingButton {
  position: fixed;
  bottom: 24px;
  right: 24px;
  width: 56px;
  height: 56px;
  border-radius: 50%;
  background: var(--ifm-color-primary);
  color: white;
  border: none;
  font-size: 24px;
  cursor: pointer;
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
  z-index: 1000;
}

.chatPanel {
  position: fixed;
  bottom: 96px;
  right: 24px;
  width: 380px;
  height: 600px;
  border-radius: 12px;
  box-shadow: 0 8px 32px rgba(0, 0, 0, 0.2);
  z-index: 999;
  overflow: hidden;
}

@media (max-width: 480px) {
  .chatPanel {
    width: calc(100vw - 32px);
    height: calc(100vh - 160px);
    right: 16px;
  }
}
```

## Project Structure

### Documentation (this feature)

```text
specs/003-context-rag-chatbot/
├── spec.md                    # Feature specification (complete)
├── plan.md                    # This file
├── checklists/
│   └── requirements.md        # Requirements checklist
└── tasks.md                   # Task list
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── __init__.py
│   ├── main.py                # FastAPI app entry point
│   ├── config.py              # Environment configuration
│   ├── api/
│   │   ├── __init__.py
│   │   ├── chat.py            # /api/chat streaming endpoint
│   │   ├── chapters.py        # /api/chapters endpoint
│   │   └── index.py           # /api/index endpoint (admin)
│   ├── models/
│   │   ├── __init__.py
│   │   ├── chat.py            # Pydantic models for chat
│   │   └── chapter.py         # Chapter metadata models
│   └── services/
│       ├── __init__.py
│       ├── rag.py             # RAG pipeline service
│       ├── gemini.py          # Gemini LLM service
│       ├── embedding.py       # Google embedding service
│       ├── indexer.py         # Content indexing service
│       ├── qdrant.py          # Qdrant client wrapper
│       └── postgres.py        # Neon Postgres client
├── scripts/
│   ├── index_chapters.py      # CLI for batch indexing
│   └── test_connection.py     # Verify external services
├── requirements.txt
├── .env.example
└── Dockerfile

src/
├── components/
│   └── ChapterChat/
│       ├── index.tsx          # Main chat component with useChat
│       ├── FloatingButton.tsx # Floating toggle button
│       └── styles.module.css  # Component styles
├── theme/
│   └── DocItem/
│       └── Layout/
│           └── index.tsx      # Swizzled to inject FloatingChatButton
└── css/
    └── custom.css             # Global chat styling
```

## Integration Strategy

### Docusaurus Integration

1. **Install Vercel AI SDK**: `npm install ai @ai-sdk/google`
2. **Theme Swizzling**: Swizzle `DocItem/Layout` to inject `FloatingChatButton`
3. **Chapter Detection**: Use `useDoc()` hook to get chapter metadata
4. **Environment**: Add `CHAT_API_URL` to docusaurus.config.ts

### Backend Deployment

1. **Development**: Local FastAPI with uvicorn
2. **Production**: Railway.app (recommended for hackathon - easy deploy)

### Environment Variables

```bash
# Backend (.env)
GOOGLE_API_KEY=xxx              # Single key for embeddings + Gemini
QDRANT_URL=https://xxx.qdrant.tech
QDRANT_API_KEY=xxx
NEON_DATABASE_URL=postgresql://xxx

# Frontend (docusaurus.config.ts customFields)
CHAT_API_URL=https://your-backend.railway.app
```

## Complexity Tracking

| Decision | Justification | Simpler Alternative Rejected |
|----------|---------------|------------------------------|
| Gemini 2.0 Flash | Single API key for embeddings + LLM | Multi-provider adds complexity |
| Vercel AI SDK | Built-in streaming, useChat hook | Raw fetch/SSE requires more code |
| Separate backend | Constitution mandates FastAPI; need RAG orchestration | Serverless functions lack state |

## Risk Analysis

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| Gemini rate limits | Medium | Medium | Implement request queuing |
| Qdrant free-tier limits | Medium | High | Monitor usage; implement caching |
| CORS issues | Low | Medium | Configure FastAPI CORS properly |

## ADR Candidates

📋 **Architectural decision detected**: Gemini over OpenAI ChatKit
   - Trade-offs: More frontend code vs. single API provider
   - Document reasoning? Run `/sp.adr gemini-over-chatkit`

---

## References

- [Vercel AI SDK Documentation](https://sdk.vercel.ai/docs)
- [Gemini API Documentation](https://ai.google.dev/docs)
- [Qdrant Cloud Documentation](https://qdrant.tech/documentation/)
