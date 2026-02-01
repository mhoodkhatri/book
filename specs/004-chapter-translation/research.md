# Research: Chapter Translation Toggle

**Feature**: 004-chapter-translation
**Date**: 2026-01-29
**Status**: Complete

## Research Questions

### 1. LLM Translation Approach

**Question**: Which LLM service to use for English-to-Urdu translation?

**Decision**: Use existing Groq service (Llama 3.3 70B) as primary, Gemini as fallback

**Rationale**:
- Already integrated and tested in RAG chatbot
- Llama 3.3 70B has strong multilingual capabilities including Urdu
- No additional API keys or service configuration required
- Consistent error handling patterns established

**Alternatives Considered**:
| Option | Pros | Cons | Decision |
|--------|------|------|----------|
| Groq (Llama 3.3) | Fast inference, existing integration | Less specialized than dedicated translation APIs | ✅ Selected |
| Google Translate API | Purpose-built, high quality | Additional service, cost, API key | ❌ Rejected |
| OpenAI GPT-4 | High quality | Not in existing stack, additional cost | ❌ Rejected |
| DeepL | Excellent translation quality | No Urdu support | ❌ Rejected |

### 2. Content Extraction Strategy

**Question**: How to extract chapter content for translation?

**Decision**: Frontend DOM extraction via `document.querySelector('.markdown')`

**Rationale**:
- Docusaurus renders all chapter content inside `.markdown` class container
- Extracts already-rendered HTML, preserving all formatting
- Avoids re-parsing MDX source files
- Simple JavaScript API, no additional libraries

**Implementation Pattern**:
```typescript
// Extract chapter content
const contentElement = document.querySelector('.markdown');
const htmlContent = contentElement?.innerHTML ?? '';

// Send to backend for translation
const response = await fetch('/api/translate', {
  method: 'POST',
  body: JSON.stringify({ content: htmlContent, targetLanguage: 'urdu' })
});
```

### 3. Code Block Preservation

**Question**: How to prevent code blocks from being translated?

**Decision**: Backend HTML parsing with BeautifulSoup, skip `<pre>` and `<code>` elements

**Rationale**:
- Server-side parsing gives more control over DOM traversal
- BeautifulSoup handles malformed HTML gracefully
- Can identify code blocks by tag name (`<pre>`, `<code>`) and class (`language-*`)
- Preserves inline code (backtick code) as well as code blocks

**Implementation Pattern**:
```python
from bs4 import BeautifulSoup

def extract_translatable_text(html: str) -> list[tuple[str, str]]:
    """Extract text nodes, skipping code elements."""
    soup = BeautifulSoup(html, 'html.parser')

    # Skip these elements entirely
    for code_element in soup.find_all(['pre', 'code']):
        code_element['data-skip-translation'] = 'true'

    # Extract text from remaining elements
    # ... (implementation details in translator.py)
```

### 4. RTL Text Handling

**Question**: How to properly render Urdu (right-to-left) text?

**Decision**: Add `dir="rtl"` attribute to content container + CSS adjustments

**Rationale**:
- HTML `dir` attribute is the standard approach for RTL
- Docusaurus supports RTL via CSS, no core changes needed
- Can scope RTL styles to translated content only
- Maintains LTR for code blocks (code is always LTR)

**Implementation Pattern**:
```typescript
// When translation is active
contentElement.setAttribute('dir', 'rtl');
contentElement.classList.add('urdu-content');

// When restoring English
contentElement.setAttribute('dir', 'ltr');
contentElement.classList.remove('urdu-content');
```

**CSS Requirements**:
```css
.urdu-content {
  font-family: 'Noto Nastaliq Urdu', 'Jameel Noori Nastaleeq', serif;
  line-height: 2;
  text-align: right;
}

.urdu-content pre,
.urdu-content code {
  direction: ltr;
  text-align: left;
}
```

### 5. Caching Strategy

**Question**: Where to cache original and translated content?

**Decision**: localStorage with chapter-keyed entries

**Rationale**:
- Persists across browser sessions (unlike sessionStorage)
- ~5-10MB per domain, sufficient for chapter content
- Synchronous read/write API (instant restore)
- No backend infrastructure needed
- Simple key-value model

**Cache Schema**:
```typescript
interface ChapterTranslationCache {
  chapterId: string;
  english: string;         // Original HTML
  urdu: string | null;     // Translated HTML (null if not yet translated)
  translatedAt: number;    // Unix timestamp
}

// Key format: `translation_cache_${chapterId}`
// Example: translation_cache_module-1-ros2/nodes-topics
```

**Cache Invalidation**:
- Cache persists until manually cleared or browser storage cleared
- Optional: Add TTL of 7 days for translations (LLM outputs may improve)
- Page refresh does NOT invalidate cache

### 6. Scroll Position Preservation

**Question**: How to maintain reading position during language toggle?

**Decision**: Capture scroll percentage before DOM update, restore after

**Rationale**:
- Absolute pixel position may shift due to content length differences
- Percentage-based position is more robust
- Can use Intersection Observer for section-aware preservation (enhancement)

**Implementation Pattern**:
```typescript
function preserveScrollPosition(callback: () => void) {
  const scrollContainer = document.documentElement;
  const scrollHeight = scrollContainer.scrollHeight;
  const scrollTop = scrollContainer.scrollTop;
  const scrollPercent = scrollTop / (scrollHeight - window.innerHeight);

  callback(); // DOM update happens here

  // Restore position after DOM settles
  requestAnimationFrame(() => {
    const newScrollHeight = scrollContainer.scrollHeight;
    const newScrollTop = scrollPercent * (newScrollHeight - window.innerHeight);
    scrollContainer.scrollTop = newScrollTop;
  });
}
```

### 7. Error Handling Strategy

**Question**: How to handle translation failures gracefully?

**Decision**: Keep original content, show toast notification, revert button state

**Rationale**:
- User should never lose access to content
- Clear feedback about what went wrong
- Easy retry via same button
- No page refresh required

**Error Scenarios**:
| Scenario | User Experience | Button State |
|----------|-----------------|--------------|
| Network error | Toast: "Translation failed. Check connection." | "Translate to Urdu" |
| LLM timeout | Toast: "Translation timed out. Try again." | "Translate to Urdu" |
| Invalid response | Toast: "Translation error. Original content preserved." | "Translate to Urdu" |
| Cache full | Warning: "Cache full. Clearing old translations." | Continues normally |

### 8. RAG Chatbot Integration

**Question**: How does translation affect the RAG chatbot?

**Decision**: No changes required - chatbot operates independently

**Rationale**:
- RAG queries the vector store (English content indexed in Qdrant)
- Chatbot works on original English content regardless of display language
- User can ask questions about translated content, gets English answers
- This is actually beneficial - technical accuracy preserved in responses

**Interaction Flow**:
1. User views Urdu translation
2. User selects text and asks chatbot
3. Chatbot receives English query (from selection context)
4. RAG retrieves English context from Qdrant
5. Response generated in English
6. User sees English response in chat panel

### 9. Loading State UX

**Question**: How to indicate translation is in progress?

**Decision**: Button transforms to spinner + "Translating..." text

**Rationale**:
- Consistent with existing UI patterns (chat loading state)
- Clear visual feedback
- Button disabled during translation (prevent double-click)
- Optional: Skeleton loading on content area for long translations

**Implementation Pattern**:
```typescript
// Button states
type ButtonState =
  | { state: 'english'; label: 'Translate to Urdu' }
  | { state: 'translating'; label: 'Translating...'; disabled: true }
  | { state: 'urdu'; label: 'Show Original' }
  | { state: 'restoring'; label: 'Restoring...'; disabled: true };
```

### 10. Technical Terms Handling

**Question**: Should technical terms (ROS2, Gazebo, Isaac Sim) be translated?

**Decision**: Preserve technical terms in English within Urdu text

**Rationale**:
- Technical terms are universally recognized in English
- Urdu translations of "ROS2" or "Gazebo" would confuse readers
- Industry standard practice in technical documentation
- LLM can be prompted to preserve specific terms

**Prompt Pattern**:
```
Translate the following HTML content from English to Urdu.
IMPORTANT:
- Preserve all HTML tags exactly as they are
- Do NOT translate text inside <pre> or <code> tags
- Keep technical terms in English: ROS, ROS2, Gazebo, Isaac Sim, NVIDIA,
  URDF, SLAM, Nav2, lidar, IMU, API, SDK, etc.
- Maintain the same heading hierarchy (h1, h2, h3, etc.)
- Preserve all class names and attributes
```

## Dependencies Identified

| Dependency | Purpose | Status |
|------------|---------|--------|
| Groq SDK | LLM translation calls | ✅ Already installed |
| BeautifulSoup4 | HTML parsing in backend | ⚠️ Need to verify/install |
| Noto Nastaliq Urdu font | Urdu typography | ⚠️ Need to add to Docusaurus |

## Risks and Mitigations

| Risk | Impact | Mitigation |
|------|--------|------------|
| LLM translation quality varies | User sees inconsistent Urdu | Add review process, prompt engineering |
| Long chapters timeout | Translation fails | Implement streaming or chunking |
| localStorage quota exceeded | Cache fails | Implement LRU eviction |
| RTL layout breaks | Poor readability | Comprehensive CSS testing |

## Next Steps

1. ✅ Research complete - proceed to Phase 1 Design
2. Create data-model.md with entity definitions
3. Create API contract in contracts/translate-api.yaml
4. Create quickstart.md for developer onboarding
