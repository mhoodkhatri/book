# Feature Specification: Chapter Translation Toggle

**Feature Branch**: `004-chapter-translation`
**Created**: 2026-01-29
**Status**: Draft
**Input**: User description: "Implement a Translate button that toggles book chapter content between English and Urdu in-place, preserving reading position, formatting, and structure. The original English content must be cached for instant toggle-back."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Translate Current Chapter to Urdu (Priority: P1)

As a reader viewing a book chapter in English, I want to click a single "Translate" button so that the chapter content is translated to Urdu in-place, allowing me to read the technical content in my preferred language without losing my reading position.

**Why this priority**: This is the core feature - enabling Urdu-speaking readers to access the textbook content in their native language. Without this, the feature has no value.

**Independent Test**: Can be fully tested by navigating to any chapter, clicking the Translate button, and verifying that all visible content changes from English to Urdu while maintaining the same page layout and scroll position.

**Acceptance Scenarios**:

1. **Given** I am viewing a chapter in English, **When** I click the "Translate" button, **Then** all chapter content (headings, paragraphs, lists, code comments) is replaced with Urdu text in the same layout
2. **Given** I am scrolled to a specific section of a chapter, **When** I click "Translate", **Then** my scroll position remains at the same relative location after translation
3. **Given** I am viewing a chapter in English, **When** I click "Translate", **Then** the button state changes to indicate the current language is Urdu

---

### User Story 2 - Toggle Back to English (Priority: P1)

As a reader viewing translated Urdu content, I want to click the same "Translate" button to restore the original English content exactly as it was, so I can easily switch between languages without any delay or content loss.

**Why this priority**: The toggle behavior is essential for the feature to be complete - users must be able to restore the original content. This is equally critical as P1 because one-way translation is not useful.

**Independent Test**: Can be tested by translating a chapter to Urdu, then clicking the button again and verifying the original English content is restored exactly, with no regeneration delay.

**Acceptance Scenarios**:

1. **Given** I am viewing a chapter in Urdu, **When** I click the "Translate" button, **Then** the original English content is restored exactly as it was before translation
2. **Given** I have toggled to Urdu and back to English, **When** I compare the restored content to the original, **Then** they are identical (no text differences)
3. **Given** I click Translate to Urdu, **When** I immediately click again to restore English, **Then** the restore happens instantly without any API call or loading state

---

### User Story 3 - Preserve Formatting and Structure (Priority: P2)

As a reader of technical content, I want the translation to preserve all formatting (headings, lists, code blocks, emphasis) so that the Urdu content maintains the same pedagogical structure and readability as the English original.

**Why this priority**: Technical textbooks rely on formatting for clarity. While not blocking basic functionality, poor formatting would make the translated content less useful.

**Independent Test**: Can be tested by translating a chapter with diverse formatting (H1-H4 headings, bullet lists, numbered lists, code blocks, bold/italic text, tables) and verifying each element type is preserved.

**Acceptance Scenarios**:

1. **Given** a chapter has H1, H2, H3 headings, **When** translated to Urdu, **Then** the heading hierarchy and visual styling is preserved
2. **Given** a chapter contains code blocks, **When** translated to Urdu, **Then** code blocks remain untranslated (code is language-agnostic) while surrounding explanatory text is translated
3. **Given** a chapter has bullet lists and numbered lists, **When** translated, **Then** the list structure and numbering is maintained
4. **Given** a chapter has bold or italic text for emphasis, **When** translated, **Then** the same emphasis is applied to the corresponding Urdu text

---

### User Story 4 - Visual Feedback During Translation (Priority: P3)

As a reader, I want to see clear visual feedback when translation is in progress so I understand the system is working and know when I can start reading.

**Why this priority**: Important for user experience but the feature works without sophisticated loading states. A basic indicator is sufficient for MVP.

**Independent Test**: Can be tested by clicking Translate on a long chapter and observing the loading indicator appears, then disappears when translation completes.

**Acceptance Scenarios**:

1. **Given** I click the "Translate" button, **When** translation is in progress, **Then** I see a loading indicator or the button shows a "Translating..." state
2. **Given** translation is in progress, **When** it completes successfully, **Then** the loading indicator disappears and the translated content is displayed
3. **Given** I toggle between languages multiple times, **When** restoring cached English content, **Then** no loading state is shown (instant restore)

---

### Edge Cases

- What happens when the user navigates to a different chapter while translation is in progress?
  - The in-progress translation should be cancelled; new chapter loads in English (default state)

- What happens if translation fails (network error, API timeout)?
  - The original English content remains displayed; an error notification is shown; the button state reverts to "Translate to Urdu"

- What happens with very long chapters?
  - Translation may take longer; a progress indicator should remain visible; content should update incrementally if possible

- What happens to code blocks and technical terms?
  - Code blocks are preserved unchanged; technical terms (ROS2, Gazebo, Isaac Sim) are kept in English within Urdu text

- What happens if the user is offline?
  - If cached translation exists, show cached Urdu; if no cache, show error and keep English

- What happens to embedded images, diagrams, and videos?
  - These remain unchanged; only text content is translated

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST display a single "Translate" button accessible from all book chapter pages
- **FR-002**: System MUST translate the currently visible chapter content from English to Urdu when the button is clicked (first click)
- **FR-003**: System MUST restore the original English content exactly when the button is clicked again (second click)
- **FR-004**: System MUST replace content in-place without opening any modal, popup, or new window
- **FR-005**: System MUST preserve the user's scroll position when switching between languages
- **FR-006**: System MUST store the original English content locally so that toggling back to English does not require regeneration
- **FR-007**: System MUST store translated Urdu content locally (per chapter) so that re-toggling to Urdu does not require re-translation
- **FR-008**: System MUST preserve all formatting (headings, lists, emphasis, tables) in the translated content
- **FR-009**: System MUST NOT translate code blocks, code snippets, or inline code
- **FR-010**: System MUST display a loading indicator while translation is in progress
- **FR-011**: System MUST handle translation errors gracefully by displaying an error message and keeping the original content
- **FR-012**: System MUST update the button label/icon to reflect the current language state
- **FR-013**: System MUST NOT affect the existing RAG chatbot functionality

### Key Entities

- **TranslationState**: Represents the current translation status of a chapter (language: English|Urdu, isTranslating: boolean, error: string|null)
- **ChapterCache**: Stores original English content and translated Urdu content per chapter ID in localStorage; keyed by chapter identifier with version hash for cache invalidation on content updates
- **TranslationRequest**: Contains chapter content to be translated with formatting markers preserved

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can translate a chapter from English to Urdu with a single button click
- **SC-002**: Users can restore original English content with a single button click
- **SC-003**: Toggle from Urdu back to English completes instantly (under 100ms perceived latency) using cached content
- **SC-004**: Scroll position is preserved within 50 pixels of original position after language toggle
- **SC-005**: All heading levels, list structures, and text emphasis are preserved in translated content
- **SC-006**: Code blocks remain unchanged (0% translation) in the Urdu version
- **SC-007**: Users can access the translate button within 2 seconds of page load
- **SC-008**: Translation errors display a user-friendly message without breaking the page
- **SC-009**: RAG chatbot continues to function identically after translation feature is added
- **SC-010**: Translation completes for average chapter length (2000 words) within 30 seconds maximum

## Clarifications

### Session 2026-01-29

- Q: Which translation provider should the system use for English-to-Urdu translation? → A: Groq (existing infrastructure)
- Q: How should translated content be cached across browser sessions? → A: localStorage with per-chapter keys
- Q: What is the acceptable maximum response time for the initial translation of a chapter? → A: 30 seconds
- Q: What is the cache expiry policy for translated content? → A: 7 days (translations older than 7 days are considered stale and re-translated on next request)

## Assumptions

- The translation will use the existing Groq API infrastructure with an LLM capable of translation (confirmed decision)
- Urdu text will display correctly with the existing font stack (right-to-left text direction will be handled via CSS)
- Translation quality is acceptable at LLM capability level (not professional human translation quality)
- Users have sufficient browser storage (localStorage/sessionStorage) for caching one chapter's content at a time
- The backend can handle translation requests without significant infrastructure changes
- Technical terms in robotics/AI domain may remain in English within Urdu text for clarity
