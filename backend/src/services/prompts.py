"""Prompt builder for chapter-scoped RAG responses."""

from src.services.rag import RetrievedChunk


class PromptBuilder:
    """Builds prompts for chapter-scoped Q&A."""

    SYSTEM_TEMPLATE = """You are a helpful teaching assistant for the "Physical AI & Humanoid Robotics" textbook.

IMPORTANT CONTEXT RULES:
1. You can ONLY answer questions about the current chapter: "{chapter_title}"
2. Use ONLY the provided context to answer questions
3. Always cite your sources using section references like [§Section Name]
4. If the question cannot be answered from the provided context, politely explain this
5. Be concise but thorough in your explanations
6. Use examples from the textbook when helpful

CURRENT CHAPTER: {chapter_title}
CHAPTER ID: {chapter_id}

RETRIEVED CONTEXT:
{context}

{selected_text_section}

RESPONSE GUIDELINES:
- Start with a direct answer to the question
- Include relevant citations [§Section Name] when referencing the context
- If you need to explain a concept not in the context, say "While not covered in this chapter..."
- Keep responses focused and educational"""

    SELECTED_TEXT_TEMPLATE = """USER SELECTED TEXT (use this as primary focus):
\"\"\"{selected_text}\"\"\""""

    REFUSAL_OUT_OF_SCOPE = """I can only answer questions about the current chapter: "{chapter_title}".

Your question doesn't seem to be directly related to this chapter's content. Here are some things I can help with based on this chapter's topics:

{chapter_topics}

Would you like to ask about any of these topics instead?"""

    REFUSAL_NO_CONTENT = """I don't have any indexed content for this chapter yet. The chapter may not have been processed for the AI assistant.

Please try:
1. Checking if you're on a chapter page (not an index page)
2. Asking about a different chapter
3. Contacting the textbook maintainers if this persists"""

    REFUSAL_OFF_TOPIC = """I'm a teaching assistant specifically for the "Physical AI & Humanoid Robotics" textbook. I can only answer questions related to the textbook content.

Your question appears to be about something outside the scope of this textbook. Is there anything about {chapter_title} I can help you with instead?"""

    def build_system_prompt(
        self,
        chapter_id: str,
        chapter_title: str,
        context: str,
        selected_text: str | None = None,
    ) -> str:
        """
        Build the system prompt with chapter context.

        Args:
            chapter_id: Current chapter URL slug
            chapter_title: Human-readable chapter title
            context: Formatted context from retrieved chunks
            selected_text: User-selected text (optional)

        Returns:
            Complete system prompt
        """
        selected_text_section = ""
        if selected_text:
            selected_text_section = self.SELECTED_TEXT_TEMPLATE.format(
                selected_text=selected_text[:1000]  # Limit length
            )

        return self.SYSTEM_TEMPLATE.format(
            chapter_title=chapter_title,
            chapter_id=chapter_id,
            context=context or "No specific context retrieved for this query.",
            selected_text_section=selected_text_section,
        )

    def build_refusal_response(
        self,
        reason: str,
        chapter_title: str,
        chunks: list[RetrievedChunk] | None = None,
    ) -> str:
        """
        Build a polite refusal response when question can't be answered.

        Args:
            reason: Why the question can't be answered
            chapter_title: Current chapter title
            chunks: Available chunks (for topic suggestions)

        Returns:
            Refusal message
        """
        if reason == "no_content":
            return self.REFUSAL_NO_CONTENT

        if reason == "off_topic":
            return self.REFUSAL_OFF_TOPIC.format(chapter_title=chapter_title)

        if reason == "low_relevance":
            # Extract topics from available chunks
            topics = self._extract_topics(chunks) if chunks else []
            topic_list = "\n".join(f"- {topic}" for topic in topics[:5])

            if not topic_list:
                topic_list = "- (No topics available - chapter may not be fully indexed)"

            return self.REFUSAL_OUT_OF_SCOPE.format(
                chapter_title=chapter_title,
                chapter_topics=topic_list,
            )

        # Default refusal
        return f"I'm sorry, I couldn't find relevant information in the current chapter ({chapter_title}) to answer your question."

    def _extract_topics(
        self,
        chunks: list[RetrievedChunk],
    ) -> list[str]:
        """Extract unique section headings as available topics."""
        if not chunks:
            return []

        seen = set()
        topics = []
        for chunk in chunks:
            heading = chunk.section_heading
            if heading and heading not in seen:
                seen.add(heading)
                topics.append(heading)

        return topics

    def is_likely_off_topic(self, query: str) -> bool:
        """
        Quick check if query is obviously off-topic.

        This catches things like weather, sports, general chit-chat.
        """
        off_topic_patterns = [
            "weather",
            "sports",
            "news",
            "stock",
            "price",
            "movie",
            "music",
            "game",
            "joke",
            "hello",
            "hi there",
            "how are you",
            "what's up",
            "who are you",
            "your name",
        ]

        query_lower = query.lower()
        return any(pattern in query_lower for pattern in off_topic_patterns)


# Singleton instance
_prompt_builder: PromptBuilder | None = None


def get_prompt_builder() -> PromptBuilder:
    """Get or create the prompt builder singleton."""
    global _prompt_builder
    if _prompt_builder is None:
        _prompt_builder = PromptBuilder()
    return _prompt_builder
