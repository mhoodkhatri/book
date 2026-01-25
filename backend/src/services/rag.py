"""RAG (Retrieval-Augmented Generation) service for chapter-scoped Q&A."""

from dataclasses import dataclass

from src.services.embedding import get_embedding_service
from src.services.qdrant import get_qdrant_service


@dataclass
class RetrievedChunk:
    """A chunk retrieved from the vector store."""

    content: str
    section_heading: str
    chapter_id: str
    chapter_title: str
    url: str
    score: float
    chunk_index: int


class RAGService:
    """Service for retrieval-augmented generation with chapter boundaries."""

    # Minimum relevance score to consider a chunk relevant
    RELEVANCE_THRESHOLD = 0.3

    # Minimum score to consider the question answerable
    ANSWERABLE_THRESHOLD = 0.5

    def __init__(self):
        self.embedding_service = get_embedding_service()
        self.qdrant_service = get_qdrant_service()

    def retrieve(
        self,
        query: str,
        chapter_id: str,
        top_k: int = 5,
    ) -> list[RetrievedChunk]:
        """
        Retrieve relevant chunks from the specified chapter only.

        Args:
            query: The user's question
            chapter_id: The chapter to search within
            top_k: Maximum number of chunks to return

        Returns:
            List of relevant chunks sorted by relevance
        """
        # Generate query embedding
        query_embedding = self.embedding_service.embed_text(query)

        # Search Qdrant with chapter filter
        results = self.qdrant_service.search(
            query_vector=query_embedding,
            chapter_id=chapter_id,
            top_k=top_k,
        )

        # Convert to RetrievedChunk objects
        chunks = []
        for result in results:
            if result["score"] >= self.RELEVANCE_THRESHOLD:
                chunks.append(
                    RetrievedChunk(
                        content=result["content"],
                        section_heading=result["section_heading"],
                        chapter_id=result["chapter_id"],
                        chapter_title=result["chapter_title"],
                        url=result["url"],
                        score=result["score"],
                        chunk_index=result["chunk_index"],
                    )
                )

        return chunks

    def is_question_answerable(
        self,
        chunks: list[RetrievedChunk],
    ) -> tuple[bool, str]:
        """
        Determine if the question can be answered from the retrieved chunks.

        Args:
            chunks: Retrieved chunks from the current chapter

        Returns:
            Tuple of (is_answerable, reason)
        """
        if not chunks:
            return False, "no_content"

        # Check if the best match is relevant enough
        best_score = max(chunk.score for chunk in chunks)

        if best_score < self.ANSWERABLE_THRESHOLD:
            return False, "low_relevance"

        return True, "answerable"

    def get_context_for_prompt(
        self,
        chunks: list[RetrievedChunk],
        max_chunks: int = 5,
    ) -> str:
        """
        Format retrieved chunks as context for the LLM prompt.

        Args:
            chunks: Retrieved chunks to format
            max_chunks: Maximum number of chunks to include

        Returns:
            Formatted context string with citations
        """
        if not chunks:
            return ""

        context_parts = []
        for i, chunk in enumerate(chunks[:max_chunks]):
            citation = f"[§{chunk.section_heading}]"
            context_parts.append(
                f"--- Source {i + 1} {citation} ---\n{chunk.content}\n"
            )

        return "\n".join(context_parts)

    def format_citations(
        self,
        chunks: list[RetrievedChunk],
    ) -> list[dict]:
        """
        Format chunks as citation references.

        Returns:
            List of citation dictionaries with section, url, and score
        """
        seen = set()
        citations = []

        for chunk in chunks:
            key = (chunk.section_heading, chunk.url)
            if key not in seen:
                seen.add(key)
                citations.append({
                    "section": chunk.section_heading,
                    "url": chunk.url,
                    "relevance": round(chunk.score, 3),
                })

        return citations


# Singleton instance
_rag_service: RAGService | None = None


def get_rag_service() -> RAGService:
    """Get or create the RAG service singleton."""
    global _rag_service
    if _rag_service is None:
        _rag_service = RAGService()
    return _rag_service
