"""Content indexer for chunking and storing chapter content."""

import re
import uuid
from dataclasses import dataclass

from src.services.embedding import get_embedding_service
from src.services.qdrant import get_qdrant_service


@dataclass
class ContentChunk:
    """A chunk of chapter content ready for indexing."""

    id: str
    content: str
    chapter_id: str
    chapter_title: str
    module: str
    section_heading: str
    chunk_index: int
    word_count: int
    url: str


class ContentIndexer:
    """Service for chunking and indexing chapter content."""

    def __init__(self):
        self.embedding_service = get_embedding_service()
        self.qdrant_service = get_qdrant_service()

    def chunk_markdown(
        self,
        markdown: str,
        chapter_id: str,
        chapter_title: str,
        target_tokens: int = 750,
    ) -> list[ContentChunk]:
        """
        Split markdown content into semantic chunks.

        Chunks are split by sections (## headings) and further split if too long.
        Target: 500-1000 tokens per chunk for optimal retrieval.

        Args:
            markdown: The markdown content to chunk
            chapter_id: URL slug for the chapter
            chapter_title: Human-readable chapter title
            target_tokens: Target chunk size in tokens (rough estimate: 1 token ≈ 4 chars)

        Returns:
            List of ContentChunk objects
        """
        chunks = []
        module = chapter_id.split("/")[0] if "/" in chapter_id else chapter_id

        # Split by sections (## headings)
        sections = re.split(r'\n(?=## )', markdown)

        chunk_index = 0
        for section in sections:
            if not section.strip():
                continue

            # Extract section heading
            heading_match = re.match(r'^## (.+?)(?:\n|$)', section)
            section_heading = heading_match.group(1).strip() if heading_match else "Introduction"

            # Remove the heading from content
            content = re.sub(r'^## .+?\n', '', section).strip()

            if not content:
                continue

            # Create URL anchor from heading
            anchor = re.sub(r'[^a-z0-9]+', '-', section_heading.lower()).strip('-')
            url = f"/docs/{chapter_id}#{anchor}"

            # Split large sections into smaller chunks
            section_chunks = self._split_section(
                content=content,
                target_chars=target_tokens * 4,  # Rough token to char conversion
            )

            for chunk_content in section_chunks:
                word_count = len(chunk_content.split())

                chunks.append(
                    ContentChunk(
                        id=str(uuid.uuid4()),
                        content=chunk_content,
                        chapter_id=chapter_id,
                        chapter_title=chapter_title,
                        module=module,
                        section_heading=section_heading,
                        chunk_index=chunk_index,
                        word_count=word_count,
                        url=url,
                    )
                )
                chunk_index += 1

        return chunks

    def _split_section(
        self,
        content: str,
        target_chars: int,
    ) -> list[str]:
        """Split a section into smaller chunks if needed."""
        if len(content) <= target_chars:
            return [content]

        chunks = []
        paragraphs = content.split('\n\n')
        current_chunk = []
        current_length = 0

        for para in paragraphs:
            para_length = len(para)

            if current_length + para_length > target_chars and current_chunk:
                # Save current chunk and start new one
                chunks.append('\n\n'.join(current_chunk))
                current_chunk = [para]
                current_length = para_length
            else:
                current_chunk.append(para)
                current_length += para_length

        # Don't forget the last chunk
        if current_chunk:
            chunks.append('\n\n'.join(current_chunk))

        return chunks

    def index_chapter(
        self,
        chapter_id: str,
        chapter_title: str,
        content: str,
        reindex: bool = False,
    ) -> int:
        """
        Index a chapter's content into Qdrant.

        Args:
            chapter_id: URL slug for the chapter
            chapter_title: Human-readable chapter title
            content: Markdown content
            reindex: If True, delete existing chunks first

        Returns:
            Number of chunks created
        """
        # Ensure collection exists
        self.qdrant_service.ensure_collection()

        # Delete existing chunks if reindexing
        if reindex:
            self.qdrant_service.delete_chapter(chapter_id)

        # Chunk the content
        chunks = self.chunk_markdown(
            markdown=content,
            chapter_id=chapter_id,
            chapter_title=chapter_title,
        )

        if not chunks:
            return 0

        # Generate embeddings
        texts = [chunk.content for chunk in chunks]
        embeddings = self.embedding_service.embed_batch(texts)

        # Prepare for upsert
        qdrant_chunks = []
        for chunk, embedding in zip(chunks, embeddings):
            qdrant_chunks.append({
                "id": chunk.id,
                "vector": embedding,
                "content": chunk.content,
                "chapter_id": chunk.chapter_id,
                "chapter_title": chunk.chapter_title,
                "module": chunk.module,
                "section_heading": chunk.section_heading,
                "chunk_index": chunk.chunk_index,
                "word_count": chunk.word_count,
                "url": chunk.url,
            })

        # Upsert to Qdrant
        self.qdrant_service.upsert_chunks(qdrant_chunks)

        return len(chunks)


# Singleton instance
_indexer: ContentIndexer | None = None


def get_indexer() -> ContentIndexer:
    """Get or create the indexer singleton."""
    global _indexer
    if _indexer is None:
        _indexer = ContentIndexer()
    return _indexer
