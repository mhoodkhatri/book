"""Qdrant client wrapper for vector storage operations.

Supports both Qdrant Cloud (via URL + API key) and local file-based storage.
If QDRANT_URL is set, uses cloud; otherwise, uses local persistence.
"""

from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams, PointStruct, Filter, FieldCondition, MatchValue

from src.config import get_settings


class QdrantService:
    """Wrapper for Qdrant vector database operations."""

    def __init__(self):
        settings = get_settings()

        if settings.qdrant_url:
            # Cloud mode
            self.client = QdrantClient(
                url=settings.qdrant_url,
                api_key=settings.qdrant_api_key,
            )
        else:
            # Local file-based mode (no server needed)
            self.client = QdrantClient(path=settings.qdrant_local_path)

        self.collection_name = settings.qdrant_collection
        self.dimension = settings.embedding_dimension

    def ensure_collection(self) -> bool:
        """Create collection if it doesn't exist, including payload index."""
        collections = self.client.get_collections().collections
        exists = any(c.name == self.collection_name for c in collections)

        if not exists:
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=self.dimension,
                    distance=Distance.COSINE,
                ),
            )
            # Required for filtered deletes and searches by chapter_id
            self.client.create_payload_index(
                collection_name=self.collection_name,
                field_name="chapter_id",
                field_schema="keyword",
            )
            return True
        return False

    def search(
        self,
        query_vector: list[float],
        chapter_id: str,
        top_k: int = 5,
    ) -> list[dict]:
        """
        Search for similar chunks within a specific chapter.

        Args:
            query_vector: The query embedding vector
            chapter_id: Filter results to this chapter only
            top_k: Number of results to return

        Returns:
            List of matching chunks with scores
        """
        results = self.client.query_points(
            collection_name=self.collection_name,
            query=query_vector,
            query_filter=Filter(
                must=[
                    FieldCondition(
                        key="chapter_id",
                        match=MatchValue(value=chapter_id),
                    )
                ]
            ),
            limit=top_k,
        )

        return [
            {
                "id": str(hit.id),
                "score": hit.score,
                "content": hit.payload.get("content", ""),
                "section_heading": hit.payload.get("section_heading", ""),
                "chapter_id": hit.payload.get("chapter_id", ""),
                "chapter_title": hit.payload.get("chapter_title", ""),
                "url": hit.payload.get("url", ""),
                "chunk_index": hit.payload.get("chunk_index", 0),
            }
            for hit in results.points
        ]

    def upsert_chunks(self, chunks: list[dict]) -> int:
        """
        Insert or update chunks in the collection.

        Args:
            chunks: List of chunk dictionaries with 'id', 'vector', and payload fields

        Returns:
            Number of chunks upserted
        """
        points = [
            PointStruct(
                id=chunk["id"],
                vector=chunk["vector"],
                payload={
                    "content": chunk["content"],
                    "chapter_id": chunk["chapter_id"],
                    "chapter_title": chunk["chapter_title"],
                    "module": chunk.get("module", ""),
                    "section_heading": chunk.get("section_heading", ""),
                    "chunk_index": chunk.get("chunk_index", 0),
                    "word_count": chunk.get("word_count", 0),
                    "url": chunk.get("url", ""),
                },
            )
            for chunk in chunks
        ]

        self.client.upsert(
            collection_name=self.collection_name,
            points=points,
        )

        return len(points)

    def delete_chapter(self, chapter_id: str) -> int:
        """Delete all chunks for a specific chapter."""
        result = self.client.delete(
            collection_name=self.collection_name,
            points_selector=models.FilterSelector(
                filter=Filter(
                    must=[
                        FieldCondition(
                            key="chapter_id",
                            match=MatchValue(value=chapter_id),
                        )
                    ]
                )
            ),
        )
        return result

    def get_collection_info(self) -> dict:
        """Get collection statistics."""
        info = self.client.get_collection(self.collection_name)
        return {
            "name": self.collection_name,
            "vectors_count": info.vectors_count,
            "points_count": info.points_count,
        }


# Singleton instance
_qdrant_service: QdrantService | None = None


def get_qdrant_service() -> QdrantService:
    """Get or create the Qdrant service singleton."""
    global _qdrant_service
    if _qdrant_service is None:
        _qdrant_service = QdrantService()
    return _qdrant_service
