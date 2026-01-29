"""Google text-embedding-004 service for generating embeddings."""

import google.generativeai as genai

from src.config import get_settings


class EmbeddingService:
    """Service for generating text embeddings using Google's text-embedding-004."""

    def __init__(self):
        settings = get_settings()
        genai.configure(api_key=settings.google_api_key)
        self.model = settings.embedding_model
        self.dimension = settings.embedding_dimension

    def embed_text(self, text: str) -> list[float]:
        """
        Generate embedding for a single text (for queries).

        Args:
            text: The text to embed

        Returns:
            768-dimensional embedding vector
        """
        result = genai.embed_content(
            model=f"models/{self.model}",
            content=text,
            task_type="retrieval_query",
        )
        return result["embedding"]

    def embed_documents(self, texts: list[str]) -> list[list[float]]:
        """
        Generate embeddings for multiple documents (for indexing).

        Args:
            texts: List of texts to embed

        Returns:
            List of 768-dimensional embedding vectors
        """
        if not texts:
            return []

        # Google API supports batch embedding
        result = genai.embed_content(
            model=f"models/{self.model}",
            content=texts,
            task_type="retrieval_document",
        )

        # Handle both single and batch results
        embeddings = result["embedding"]
        if texts and not isinstance(embeddings[0], list):
            # Single text returns flat list
            return [embeddings]
        return embeddings

    def embed_batch(
        self,
        texts: list[str],
        batch_size: int = 100,
    ) -> list[list[float]]:
        """
        Generate embeddings for large number of documents in batches.

        Args:
            texts: List of texts to embed
            batch_size: Number of texts per API call

        Returns:
            List of embedding vectors
        """
        all_embeddings = []

        for i in range(0, len(texts), batch_size):
            batch = texts[i : i + batch_size]
            batch_embeddings = self.embed_documents(batch)
            all_embeddings.extend(batch_embeddings)

        return all_embeddings


# Singleton instance
_embedding_service: EmbeddingService | None = None


def get_embedding_service() -> EmbeddingService:
    """Get or create the embedding service singleton."""
    global _embedding_service
    if _embedding_service is None:
        _embedding_service = EmbeddingService()
    return _embedding_service
