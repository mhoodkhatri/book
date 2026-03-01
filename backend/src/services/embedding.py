"""Local embedding service using fastembed (ONNX-based, no API key needed)."""

from fastembed import TextEmbedding

from src.config import get_settings


class EmbeddingService:
    """Service for generating text embeddings using fastembed."""

    def __init__(self):
        settings = get_settings()
        self.model = TextEmbedding(
            model_name=settings.embedding_model,
            cache_dir=settings.embedding_cache_dir,
        )
        self.dimension = settings.embedding_dimension

    def embed_text(self, text: str) -> list[float]:
        """
        Generate embedding for a single text (for queries).

        Args:
            text: The text to embed

        Returns:
            Embedding vector
        """
        embeddings = list(self.model.embed([text]))
        return embeddings[0].tolist()

    def embed_documents(self, texts: list[str]) -> list[list[float]]:
        """
        Generate embeddings for multiple documents (for indexing).

        Args:
            texts: List of texts to embed

        Returns:
            List of embedding vectors
        """
        if not texts:
            return []

        embeddings = list(self.model.embed(texts))
        return [e.tolist() for e in embeddings]

    def embed_batch(
        self,
        texts: list[str],
        batch_size: int = 100,
    ) -> list[list[float]]:
        """
        Generate embeddings for large number of documents in batches.

        Args:
            texts: List of texts to embed
            batch_size: Number of texts per batch

        Returns:
            List of embedding vectors
        """
        embeddings = list(self.model.embed(texts, batch_size=batch_size))
        return [e.tolist() for e in embeddings]


# Singleton instance
_embedding_service: EmbeddingService | None = None


def get_embedding_service() -> EmbeddingService:
    """Get or create the embedding service singleton."""
    global _embedding_service
    if _embedding_service is None:
        _embedding_service = EmbeddingService()
    return _embedding_service
