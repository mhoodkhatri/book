"""Configuration module for environment variables."""

import os
from functools import lru_cache
from dotenv import load_dotenv

load_dotenv()


class Settings:
    """Application settings loaded from environment variables."""

    # Google AI API (optional, only if using Google embeddings)
    google_api_key: str = os.getenv("GOOGLE_API_KEY", "")

    # Qdrant — supports both cloud and local file-based storage
    # If QDRANT_URL is set, use cloud; otherwise, use local file storage
    qdrant_url: str = os.getenv("QDRANT_URL", "")
    qdrant_api_key: str = os.getenv("QDRANT_API_KEY", "")
    qdrant_local_path: str = os.getenv(
        "QDRANT_LOCAL_PATH",
        os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "qdrant_data"),
    )

    # Collection name for textbook chunks
    qdrant_collection: str = "textbook_chunks"

    # Gemini model (backup, not used with Groq)
    gemini_model: str = "gemini-2.0-flash"

    # Groq API (primary LLM provider)
    groq_api_key: str = os.getenv("GROQ_API_KEY", "")
    groq_model: str = "llama-3.3-70b-versatile"  # Fast and capable

    # Embedding model (fastembed, ONNX-based, runs locally)
    embedding_model: str = "BAAI/bge-small-en-v1.5"
    embedding_dimension: int = 384
    embedding_cache_dir: str = os.getenv(
        "EMBEDDING_CACHE_DIR",
        os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), ".embedding_cache"),
    )

    # Server
    host: str = os.getenv("HOST", "0.0.0.0")
    port: int = int(os.getenv("PORT", "8000"))

    # CORS origins — reads from CORS_ORIGINS env var in production;
    # falls back to localhost defaults for local development
    cors_origins: list[str] = [o.strip() for o in os.getenv("CORS_ORIGINS", "").split(",") if o.strip()] or [
        "http://localhost:3000",
        "http://localhost:3001",
        "http://127.0.0.1:3000",
    ]

    def validate(self) -> None:
        """Validate required settings are present."""
        if not self.groq_api_key:
            raise ValueError("GROQ_API_KEY environment variable is required")


@lru_cache
def get_settings() -> Settings:
    """Get cached settings instance."""
    return Settings()
