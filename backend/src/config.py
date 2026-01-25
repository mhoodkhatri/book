"""Configuration module for environment variables."""

import os
from functools import lru_cache
from dotenv import load_dotenv

load_dotenv()


class Settings:
    """Application settings loaded from environment variables."""

    # Google AI API (Gemini + Embeddings)
    google_api_key: str = os.getenv("GOOGLE_API_KEY", "")

    # Qdrant Cloud
    qdrant_url: str = os.getenv("QDRANT_URL", "")
    qdrant_api_key: str = os.getenv("QDRANT_API_KEY", "")

    # Collection name for textbook chunks
    qdrant_collection: str = "textbook_chunks"

    # Gemini model (backup, not used with Groq)
    gemini_model: str = "gemini-2.0-flash"

    # Groq API (primary LLM provider)
    groq_api_key: str = os.getenv("GROQ_API_KEY", "")
    groq_model: str = "llama-3.3-70b-versatile"  # Fast and capable

    # Embedding model
    embedding_model: str = "text-embedding-004"
    embedding_dimension: int = 768

    # Server
    host: str = os.getenv("HOST", "0.0.0.0")
    port: int = int(os.getenv("PORT", "8000"))

    # CORS origins (for development)
    cors_origins: list[str] = [
        "http://localhost:3000",
        "http://localhost:3001",
        "http://127.0.0.1:3000",
    ]

    def validate(self) -> None:
        """Validate required settings are present."""
        if not self.google_api_key:
            raise ValueError("GOOGLE_API_KEY environment variable is required")
        if not self.qdrant_url:
            raise ValueError("QDRANT_URL environment variable is required")
        if not self.qdrant_api_key:
            raise ValueError("QDRANT_API_KEY environment variable is required")


@lru_cache
def get_settings() -> Settings:
    """Get cached settings instance."""
    return Settings()
