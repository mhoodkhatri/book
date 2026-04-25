"""Embedding service using HuggingFace Inference API (no local model load)."""

import os
import time

import httpx

from src.config import get_settings


HF_API_URL = "https://router.huggingface.co/hf-inference/models/{model}/pipeline/feature-extraction"


class EmbeddingService:
    """Generates embeddings via HuggingFace Inference API (remote, no RAM cost)."""

    def __init__(self):
        settings = get_settings()
        self.model_name = settings.embedding_model
        self.dimension = settings.embedding_dimension
        self.url = HF_API_URL.format(model=self.model_name)
        token = os.getenv("HF_TOKEN", "").strip()
        self.headers = {"Authorization": f"Bearer {token}"} if token else {}
        self._client = httpx.Client(timeout=30.0)

    def _post(self, payload: dict) -> list:
        for attempt in range(5):
            r = self._client.post(self.url, headers=self.headers, json=payload)
            if r.status_code == 503:
                time.sleep(min(2 ** attempt, 15))
                continue
            r.raise_for_status()
            return r.json()
        r.raise_for_status()
        return r.json()

    def embed_text(self, text: str) -> list[float]:
        out = self._post({"inputs": text, "options": {"wait_for_model": True}})
        if isinstance(out, list) and out and isinstance(out[0], list):
            return out[0] if isinstance(out[0][0], (int, float)) else out[0]
        return out

    def embed_documents(self, texts: list[str]) -> list[list[float]]:
        if not texts:
            return []
        out = self._post({"inputs": texts, "options": {"wait_for_model": True}})
        return out

    def embed_batch(self, texts: list[str], batch_size: int = 32) -> list[list[float]]:
        results: list[list[float]] = []
        for i in range(0, len(texts), batch_size):
            results.extend(self.embed_documents(texts[i : i + batch_size]))
        return results


_embedding_service: EmbeddingService | None = None


def get_embedding_service() -> EmbeddingService:
    global _embedding_service
    if _embedding_service is None:
        _embedding_service = EmbeddingService()
    return _embedding_service
