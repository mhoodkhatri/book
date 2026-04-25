"""DeepSeek LLM service (OpenAI-compatible API)."""

import os
import httpx


class DeepSeekService:
    """Calls DeepSeek's OpenAI-compatible chat completions endpoint."""

    URL = "https://api.deepseek.com/v1/chat/completions"

    def __init__(self):
        self.api_key = os.getenv("DEEPSEEK_API_KEY", "").strip()
        self.model = os.getenv("DEEPSEEK_MODEL", "deepseek-chat")
        self._client = httpx.Client(timeout=120.0)

    def generate_response(
        self,
        system_prompt: str,
        messages: list[dict],
        max_tokens: int = 4096,
        temperature: float = 0.3,
    ) -> str:
        msgs = [{"role": "system", "content": system_prompt}] + [
            {"role": m["role"], "content": m["content"]} for m in messages
        ]
        r = self._client.post(
            self.URL,
            headers={
                "Authorization": f"Bearer {self.api_key}",
                "Content-Type": "application/json",
            },
            json={
                "model": self.model,
                "messages": msgs,
                "max_tokens": max_tokens,
                "temperature": temperature,
            },
        )
        r.raise_for_status()
        data = r.json()
        return data["choices"][0]["message"]["content"]


_service: DeepSeekService | None = None


def get_deepseek_service() -> DeepSeekService:
    global _service
    if _service is None:
        _service = DeepSeekService()
    return _service
