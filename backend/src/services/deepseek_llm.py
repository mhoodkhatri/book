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
        self._aclient = httpx.AsyncClient(timeout=120.0)

    def _payload(self, system_prompt, messages, max_tokens, temperature):
        msgs = [{"role": "system", "content": system_prompt}] + [
            {"role": m["role"], "content": m["content"]} for m in messages
        ]
        return {
            "model": self.model,
            "messages": msgs,
            "max_tokens": max_tokens,
            "temperature": temperature,
        }

    def _headers(self):
        return {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json",
        }

    def generate_response(
        self,
        system_prompt: str,
        messages: list[dict],
        max_tokens: int = 4096,
        temperature: float = 0.3,
    ) -> str:
        r = self._client.post(
            self.URL,
            headers=self._headers(),
            json=self._payload(system_prompt, messages, max_tokens, temperature),
        )
        r.raise_for_status()
        return r.json()["choices"][0]["message"]["content"]

    async def agenerate_response(
        self,
        system_prompt: str,
        messages: list[dict],
        max_tokens: int = 4096,
        temperature: float = 0.3,
    ) -> str:
        r = await self._aclient.post(
            self.URL,
            headers=self._headers(),
            json=self._payload(system_prompt, messages, max_tokens, temperature),
        )
        r.raise_for_status()
        return r.json()["choices"][0]["message"]["content"]


_service: DeepSeekService | None = None


def get_deepseek_service() -> DeepSeekService:
    global _service
    if _service is None:
        _service = DeepSeekService()
    return _service
