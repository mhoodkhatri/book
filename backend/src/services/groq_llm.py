"""Groq LLM service for fast chat responses."""

from typing import AsyncGenerator
from groq import Groq, AsyncGroq

from src.config import get_settings


class GroqService:
    """Service for generating chat responses using Groq."""

    def __init__(self):
        settings = get_settings()
        self.client = Groq(api_key=settings.groq_api_key)
        self.async_client = AsyncGroq(api_key=settings.groq_api_key)
        self.model = settings.groq_model

    async def stream_response(
        self,
        system_prompt: str,
        messages: list[dict],
    ) -> AsyncGenerator[str, None]:
        """
        Stream a response from Groq.

        Args:
            system_prompt: The system instructions including RAG context
            messages: Conversation history with 'role' and 'content'

        Yields:
            Text chunks as they are generated
        """
        # Build messages with system prompt
        groq_messages = [{"role": "system", "content": system_prompt}]

        for msg in messages:
            groq_messages.append({
                "role": msg["role"],
                "content": msg["content"]
            })

        # Stream the response
        stream = await self.async_client.chat.completions.create(
            model=self.model,
            messages=groq_messages,
            temperature=0.7,
            max_tokens=2048,
            stream=True,
        )

        async for chunk in stream:
            if chunk.choices[0].delta.content:
                yield chunk.choices[0].delta.content

    def generate_response(
        self,
        system_prompt: str,
        messages: list[dict],
    ) -> str:
        """
        Generate a complete response (non-streaming).

        Args:
            system_prompt: The system instructions including RAG context
            messages: Conversation history

        Returns:
            Complete response text
        """
        groq_messages = [{"role": "system", "content": system_prompt}]

        for msg in messages:
            groq_messages.append({
                "role": msg["role"],
                "content": msg["content"]
            })

        response = self.client.chat.completions.create(
            model=self.model,
            messages=groq_messages,
            temperature=0.7,
            max_tokens=2048,
        )

        return response.choices[0].message.content


# Singleton instance
_groq_service: GroqService | None = None


def get_groq_service() -> GroqService:
    """Get or create the Groq service singleton."""
    global _groq_service
    if _groq_service is None:
        _groq_service = GroqService()
    return _groq_service
