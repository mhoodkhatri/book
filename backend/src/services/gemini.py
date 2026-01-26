"""Gemini 2.0 Flash service for LLM chat responses."""

from typing import AsyncGenerator
import google.generativeai as genai

from src.config import get_settings


class GeminiService:
    """Service for generating chat responses using Gemini 2.0 Flash."""

    def __init__(self):
        settings = get_settings()
        genai.configure(api_key=settings.google_api_key)
        self.model = genai.GenerativeModel(
            model_name=settings.gemini_model,
            generation_config={
                "temperature": 0.7,
                "top_p": 0.95,
                "top_k": 40,
                "max_output_tokens": 2048,
            },
        )

    async def stream_response(
        self,
        system_prompt: str,
        messages: list[dict],
    ) -> AsyncGenerator[str, None]:
        """
        Stream a response from Gemini.

        Args:
            system_prompt: The system instructions including RAG context
            messages: Conversation history with 'role' and 'content'

        Yields:
            Text chunks as they are generated
        """
        # Build the conversation history
        history = []
        for msg in messages[:-1]:  # All but the last message
            role = "user" if msg["role"] == "user" else "model"
            history.append({"role": role, "parts": [msg["content"]]})

        # Start chat with history
        chat = self.model.start_chat(history=history)

        # Build the prompt with system context + user message
        user_message = messages[-1]["content"] if messages else ""
        full_prompt = f"{system_prompt}\n\nUser question: {user_message}"

        # Stream the response
        response = await chat.send_message_async(full_prompt, stream=True)

        async for chunk in response:
            if chunk.text:
                yield chunk.text

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
        history = []
        for msg in messages[:-1]:
            role = "user" if msg["role"] == "user" else "model"
            history.append({"role": role, "parts": [msg["content"]]})

        chat = self.model.start_chat(history=history)

        user_message = messages[-1]["content"] if messages else ""
        full_prompt = f"{system_prompt}\n\nUser question: {user_message}"

        response = chat.send_message(full_prompt)
        return response.text


# Singleton instance
_gemini_service: GeminiService | None = None


def get_gemini_service() -> GeminiService:
    """Get or create the Gemini service singleton."""
    global _gemini_service
    if _gemini_service is None:
        _gemini_service = GeminiService()
    return _gemini_service
