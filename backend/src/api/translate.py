"""Translation API endpoint using Groq LLM."""

import json
from typing import AsyncGenerator

from fastapi import APIRouter, HTTPException
from fastapi.responses import StreamingResponse

from src.models.translate import TranslateRequest, TranslateResponse
from src.services.groq_llm import get_groq_service

router = APIRouter()

TRANSLATION_SYSTEM_PROMPT = """You are a translation assistant for an interactive textbook on Physical AI and Humanoid Robotics.

## Your Task
Translate the provided content from English to {target_language}.

## Critical Rules - Follow Exactly:

1. **Preserve Technical Terms in English** - Keep these terms in English, optionally add {target_language} explanation in brackets:
   - AI, Artificial Intelligence, Machine Learning, Deep Learning, Neural Network
   - ROS, ROS 2, Gazebo, Isaac Sim, NVIDIA
   - Robot, Robotics, Humanoid, Sensor, Actuator, Motor
   - API, SDK, Framework, Node, Topic, Service
   - URDF, SDF, TF, Transform
   - Python, C++, Bash, YAML, JSON
   - GPU, CPU, CUDA, Tensor
   - Any code snippets, file paths, or commands

2. **Preserve All Formatting**:
   - Keep markdown headings (#, ##, ###)
   - Keep bullet points (-, *, 1.)
   - Keep code blocks (```python, ```bash, etc.)
   - Keep bold (**text**) and italic (*text*)
   - Keep links [text](url)
   - Keep equations and mathematical notation

3. **Do NOT**:
   - Summarize or shorten the content
   - Add explanations not in the original
   - Modify code examples
   - Change the structure or order
   - Translate code, commands, or file names

4. **Output**: Return ONLY the translated text, maintaining exact same structure.
"""


def build_translation_prompt(content: str, target_language: str, chapter_title: str) -> str:
    """Build the user prompt for translation."""
    prompt = f"Translate the following chapter"
    if chapter_title:
        prompt += f' "{chapter_title}"'
    prompt += f" to {target_language}:\n\n{content}"
    return prompt


async def generate_translation_stream(
    request: TranslateRequest,
) -> AsyncGenerator[str, None]:
    """Generate streaming translation response."""
    llm_service = get_groq_service()

    system_prompt = TRANSLATION_SYSTEM_PROMPT.format(
        target_language=request.target_language.capitalize()
    )

    user_prompt = build_translation_prompt(
        content=request.content,
        target_language=request.target_language,
        chapter_title=request.chapter_title,
    )

    messages = [{"role": "user", "content": user_prompt}]

    try:
        async for chunk in llm_service.stream_response(system_prompt, messages):
            # Stream in Vercel AI SDK format
            yield f'0:{json.dumps(chunk)}\n'

        yield f'e:{json.dumps({"finishReason": "stop"})}\n'
        yield f'd:{json.dumps({"finishReason": "stop"})}\n'

    except Exception as e:
        error_msg = f"Translation error: {str(e)}"
        yield f'0:{json.dumps(error_msg)}\n'
        yield f'e:{json.dumps({"finishReason": "error", "error": str(e)})}\n'


@router.post("/api/translate")
async def translate(request: TranslateRequest):
    """
    Translate chapter content to target language (default: Urdu).

    Uses Groq LLM with streaming response for real-time translation.
    Preserves technical terms, formatting, and code blocks.
    """
    if not request.content.strip():
        raise HTTPException(status_code=400, detail="Content cannot be empty")

    return StreamingResponse(
        generate_translation_stream(request),
        media_type="text/event-stream",
        headers={
            "Cache-Control": "no-cache",
            "Connection": "keep-alive",
            "X-Accel-Buffering": "no",
        },
    )


@router.post("/api/translate/sync")
async def translate_sync(request: TranslateRequest) -> TranslateResponse:
    """
    Non-streaming translation endpoint for simpler integrations.

    Returns complete translation at once.
    """
    if not request.content.strip():
        raise HTTPException(status_code=400, detail="Content cannot be empty")

    llm_service = get_groq_service()

    system_prompt = TRANSLATION_SYSTEM_PROMPT.format(
        target_language=request.target_language.capitalize()
    )

    user_prompt = build_translation_prompt(
        content=request.content,
        target_language=request.target_language,
        chapter_title=request.chapter_title,
    )

    messages = [{"role": "user", "content": user_prompt}]

    try:
        translated = llm_service.generate_response(system_prompt, messages)

        return TranslateResponse(
            translated_content=translated,
            source_language="english",
            target_language=request.target_language,
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Translation failed: {str(e)}")
