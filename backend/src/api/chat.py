"""Chat API endpoint with streaming responses."""

import json
from typing import AsyncGenerator

from fastapi import APIRouter, HTTPException
from fastapi.responses import StreamingResponse

from src.models.chat import ChatRequest
from src.services.rag import get_rag_service
from src.services.groq_llm import get_groq_service
from src.services.prompts import get_prompt_builder

router = APIRouter()


async def generate_sse_stream(
    request: ChatRequest,
) -> AsyncGenerator[str, None]:
    """
    Generate Server-Sent Events stream for chat response.

    Follows the Vercel AI SDK streaming format:
    - 0:text - Text chunks
    - e:data - Completion event
    - d:data - Done event
    """
    rag_service = get_rag_service()
    llm_service = get_groq_service()
    prompt_builder = get_prompt_builder()

    # Extract the user's question
    user_message = request.messages[-1].content if request.messages else ""

    # Quick off-topic check
    if prompt_builder.is_likely_off_topic(user_message):
        refusal = prompt_builder.build_refusal_response(
            reason="off_topic",
            chapter_title=request.chapter_title,
        )
        # Stream the refusal as a single chunk
        yield f'0:{json.dumps(refusal)}\n'
        yield f'e:{json.dumps({"finishReason": "stop"})}\n'
        yield f'd:{json.dumps({"finishReason": "stop"})}\n'
        return

    # Retrieve relevant context from the chapter
    chunks = rag_service.retrieve(
        query=user_message,
        chapter_id=request.chapter_id,
        top_k=5,
    )

    # Check if question is answerable
    is_answerable, reason = rag_service.is_question_answerable(chunks)

    if not is_answerable:
        refusal = prompt_builder.build_refusal_response(
            reason=reason,
            chapter_title=request.chapter_title,
            chunks=chunks,
        )
        yield f'0:{json.dumps(refusal)}\n'
        yield f'e:{json.dumps({"finishReason": "stop"})}\n'
        yield f'd:{json.dumps({"finishReason": "stop"})}\n'
        return

    # Build context from chunks
    context = rag_service.get_context_for_prompt(chunks)

    # Build system prompt
    system_prompt = prompt_builder.build_system_prompt(
        chapter_id=request.chapter_id,
        chapter_title=request.chapter_title,
        context=context,
        selected_text=request.selected_text,
    )

    # Convert messages to dict format for Gemini
    messages = [{"role": m.role, "content": m.content} for m in request.messages]

    # Stream response from Gemini
    try:
        async for chunk in llm_service.stream_response(system_prompt, messages):
            # Escape the chunk for JSON and SSE
            yield f'0:{json.dumps(chunk)}\n'

        # Send completion events
        yield f'e:{json.dumps({"finishReason": "stop"})}\n'
        yield f'd:{json.dumps({"finishReason": "stop"})}\n'

    except Exception as e:
        error_msg = f"I encountered an error while generating a response. Please try again."
        yield f'0:{json.dumps(error_msg)}\n'
        yield f'e:{json.dumps({"finishReason": "error", "error": str(e)})}\n'
        yield f'd:{json.dumps({"finishReason": "error"})}\n'


@router.post("/api/chat")
async def chat(request: ChatRequest):
    """
    Chat endpoint with streaming responses.

    Uses RAG to retrieve chapter-specific context and streams
    responses from Groq (Llama 3.3 70B).

    The response format follows the Vercel AI SDK streaming protocol
    for compatibility with the useChat hook.
    """
    try:
        return StreamingResponse(
            generate_sse_stream(request),
            media_type="text/event-stream",
            headers={
                "Cache-Control": "no-cache",
                "Connection": "keep-alive",
                "X-Accel-Buffering": "no",  # Disable nginx buffering
            },
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Chat error: {str(e)}")


@router.post("/api/chat/sync")
async def chat_sync(request: ChatRequest):
    """
    Non-streaming chat endpoint for testing.

    Returns the complete response at once.
    """
    rag_service = get_rag_service()
    llm_service = get_groq_service()
    prompt_builder = get_prompt_builder()

    user_message = request.messages[-1].content if request.messages else ""

    # Off-topic check
    if prompt_builder.is_likely_off_topic(user_message):
        return {
            "content": prompt_builder.build_refusal_response(
                reason="off_topic",
                chapter_title=request.chapter_title,
            ),
            "citations": [],
        }

    # Retrieve context
    chunks = rag_service.retrieve(
        query=user_message,
        chapter_id=request.chapter_id,
        top_k=5,
    )

    is_answerable, reason = rag_service.is_question_answerable(chunks)

    if not is_answerable:
        return {
            "content": prompt_builder.build_refusal_response(
                reason=reason,
                chapter_title=request.chapter_title,
                chunks=chunks,
            ),
            "citations": [],
        }

    # Build prompt and generate response
    context = rag_service.get_context_for_prompt(chunks)
    system_prompt = prompt_builder.build_system_prompt(
        chapter_id=request.chapter_id,
        chapter_title=request.chapter_title,
        context=context,
        selected_text=request.selected_text,
    )

    messages = [{"role": m.role, "content": m.content} for m in request.messages]
    response = llm_service.generate_response(system_prompt, messages)

    return {
        "content": response,
        "citations": rag_service.format_citations(chunks),
    }
