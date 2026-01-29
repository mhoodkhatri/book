"""Pydantic models for chat API requests and responses."""

from pydantic import BaseModel, Field


class Message(BaseModel):
    """A single chat message."""

    role: str = Field(..., description="Message role: 'user' or 'assistant'")
    content: str = Field(..., description="Message content")


class ChatRequest(BaseModel):
    """Request body for the chat endpoint."""

    messages: list[Message] = Field(..., description="Conversation history")
    chapter_id: str = Field(..., alias="chapterId", description="Current chapter URL slug")
    chapter_title: str = Field(..., alias="chapterTitle", description="Current chapter title")
    selected_text: str | None = Field(
        None, alias="selectedText", description="User-selected text for context"
    )

    class Config:
        populate_by_name = True


class ChatResponse(BaseModel):
    """Response for non-streaming chat."""

    content: str = Field(..., description="Assistant response")
    citations: list[dict] = Field(default_factory=list, description="Source citations")


class ChunkResult(BaseModel):
    """A retrieved chunk from the vector store."""

    content: str
    section_heading: str
    chapter_id: str
    chapter_title: str
    url: str
    score: float
