"""Pydantic models for the translation feature."""

from pydantic import BaseModel, Field


class TranslationRequest(BaseModel):
    """Request to translate chapter content."""

    chapter_id: str = Field(
        ...,
        description="Unique chapter identifier",
        examples=["module-1-ros2/nodes-topics"],
        min_length=1,
        max_length=255,
    )

    chapter_title: str = Field(
        ...,
        description="Human-readable chapter title",
        examples=["Nodes and Topics"],
        min_length=1,
        max_length=500,
    )

    content: str = Field(
        ...,
        description="HTML content to translate",
        min_length=1,
        max_length=500000,
    )

    target_language: str = Field(
        default="urdu",
        description="Target language code",
        pattern=r"^(urdu|ur)$",
    )

    class Config:
        populate_by_name = True


class TranslationResponse(BaseModel):
    """Response containing translated content."""

    success: bool = Field(
        ...,
        description="Whether translation succeeded",
    )

    chapter_id: str = Field(
        ...,
        description="Chapter identifier (echoed from request)",
    )

    translated_content: str | None = Field(
        default=None,
        description="Translated HTML content",
    )

    source_language: str = Field(
        default="english",
        description="Source language of content",
    )

    target_language: str = Field(
        default="urdu",
        description="Target language of translation",
    )

    error: str | None = Field(
        default=None,
        description="Error message if translation failed",
    )

    metadata: dict | None = Field(
        default=None,
        description="Additional metadata (tokens used, latency, etc.)",
    )
