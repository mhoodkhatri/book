"""Pydantic models for translation API."""

from pydantic import BaseModel, Field


class TranslateRequest(BaseModel):
    """Request body for the translation endpoint."""

    content: str = Field(..., description="Chapter content to translate (markdown)")
    target_language: str = Field(
        default="urdu",
        alias="targetLanguage",
        description="Target language for translation"
    )
    chapter_title: str = Field(
        default="",
        alias="chapterTitle",
        description="Chapter title for context"
    )

    class Config:
        populate_by_name = True


class TranslateResponse(BaseModel):
    """Response for translation endpoint."""

    translated_content: str = Field(..., alias="translatedContent", description="Translated content")
    source_language: str = Field(default="english", alias="sourceLanguage")
    target_language: str = Field(alias="targetLanguage")

    class Config:
        populate_by_name = True
