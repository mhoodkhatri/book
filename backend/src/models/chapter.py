"""Pydantic models for chapter metadata."""

from pydantic import BaseModel, Field


class Chapter(BaseModel):
    """Chapter metadata."""

    id: str = Field(..., description="Chapter URL slug (e.g., 'module-1-ros2/nodes-topics')")
    title: str = Field(..., description="Chapter title")
    module: str = Field(..., description="Module the chapter belongs to")
    chunk_count: int = Field(default=0, alias="chunkCount", description="Number of indexed chunks")

    class Config:
        populate_by_name = True


class ChapterListResponse(BaseModel):
    """Response for GET /api/chapters."""

    chapters: list[Chapter]


class IndexRequest(BaseModel):
    """Request body for indexing a chapter."""

    chapter_id: str = Field(..., alias="chapterId", description="Chapter URL slug")
    content: str = Field(..., description="Markdown content to index")
    reindex: bool = Field(default=False, description="Delete existing chunks first")

    class Config:
        populate_by_name = True


class IndexResponse(BaseModel):
    """Response for POST /api/index."""

    success: bool
    chunks_created: int = Field(..., alias="chunksCreated")
    chapter_id: str = Field(..., alias="chapterId")

    class Config:
        populate_by_name = True
