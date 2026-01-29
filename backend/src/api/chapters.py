"""API endpoint for chapter metadata."""

from fastapi import APIRouter, HTTPException

from src.models.chapter import Chapter, ChapterListResponse
from src.services.qdrant import get_qdrant_service

router = APIRouter()


@router.get("/api/chapters", response_model=ChapterListResponse)
async def get_chapters():
    """
    Get list of indexed chapters.

    Returns basic metadata about all chapters that have been indexed.
    """
    try:
        qdrant = get_qdrant_service()

        # For now, return collection info
        # In production, you'd query distinct chapter_ids
        info = qdrant.get_collection_info()

        # Placeholder: In a real implementation, you'd query distinct chapters
        # from Qdrant or a separate metadata store
        chapters = []

        return ChapterListResponse(chapters=chapters)

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to get chapters: {str(e)}")


@router.get("/api/chapters/{chapter_id:path}")
async def get_chapter(chapter_id: str):
    """
    Get metadata for a specific chapter.

    Args:
        chapter_id: The chapter URL slug (e.g., 'module-1-ros2/nodes-topics')
    """
    try:
        qdrant = get_qdrant_service()

        # Search for any chunk from this chapter to verify it exists
        # Using a dummy query to get metadata
        from src.services.embedding import get_embedding_service
        embedding = get_embedding_service()

        # Get one chunk to verify chapter exists
        results = qdrant.search(
            query_vector=embedding.embed_text("chapter content"),
            chapter_id=chapter_id,
            top_k=1,
        )

        if not results:
            raise HTTPException(status_code=404, detail=f"Chapter not found: {chapter_id}")

        chunk = results[0]
        return Chapter(
            id=chapter_id,
            title=chunk.get("chapter_title", ""),
            module=chunk.get("chapter_id", "").split("/")[0],
            chunk_count=1,  # Would need to count properly in production
        )

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to get chapter: {str(e)}")
