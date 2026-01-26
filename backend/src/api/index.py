"""API endpoint for indexing chapter content."""

from fastapi import APIRouter, HTTPException

from src.models.chapter import IndexRequest, IndexResponse
from src.services.indexer import get_indexer

router = APIRouter()


@router.post("/api/index", response_model=IndexResponse)
async def index_chapter(request: IndexRequest):
    """
    Index a chapter's content into the vector store.

    This endpoint chunks the markdown content, generates embeddings,
    and stores them in Qdrant with chapter metadata for filtering.

    Args:
        request: Contains chapter_id, content (markdown), and reindex flag
    """
    try:
        indexer = get_indexer()

        # Extract chapter title from content or use ID as fallback
        lines = request.content.strip().split('\n')
        chapter_title = request.chapter_id
        for line in lines:
            if line.startswith('# '):
                chapter_title = line[2:].strip()
                break

        # Index the chapter
        chunks_created = indexer.index_chapter(
            chapter_id=request.chapter_id,
            chapter_title=chapter_title,
            content=request.content,
            reindex=request.reindex,
        )

        return IndexResponse(
            success=True,
            chunks_created=chunks_created,
            chapter_id=request.chapter_id,
        )

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to index chapter: {str(e)}")


@router.delete("/api/index/{chapter_id:path}")
async def delete_chapter_index(chapter_id: str):
    """
    Delete all indexed content for a chapter.

    Args:
        chapter_id: The chapter URL slug to delete
    """
    try:
        from src.services.qdrant import get_qdrant_service

        qdrant = get_qdrant_service()
        qdrant.delete_chapter(chapter_id)

        return {"success": True, "chapter_id": chapter_id, "message": "Chapter index deleted"}

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to delete chapter index: {str(e)}")
