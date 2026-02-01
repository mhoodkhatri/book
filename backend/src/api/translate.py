"""Translation API endpoint for chapter content."""

import logging
import asyncio
from fastapi import APIRouter, Depends, HTTPException

from src.models.translation import TranslationRequest, TranslationResponse
from src.services.translator import get_translator_service
from src.middleware.auth import get_current_user

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api", tags=["translation"])

TRANSLATION_TIMEOUT_SECONDS = 60


@router.post("/translate", response_model=TranslationResponse)
async def translate_chapter(request: TranslationRequest, user: dict = Depends(get_current_user)) -> TranslationResponse:
    """
    Translate chapter content from English to Urdu.

    Accepts HTML content, preserves code blocks and technical terms,
    and returns translated Urdu HTML.
    """
    try:
        translator = get_translator_service()

        translated_content, metadata = await asyncio.wait_for(
            translator.translate(
                content=request.content,
                chapter_title=request.chapter_title,
            ),
            timeout=TRANSLATION_TIMEOUT_SECONDS,
        )

        return TranslationResponse(
            success=True,
            chapter_id=request.chapter_id,
            translated_content=translated_content,
            source_language="english",
            target_language=request.target_language,
            metadata=metadata,
        )

    except asyncio.TimeoutError:
        logger.error("Translation timed out for chapter: %s", request.chapter_id)
        raise HTTPException(
            status_code=504,
            detail="Translation request timed out. Try a shorter chapter.",
        )
    except ValueError as e:
        logger.error("Invalid translation request: %s", str(e))
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        logger.error("Translation failed for chapter %s: %s", request.chapter_id, str(e))
        raise HTTPException(
            status_code=500,
            detail="Translation service temporarily unavailable",
        )
