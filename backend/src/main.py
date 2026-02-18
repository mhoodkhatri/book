"""FastAPI application entry point."""

from contextlib import asynccontextmanager

import asyncio

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from src.config import get_settings
from src.api import chat, chapters, index, translate
from src.middleware.auth import get_db_pool, close_db_pool
from src.services.qdrant import get_qdrant_service

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Application lifespan: initialize and cleanup resources."""
    import os, logging
    logger = logging.getLogger(__name__)
    if os.getenv("DATABASE_URL"):
        try:
            await get_db_pool()
        except Exception as e:
            logger.warning("Could not connect to auth DB on startup (will retry on first request): %s", e)
    yield
    await close_db_pool()


# Create FastAPI app
app = FastAPI(
    title="RAG Chatbot API",
    description="Context-aware RAG chatbot for Physical AI textbook",
    version="0.1.0",
    lifespan=lifespan,
)

# Configure CORS
settings = get_settings()
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(chat.router, tags=["chat"])
app.include_router(chapters.router, tags=["chapters"])
app.include_router(index.router, tags=["index"])
app.include_router(translate.router, tags=["translation"])


@app.get("/")
async def root():
    """Health check endpoint."""
    return {"status": "healthy", "service": "rag-chatbot"}


@app.get("/health")
async def health():
    """Detailed health check with actual connectivity probes."""
    checks: dict = {}

    # Qdrant connectivity
    try:
        qdrant = get_qdrant_service()
        await asyncio.to_thread(qdrant.client.get_collection, settings.qdrant_collection)
        checks["qdrant"] = "ok"
    except Exception:
        checks["qdrant"] = "error"

    # DB connectivity (non-fatal — auth is optional if DATABASE_URL not set)
    try:
        pool = await get_db_pool()
        await pool.fetchval("SELECT 1")
        checks["database"] = "ok"
    except Exception:
        checks["database"] = "degraded"

    # Groq API key present
    checks["groq"] = "configured" if settings.groq_api_key else "missing"

    status = "healthy" if checks.get("qdrant") == "ok" else "degraded"
    return {"status": status, "version": "0.1.0", "services": checks}


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(
        "src.main:app",
        host=settings.host,
        port=settings.port,
        reload=True,
    )
