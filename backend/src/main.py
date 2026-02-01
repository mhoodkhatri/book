"""FastAPI application entry point."""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from src.config import get_settings
from src.api import chat, chapters, index, translate

# Create FastAPI app
app = FastAPI(
    title="RAG Chatbot API",
    description="Context-aware RAG chatbot for Physical AI textbook",
    version="0.1.0",
)

# Configure CORS
settings = get_settings()
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins + ["*"],  # Allow all for development
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(chat.router, tags=["chat"])
app.include_router(chapters.router, tags=["chapters"])
app.include_router(index.router, tags=["index"])
app.include_router(translate.router, tags=["translate"])


@app.get("/")
async def root():
    """Health check endpoint."""
    return {"status": "healthy", "service": "rag-chatbot"}


@app.get("/health")
async def health():
    """Detailed health check."""
    return {
        "status": "healthy",
        "version": "0.1.0",
        "services": {
            "gemini": "configured",
            "qdrant": "configured",
            "embeddings": "configured",
        },
    }


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(
        "src.main:app",
        host=settings.host,
        port=settings.port,
        reload=True,
    )
