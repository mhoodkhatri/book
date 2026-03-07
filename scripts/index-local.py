"""Index all book chapters LOCALLY into Qdrant Cloud.

Runs fastembed on this machine and pushes vectors directly to Qdrant Cloud.
Does NOT go through Railway backend (avoids RAM issues).
"""
import json
import pathlib
import re
import uuid

from fastembed import TextEmbedding
from qdrant_client import QdrantClient
from qdrant_client.http.models import (
    Distance,
    PointStruct,
    VectorParams,
    Filter,
    FieldCondition,
    MatchValue,
)

# ---------- Config ----------
QDRANT_URL = "https://c2bf96c2-c416-4a0e-92e4-eac2e1457c06.europe-west3-0.gcp.cloud.qdrant.io"
QDRANT_API_KEY = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIiwiZXhwIjoxNzc3NTM1OTYwfQ.4arf_YxBt6K-QHNV8WSbWlE5zVlqMBKPNWHD4u8C6y8"
COLLECTION = "textbook_chunks"
EMBEDDING_MODEL = "BAAI/bge-small-en-v1.5"
EMBEDDING_DIM = 384
DOCS_DIR = pathlib.Path(__file__).parent.parent / "docs"


def chunk_markdown(markdown: str, chapter_id: str, chapter_title: str, target_tokens: int = 750):
    """Split markdown into semantic chunks by ## headings."""
    chunks = []
    module = chapter_id.split("/")[0] if "/" in chapter_id else chapter_id
    sections = re.split(r'\n(?=## )', markdown)

    chunk_index = 0
    for section in sections:
        if not section.strip():
            continue
        heading_match = re.match(r'^## (.+?)(?:\n|$)', section)
        section_heading = heading_match.group(1).strip() if heading_match else "Introduction"
        content = re.sub(r'^## .+?\n', '', section).strip()
        if not content:
            continue
        anchor = re.sub(r'[^a-z0-9]+', '-', section_heading.lower()).strip('-')
        url = f"/docs/{chapter_id}#{anchor}"

        # Split large sections
        target_chars = target_tokens * 4
        if len(content) <= target_chars:
            section_chunks = [content]
        else:
            section_chunks = []
            paragraphs = content.split('\n\n')
            current, current_len = [], 0
            for para in paragraphs:
                if current_len + len(para) > target_chars and current:
                    section_chunks.append('\n\n'.join(current))
                    current, current_len = [para], len(para)
                else:
                    current.append(para)
                    current_len += len(para)
            if current:
                section_chunks.append('\n\n'.join(current))

        for chunk_content in section_chunks:
            chunks.append({
                "id": str(uuid.uuid4()),
                "content": chunk_content,
                "chapter_id": chapter_id,
                "chapter_title": chapter_title,
                "module": module,
                "section_heading": section_heading,
                "chunk_index": chunk_index,
                "word_count": len(chunk_content.split()),
                "url": url,
            })
            chunk_index += 1
    return chunks


def main():
    # Init embedding model (downloads ~130MB on first run)
    print(f"Loading embedding model: {EMBEDDING_MODEL} ...")
    model = TextEmbedding(model_name=EMBEDDING_MODEL)
    print("Model loaded.\n")

    # Init Qdrant client
    client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

    # Ensure collection exists
    collections = [c.name for c in client.get_collections().collections]
    if COLLECTION not in collections:
        print(f"Creating collection '{COLLECTION}' (dim={EMBEDDING_DIM}) ...")
        client.create_collection(
            collection_name=COLLECTION,
            vectors_config=VectorParams(size=EMBEDDING_DIM, distance=Distance.COSINE),
        )
        client.create_payload_index(
            collection_name=COLLECTION,
            field_name="chapter_id",
            field_schema="keyword",
        )
        print("Collection created.\n")
    else:
        print(f"Collection '{COLLECTION}' exists.\n")

    # Find all chapters
    md_files = sorted(DOCS_DIR.rglob("*.md"))
    print(f"Found {len(md_files)} chapters to index.\n")

    total_chunks = 0
    success, failed = 0, []

    for i, f in enumerate(md_files, 1):
        chapter_id = str(f.relative_to(DOCS_DIR).with_suffix("")).replace("\\", "/")
        content = f.read_text(encoding="utf-8")

        # Extract title
        chapter_title = chapter_id
        for line in content.strip().split('\n'):
            if line.startswith('# '):
                chapter_title = line[2:].strip()
                break

        print(f"  [{i:02d}/{len(md_files)}] {chapter_id} ...", end=" ", flush=True)

        try:
            # Delete existing chunks for this chapter
            client.delete(
                collection_name=COLLECTION,
                points_selector=Filter(
                    must=[FieldCondition(key="chapter_id", match=MatchValue(value=chapter_id))]
                ),
            )

            # Chunk content
            chunks = chunk_markdown(content, chapter_id, chapter_title)
            if not chunks:
                print("SKIP (no content)")
                continue

            # Generate embeddings locally
            texts = [c["content"] for c in chunks]
            embeddings = list(model.embed(texts, batch_size=32))

            # Build points for Qdrant
            points = []
            for chunk, embedding in zip(chunks, embeddings):
                points.append(PointStruct(
                    id=chunk["id"],
                    vector=embedding.tolist(),
                    payload={k: v for k, v in chunk.items() if k != "id"},
                ))

            # Upsert to Qdrant Cloud
            client.upsert(collection_name=COLLECTION, points=points)

            total_chunks += len(chunks)
            success += 1
            print(f"OK ({len(chunks)} chunks)")

        except Exception as e:
            print(f"FAILED: {e}")
            failed.append(chapter_id)

    print(f"\n{'='*50}")
    print(f"Done: {success}/{len(md_files)} indexed, {total_chunks} total chunks.")
    if failed:
        print(f"\nFailed ({len(failed)}):")
        for c in failed:
            print(f"  - {c}")


if __name__ == "__main__":
    main()
