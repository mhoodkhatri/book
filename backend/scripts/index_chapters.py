#!/usr/bin/env python3
"""CLI script for indexing chapter content from the docs directory."""

import argparse
import os
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from dotenv import load_dotenv

load_dotenv()


def find_chapters(docs_dir: Path) -> list[tuple[str, Path]]:
    """
    Find all markdown files that represent chapters.

    Returns list of (chapter_id, file_path) tuples.
    """
    chapters = []

    for md_file in docs_dir.rglob("*.md"):
        # Skip index files and special files
        if md_file.name in ["index.md", "_category_.json", "README.md"]:
            continue

        # Create chapter_id from relative path
        rel_path = md_file.relative_to(docs_dir)
        chapter_id = str(rel_path.with_suffix("")).replace("\\", "/")

        chapters.append((chapter_id, md_file))

    return chapters


def index_chapter(chapter_id: str, file_path: Path, reindex: bool = False) -> int:
    """Index a single chapter."""
    from src.services.indexer import get_indexer

    content = file_path.read_text(encoding="utf-8")

    # Extract title from first H1
    title = chapter_id
    for line in content.split("\n"):
        if line.startswith("# "):
            title = line[2:].strip()
            break

    indexer = get_indexer()
    chunks = indexer.index_chapter(
        chapter_id=chapter_id,
        chapter_title=title,
        content=content,
        reindex=reindex,
    )

    return chunks


def main():
    parser = argparse.ArgumentParser(description="Index textbook chapters into Qdrant")
    parser.add_argument(
        "--docs-dir",
        type=Path,
        default=Path(__file__).parent.parent.parent / "docs",
        help="Path to docs directory",
    )
    parser.add_argument(
        "--chapter",
        type=str,
        help="Index only a specific chapter by ID",
    )
    parser.add_argument(
        "--reindex",
        action="store_true",
        help="Delete existing chunks before indexing",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="List chapters without indexing",
    )

    args = parser.parse_args()

    if not args.docs_dir.exists():
        print(f"ERROR: Docs directory not found: {args.docs_dir}")
        return 1

    print(f"Docs directory: {args.docs_dir}")

    # Find chapters
    chapters = find_chapters(args.docs_dir)
    print(f"Found {len(chapters)} chapters")

    if args.chapter:
        # Filter to specific chapter
        chapters = [(cid, path) for cid, path in chapters if cid == args.chapter]
        if not chapters:
            print(f"ERROR: Chapter not found: {args.chapter}")
            return 1

    if args.dry_run:
        print("\nChapters to index (dry run):")
        for chapter_id, path in chapters:
            print(f"  {chapter_id}")
        return 0

    # Ensure Qdrant collection exists
    from src.services.qdrant import get_qdrant_service

    qdrant = get_qdrant_service()
    created = qdrant.ensure_collection()
    if created:
        print("Created Qdrant collection: textbook_chunks")

    # Index chapters
    total_chunks = 0
    for chapter_id, path in chapters:
        print(f"\nIndexing: {chapter_id}")
        try:
            chunks = index_chapter(chapter_id, path, args.reindex)
            print(f"  Created {chunks} chunks")
            total_chunks += chunks
        except Exception as e:
            print(f"  ERROR: {e}")

    print(f"\n{'=' * 50}")
    print(f"Total: {total_chunks} chunks indexed from {len(chapters)} chapters")

    return 0


if __name__ == "__main__":
    sys.exit(main())
