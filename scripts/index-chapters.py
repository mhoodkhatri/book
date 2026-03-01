"""Index all book chapters into Qdrant via Railway backend."""
import json
import pathlib
import urllib.request
import urllib.error

BACKEND_URL = "https://backend-production-3cb7.up.railway.app"
DOCS_DIR = pathlib.Path(__file__).parent.parent / "docs"

def index_chapter(chapter_id: str, content: str) -> dict:
    payload = json.dumps({
        "chapter_id": chapter_id,
        "content": content,
        "reindex": True,
    }).encode("utf-8")
    req = urllib.request.Request(
        f"{BACKEND_URL}/api/index",
        data=payload,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=30) as resp:
        return json.loads(resp.read())

def main():
    md_files = sorted(DOCS_DIR.rglob("*.md"))
    print(f"Found {len(md_files)} chapters to index.\n")
    success, failed = 0, 0
    for f in md_files:
        chapter_id = str(f.relative_to(DOCS_DIR).with_suffix("")).replace("\\", "/")
        content = f.read_text(encoding="utf-8")
        print(f"  Indexing {chapter_id} ...", end=" ", flush=True)
        try:
            result = index_chapter(chapter_id, content)
            chunks = result.get("chunks_created", "?")
            print(f"OK ({chunks} chunks)")
            success += 1
        except urllib.error.HTTPError as e:
            body = e.read().decode()
            print(f"FAILED ({e.code}): {body}")
            failed += 1
        except Exception as e:
            print(f"FAILED: {e}")
            failed += 1

    print(f"\nDone: {success} indexed, {failed} failed.")

if __name__ == "__main__":
    main()
