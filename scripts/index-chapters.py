"""Index all book chapters into Qdrant via Railway backend."""
import json
import pathlib
import time
import urllib.request
import urllib.error

BACKEND_URL = "https://backend-production-3cb7.up.railway.app"
DOCS_DIR = pathlib.Path(__file__).parent.parent / "docs"
REINDEX = False  # False = skip already-indexed chapters; True = force re-index all
REQUEST_TIMEOUT = 60  # seconds per chapter
RETRY_DELAYS = [5, 15, 30]  # seconds to wait before each retry


def wait_for_healthy(max_wait: int = 60) -> bool:
    """Wait until backend health check passes."""
    for _ in range(max_wait // 5):
        try:
            with urllib.request.urlopen(f"{BACKEND_URL}/health", timeout=10) as r:
                data = json.loads(r.read())
                if data.get("status") == "healthy":
                    return True
        except Exception:
            pass
        time.sleep(5)
    return False


def index_chapter(chapter_id: str, content: str) -> dict:
    payload = json.dumps({
        "chapter_id": chapter_id,
        "content": content,
        "reindex": REINDEX,
    }).encode("utf-8")
    req = urllib.request.Request(
        f"{BACKEND_URL}/api/index",
        data=payload,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=REQUEST_TIMEOUT) as resp:
        return json.loads(resp.read())


def index_with_retry(chapter_id: str, content: str) -> tuple[bool, str]:
    """Try indexing with retries on 502/timeout, waiting for backend recovery."""
    last_error = ""
    for attempt, delay in enumerate([0] + RETRY_DELAYS):
        if delay:
            print(f"    waiting {delay}s before retry {attempt}...", end=" ", flush=True)
            time.sleep(delay)
            if not wait_for_healthy(max_wait=30):
                print("backend still down, skipping")
                return False, "backend unavailable"
        try:
            result = index_chapter(chapter_id, content)
            chunks = result.get("chunks_created", 0)
            return True, f"{chunks} chunks"
        except urllib.error.HTTPError as e:
            body = e.read().decode()
            last_error = f"HTTP {e.code}: {body[:120]}"
            if e.code == 502:
                print(f"502 (backend crashed)", end=" ", flush=True)
                continue  # retry after delay
            return False, last_error
        except Exception as e:
            last_error = str(e)
            if "timed out" in last_error.lower() or "timeout" in last_error.lower():
                print(f"timeout", end=" ", flush=True)
                continue  # retry after delay
            return False, last_error
    return False, last_error


def main():
    md_files = sorted(DOCS_DIR.rglob("*.md"))
    print(f"Found {len(md_files)} chapters to index (reindex={REINDEX}).\n")

    print("Checking backend health...", end=" ")
    if not wait_for_healthy():
        print("FAILED — backend is not healthy. Aborting.")
        return
    print("OK\n")

    success, failed = 0, []
    for f in md_files:
        chapter_id = str(f.relative_to(DOCS_DIR).with_suffix("")).replace("\\", "/")
        content = f.read_text(encoding="utf-8")
        print(f"  [{success + len(failed) + 1:02d}/{len(md_files)}] {chapter_id} ...", end=" ", flush=True)

        ok, msg = index_with_retry(chapter_id, content)
        if ok:
            print(f"OK ({msg})")
            success += 1
        else:
            print(f"FAILED: {msg}")
            failed.append(chapter_id)

        # Brief pause between chapters to avoid overwhelming the container
        time.sleep(1)

    print(f"\n{'='*50}")
    print(f"Done: {success}/{len(md_files)} indexed successfully.")
    if failed:
        print(f"\nFailed chapters ({len(failed)}):")
        for c in failed:
            print(f"  - {c}")


if __name__ == "__main__":
    main()
