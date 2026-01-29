#!/usr/bin/env python3
"""Test connections to external services."""

import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from dotenv import load_dotenv

load_dotenv()


def test_google_api():
    """Test Google AI API connection."""
    print("Testing Google AI API...")
    try:
        import google.generativeai as genai

        api_key = os.getenv("GOOGLE_API_KEY")
        if not api_key:
            print("  ERROR: GOOGLE_API_KEY not set")
            return False

        genai.configure(api_key=api_key)

        # Test embedding
        result = genai.embed_content(
            model="models/text-embedding-004",
            content="Hello, world!",
            task_type="retrieval_query",
        )
        print(f"  Embedding dimension: {len(result['embedding'])}")

        # Test Gemini
        model = genai.GenerativeModel("gemini-2.0-flash-exp")
        response = model.generate_content("Say 'Hello' in one word.")
        print(f"  Gemini response: {response.text[:50]}...")

        print("  SUCCESS: Google AI API connected")
        return True
    except Exception as e:
        print(f"  ERROR: {e}")
        return False


def test_qdrant():
    """Test Qdrant connection."""
    print("\nTesting Qdrant...")
    try:
        from qdrant_client import QdrantClient

        url = os.getenv("QDRANT_URL")
        api_key = os.getenv("QDRANT_API_KEY")

        if not url:
            print("  ERROR: QDRANT_URL not set")
            return False
        if not api_key:
            print("  ERROR: QDRANT_API_KEY not set")
            return False

        client = QdrantClient(url=url, api_key=api_key)
        collections = client.get_collections()
        print(f"  Collections: {[c.name for c in collections.collections]}")
        print("  SUCCESS: Qdrant connected")
        return True
    except Exception as e:
        print(f"  ERROR: {e}")
        return False


def main():
    """Run all connection tests."""
    print("=" * 50)
    print("RAG Chatbot - Service Connection Tests")
    print("=" * 50)

    results = {
        "Google AI": test_google_api(),
        "Qdrant": test_qdrant(),
    }

    print("\n" + "=" * 50)
    print("Summary:")
    for service, passed in results.items():
        status = "PASS" if passed else "FAIL"
        print(f"  {service}: {status}")

    all_passed = all(results.values())
    print("=" * 50)
    print(f"Overall: {'ALL TESTS PASSED' if all_passed else 'SOME TESTS FAILED'}")

    return 0 if all_passed else 1


if __name__ == "__main__":
    sys.exit(main())
