"""Translation service using Groq LLM and BeautifulSoup for HTML parsing."""

import time
import logging
from bs4 import BeautifulSoup

from src.services.groq_llm import get_groq_service

logger = logging.getLogger(__name__)

# Technical terms that must remain in English within Urdu text
PRESERVE_TERMS = [
    "ROS", "ROS 2", "ROS2", "Gazebo", "Isaac Sim", "NVIDIA",
    "URDF", "SLAM", "Nav2", "lidar", "LiDAR", "IMU", "API",
    "SDK", "Python", "C++", "C#", "JavaScript", "TypeScript",
    "Linux", "Ubuntu", "Docker", "Git", "GitHub", "CMake",
    "TCP", "UDP", "HTTP", "REST", "JSON", "YAML", "XML",
    "GPU", "CPU", "RAM", "SSD", "USB", "HDMI", "WiFi",
    "TensorFlow", "PyTorch", "OpenCV", "NumPy", "RViz",
    "MoveIt", "Humble", "Jazzy", "Foxy",
]

# Tags whose content should never be translated
SKIP_TAGS = {"pre", "code", "script", "style", "svg", "math"}

TRANSLATION_PROMPT = """Translate the following HTML content from English to Urdu.

CRITICAL RULES:
1. Preserve ALL HTML tags exactly as they are — do not add, remove, or modify any tags
2. Do NOT translate text inside <pre> or <code> tags (these have been removed and will be restored)
3. Keep these technical terms in English: {terms}
4. Maintain the same heading hierarchy (h1, h2, h3, h4, etc.)
5. Preserve all class names, id attributes, and other HTML attributes exactly
6. Preserve all image tags, video tags, and iframe tags exactly
7. Return ONLY the translated HTML — no explanations, no markdown fences, no preamble
8. Ensure the translated Urdu text is natural and readable
9. Preserve bold (<strong>) and italic (<em>) emphasis on corresponding Urdu words
10. Preserve list structure (<ul>, <ol>, <li>) and table structure (<table>, <tr>, <td>, <th>)

Content to translate:
{content}"""


class TranslatorService:
    """Service for translating HTML content from English to Urdu."""

    def __init__(self):
        self.groq = get_groq_service()

    def _extract_and_placeholder_skipped(self, html: str) -> tuple[str, dict[str, str]]:
        """
        Replace content of skip-tags (pre, code, etc.) with placeholders.
        Returns modified HTML and a mapping of placeholder -> original content.

        Processes outermost skip-tags first and skips nested ones (e.g. <code>
        inside <pre>) to avoid operating on detached elements.
        """
        soup = BeautifulSoup(html, "html.parser")
        placeholders = {}
        counter = 0

        for tag in list(soup.find_all(SKIP_TAGS)):
            # Skip tags that are no longer part of the tree (nested inside
            # a parent skip-tag that was already replaced).
            if tag.parent is None:
                continue
            placeholder_id = f"__SKIP_BLOCK_{counter}__"
            placeholders[placeholder_id] = str(tag)
            tag.replace_with(BeautifulSoup(placeholder_id, "html.parser"))
            counter += 1

        return str(soup), placeholders

    def _restore_placeholders(self, html: str, placeholders: dict[str, str]) -> str:
        """Restore original skip-tag content from placeholders."""
        result = html
        for placeholder_id, original in placeholders.items():
            result = result.replace(placeholder_id, original)
        return result

    def _build_prompt(self, content: str) -> str:
        """Build the translation prompt with technical terms list."""
        terms = ", ".join(PRESERVE_TERMS)
        return TRANSLATION_PROMPT.format(terms=terms, content=content)

    # Chunk size in chars (~1 token ≈ 4 chars). 3000 chars ≈ 750 tokens.
    # Keeps prompt + max_tokens under Groq free-tier 6000 TPM limit.
    CHUNK_CHAR_LIMIT = 3000

    def _chunk_html(self, html: str) -> list[str]:
        """Split HTML into chunks under CHUNK_CHAR_LIMIT chars, breaking at top-level tags."""
        soup = BeautifulSoup(html, "html.parser")
        chunks: list[str] = []
        current = ""
        for el in soup.find_all(recursive=False) if soup.find_all(recursive=False) else [soup]:
            piece = str(el)
            if len(current) + len(piece) > self.CHUNK_CHAR_LIMIT and current:
                chunks.append(current)
                current = piece
            else:
                current += piece
        if current:
            chunks.append(current)
        return chunks if chunks else [html]

    async def translate(self, content: str, chapter_title: str = "") -> tuple[str, dict]:
        import re as _re
        start_time = time.time()

        processable_html, placeholders = self._extract_and_placeholder_skipped(content)

        system_prompt = (
            "You are a professional English-to-Urdu translator specializing in "
            "technical and educational content. You translate HTML content while "
            "preserving all HTML structure and formatting exactly."
        )

        chunks = self._chunk_html(processable_html)
        translated_parts: list[str] = []
        for i, chunk in enumerate(chunks):
            prompt = self._build_prompt(chunk)
            messages = [{"role": "user", "content": prompt}]
            # Output budget: chars/4 (rough tokens) * 1.4 (Urdu expansion). Cap 1500.
            max_out = min(1500, max(256, int(len(chunk) / 4 * 1.4)))

            for attempt in range(3):
                try:
                    part = self.groq.generate_response(
                        system_prompt=system_prompt,
                        messages=messages,
                        max_tokens=max_out,
                        temperature=0.3,
                        model="llama-3.1-8b-instant",
                    )
                    break
                except Exception as e:
                    msg = str(e)
                    m = _re.search(r"try again in ([\d.]+)s", msg)
                    if "rate_limit" in msg.lower() or "429" in msg or "413" in msg:
                        wait = float(m.group(1)) if m else (8 * (attempt + 1))
                        wait = min(wait + 1, 30)
                        logger.warning(f"chunk {i}: rate limited, waiting {wait:.1f}s (attempt {attempt+1}/3)")
                        time.sleep(wait)
                        continue
                    raise
            else:
                raise RuntimeError(f"chunk {i}: rate-limited after 3 retries")

            translated_parts.append(self._clean_llm_response(part))

        translated_html = "".join(translated_parts)
        final_html = self._restore_placeholders(translated_html, placeholders)

        elapsed_ms = int((time.time() - start_time) * 1000)
        metadata = {
            "latency_ms": elapsed_ms,
            "model": "llama-3.1-8b-instant",
            "placeholders_restored": len(placeholders),
            "chunks_translated": len(chunks),
        }

        return final_html, metadata

    def _clean_llm_response(self, response: str) -> str:
        """Remove markdown code fences and extra whitespace from LLM response."""
        response = response.strip()
        # Remove ```html ... ``` wrapper if present
        if response.startswith("```html"):
            response = response[len("```html"):].strip()
        if response.startswith("```"):
            response = response[3:].strip()
        if response.endswith("```"):
            response = response[:-3].strip()
        return response


# Singleton instance
_translator_service: TranslatorService | None = None


def get_translator_service() -> TranslatorService:
    """Get or create the TranslatorService singleton."""
    global _translator_service
    if _translator_service is None:
        _translator_service = TranslatorService()
    return _translator_service
