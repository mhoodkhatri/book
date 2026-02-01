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

    async def translate(self, content: str, chapter_title: str = "") -> tuple[str, dict]:
        """
        Translate HTML content from English to Urdu.

        Args:
            content: HTML content to translate
            chapter_title: Chapter title for context

        Returns:
            Tuple of (translated_html, metadata_dict)
        """
        start_time = time.time()

        # Step 1: Extract and placeholder skip-tags
        processable_html, placeholders = self._extract_and_placeholder_skipped(content)

        # Step 2: Build translation prompt
        prompt = self._build_prompt(processable_html)

        # Step 3: Call Groq LLM for translation
        messages = [{"role": "user", "content": prompt}]
        system_prompt = (
            "You are a professional English-to-Urdu translator specializing in "
            "technical and educational content. You translate HTML content while "
            "preserving all HTML structure and formatting exactly."
        )

        translated_html = self.groq.generate_response(
            system_prompt=system_prompt,
            messages=messages,
            max_tokens=8192,
            temperature=0.3,
        )

        # Step 4: Clean up LLM response (remove markdown fences if present)
        translated_html = self._clean_llm_response(translated_html)

        # Step 5: Restore placeholders with original skip-tag content
        final_html = self._restore_placeholders(translated_html, placeholders)

        elapsed_ms = int((time.time() - start_time) * 1000)
        metadata = {
            "latency_ms": elapsed_ms,
            "model": self.groq.model,
            "placeholders_restored": len(placeholders),
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
