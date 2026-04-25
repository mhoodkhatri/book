import React, { useState, useCallback, useRef } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './styles.module.css';

interface TranslateButtonProps {
  chapterTitle: string;
}

export function TranslateButton({ chapterTitle }: TranslateButtonProps): React.JSX.Element {
  const { siteConfig } = useDocusaurusContext();
  const chatApiUrl = (siteConfig.customFields?.chatApiUrl as string) || 'http://localhost:8000';

  const [isUrdu, setIsUrdu] = useState(false);
  const [isTranslating, setIsTranslating] = useState(false);
  const [error, setError] = useState<string | null>(null);

  // Store original English content
  const originalContentRef = useRef<string | null>(null);
  const articleRef = useRef<HTMLElement | null>(null);

  const getArticleElement = (): HTMLElement | null => {
    return document.querySelector('article .markdown');
  };

  const extractTextContent = (element: HTMLElement): string => {
    // Clone the element to work with
    const clone = element.cloneNode(true) as HTMLElement;

    // Mark code blocks with placeholders to preserve them
    const codeBlocks = clone.querySelectorAll('pre, code');
    const codePlaceholders: string[] = [];

    codeBlocks.forEach((block, index) => {
      const placeholder = `___CODE_BLOCK_${index}___`;
      codePlaceholders.push(block.outerHTML);
      block.outerHTML = placeholder;
    });

    // Get the HTML content (preserving structure)
    let content = clone.innerHTML;

    return content;
  };

  const translateContent = useCallback(async () => {
    const article = getArticleElement();
    if (!article) {
      setError('Content not found');
      return;
    }

    articleRef.current = article;

    // Store original content before first translation
    if (!originalContentRef.current) {
      originalContentRef.current = article.innerHTML;
    }

    setIsTranslating(true);
    setError(null);

    // Extract text while preserving code blocks
    const clone = article.cloneNode(true) as HTMLElement;
    const codeBlocks: { placeholder: string; html: string }[] = [];

    // Replace code blocks with placeholders
    clone.querySelectorAll('pre').forEach((block, index) => {
      const placeholder = `[[CODE_BLOCK_${index}]]`;
      codeBlocks.push({ placeholder, html: block.outerHTML });
      const placeholderNode = document.createTextNode(placeholder);
      block.parentNode?.replaceChild(placeholderNode, block);
    });

    const contentToTranslate = clone.innerHTML;

    try {
      const response = await fetch(`${chatApiUrl}/api/translate`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          content: contentToTranslate,
          targetLanguage: 'urdu',
          chapterTitle: chapterTitle,
        }),
      });

      if (!response.ok) throw new Error(`HTTP error: ${response.status}`);

      const reader = response.body?.getReader();
      if (!reader) throw new Error('No response body');

      const decoder = new TextDecoder();
      let translatedContent = '';

      while (true) {
        const { done, value } = await reader.read();
        if (done) break;

        const chunk = decoder.decode(value, { stream: true });
        const lines = chunk.split('\n').filter(line => line.trim());

        for (const line of lines) {
          if (line.startsWith('0:')) {
            try {
              const text = JSON.parse(line.slice(2));
              translatedContent += text;
            } catch {
              // Ignore parse errors
            }
          }
        }
      }

      // Restore code blocks in translated content
      codeBlocks.forEach(({ placeholder, html }) => {
        translatedContent = translatedContent.replace(placeholder, html);
      });

      // Apply translated content
      article.innerHTML = translatedContent;
      article.setAttribute('dir', 'rtl');
      article.setAttribute('lang', 'ur');
      article.classList.add(styles.urduContent);

      setIsUrdu(true);
    } catch (err) {
      console.error('Translation error:', err);
      setError('Translation failed. Please try again.');
    } finally {
      setIsTranslating(false);
    }
  }, [chatApiUrl, chapterTitle]);

  const restoreEnglish = useCallback(() => {
    const article = articleRef.current || getArticleElement();
    if (article && originalContentRef.current) {
      article.innerHTML = originalContentRef.current;
      article.setAttribute('dir', 'ltr');
      article.setAttribute('lang', 'en');
      article.classList.remove(styles.urduContent);
      setIsUrdu(false);
      setError(null);
    }
  }, []);

  const handleClick = () => {
    if (isTranslating) return;

    if (isUrdu) {
      restoreEnglish();
    } else {
      translateContent();
    }
  };

  return (
    <button
      className={`${styles.translateButton} ${isUrdu ? styles.active : ''} ${isTranslating ? styles.loading : ''}`}
      onClick={handleClick}
      disabled={isTranslating}
      title={isUrdu ? 'Switch to English' : 'Translate to Urdu'}
    >
      <span className={styles.icon}>
        {isTranslating ? '⏳' : '🌐'}
      </span>
      <span className={styles.text}>
        {isTranslating
          ? 'Translating...'
          : isUrdu
            ? 'English'
            : 'اردو'}
      </span>
      {error && <span className={styles.errorDot} title={error}>!</span>}
    </button>
  );
}

export default TranslateButton;
