import React, { useState, useCallback, useEffect, useRef } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { getCachedTranslation, cacheTranslation } from './cache';
import { useAuthGuard } from '@site/src/hooks/useAuthGuard';
import LockedFeatureModal from '@site/src/components/Auth/LockedFeatureModal';
import styles from './styles.module.css';

interface Props {
  chapterId: string;
  chapterTitle: string;
}

type Language = 'english' | 'urdu';

const ERROR_DISMISS_MS = 5000;

export default function TranslationButton({
  chapterId,
  chapterTitle,
}: Props): React.JSX.Element {
  const { isAuthenticated, loginUrl } = useAuthGuard();
  const { siteConfig } = useDocusaurusContext();
  const apiUrl =
    (siteConfig.customFields?.chatApiUrl as string) || 'http://localhost:8000';

  const [language, setLanguage] = useState<Language>('english');
  const [isTranslating, setIsTranslating] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [showLockedModal, setShowLockedModal] = useState(false);
  const abortControllerRef = useRef<AbortController | null>(null);
  const errorTimerRef = useRef<ReturnType<typeof setTimeout> | null>(null);

  // Reset state when navigating to a different chapter
  useEffect(() => {
    setLanguage('english');
    setIsTranslating(false);
    setError(null);
  }, [chapterId]);

  // Cleanup: abort in-flight request and clear error timer on unmount
  useEffect(() => {
    return () => {
      abortControllerRef.current?.abort();
      if (errorTimerRef.current) clearTimeout(errorTimerRef.current);
    };
  }, []);

  const showError = useCallback((message: string) => {
    setError(message);
    if (errorTimerRef.current) clearTimeout(errorTimerRef.current);
    errorTimerRef.current = setTimeout(() => setError(null), ERROR_DISMISS_MS);
  }, []);

  /**
   * Capture scroll position as a percentage, run the callback (DOM mutation),
   * then restore scroll position via requestAnimationFrame.
   */
  const preserveScroll = useCallback((callback: () => void) => {
    const doc = document.documentElement;
    const maxScroll = doc.scrollHeight - window.innerHeight;
    const scrollPercent = maxScroll > 0 ? doc.scrollTop / maxScroll : 0;

    callback();

    requestAnimationFrame(() => {
      const newMaxScroll = doc.scrollHeight - window.innerHeight;
      doc.scrollTop = scrollPercent * newMaxScroll;
    });
  }, []);

  const getContentContainer = useCallback((): HTMLElement | null => {
    return document.querySelector('.markdown') as HTMLElement | null;
  }, []);

  const applyUrduStyles = useCallback((container: HTMLElement) => {
    container.setAttribute('dir', 'rtl');
    container.classList.add('urdu-content');
  }, []);

  const removeUrduStyles = useCallback((container: HTMLElement) => {
    container.setAttribute('dir', 'ltr');
    container.classList.remove('urdu-content');
  }, []);

  const handleTranslate = useCallback(async () => {
    // Gate behind authentication
    if (!isAuthenticated) {
      setShowLockedModal(true);
      return;
    }

    const container = getContentContainer();
    if (!container) return;

    // --- Toggle back to English ---
    if (language === 'urdu') {
      const cached = getCachedTranslation(chapterId);
      if (cached?.english) {
        preserveScroll(() => {
          container.innerHTML = cached.english;
          removeUrduStyles(container);
        });
      }
      setLanguage('english');
      return;
    }

    // --- Translate to Urdu ---
    setIsTranslating(true);
    setError(null);

    // Check cache first
    const cached = getCachedTranslation(chapterId);
    if (cached?.urdu) {
      preserveScroll(() => {
        container.innerHTML = cached.urdu!;
        applyUrduStyles(container);
      });
      setLanguage('urdu');
      setIsTranslating(false);
      return;
    }

    // Cache miss — call backend API
    const englishContent = container.innerHTML;
    const controller = new AbortController();
    abortControllerRef.current = controller;

    try {
      const response = await fetch(`${apiUrl}/api/translate`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        credentials: 'include',
        body: JSON.stringify({
          chapter_id: chapterId,
          chapter_title: chapterTitle,
          content: englishContent,
          target_language: 'urdu',
        }),
        signal: controller.signal,
      });

      if (response.status === 401) {
        window.location.href = loginUrl;
        return;
      }

      if (!response.ok) {
        const errorBody = await response.json().catch(() => null);
        const detail =
          errorBody?.detail || `Translation failed (HTTP ${response.status})`;
        throw new Error(detail);
      }

      const data = await response.json();

      if (!data.success || !data.translated_content) {
        throw new Error(data.error || 'Translation returned empty content');
      }

      // Cache both English and Urdu
      cacheTranslation(chapterId, englishContent, data.translated_content);

      // Update DOM
      preserveScroll(() => {
        container.innerHTML = data.translated_content;
        applyUrduStyles(container);
      });

      setLanguage('urdu');
    } catch (err: unknown) {
      if (err instanceof DOMException && err.name === 'AbortError') {
        // User navigated away — silently ignore
        return;
      }
      const message =
        !navigator.onLine
          ? 'You are offline. Please check your internet connection.'
          : err instanceof Error
            ? err.message
            : 'Translation failed. Please try again.';
      showError(message);
    } finally {
      setIsTranslating(false);
      abortControllerRef.current = null;
    }
  }, [
    isAuthenticated,
    loginUrl,
    language,
    chapterId,
    chapterTitle,
    apiUrl,
    preserveScroll,
    getContentContainer,
    applyUrduStyles,
    removeUrduStyles,
    showError,
  ]);

  const buttonLabel = isTranslating
    ? 'Translating...'
    : language === 'english'
      ? 'Translate to Urdu'
      : 'Show Original';

  const isLocked = !isAuthenticated && language === 'english';

  return (
    <>
      <button
        className={`${styles.translationButton}${isLocked ? ` ${styles.locked}` : ''}`}
        onClick={handleTranslate}
        disabled={isTranslating}
        aria-label={buttonLabel}
        title={buttonLabel}
      >
        {isTranslating && <span className={styles.spinner} />}
        {buttonLabel}
        {isLocked && <span className={styles.lockIcon}>🔒</span>}
      </button>
      {error && <div className={styles.errorMessage}>{error}</div>}
      <LockedFeatureModal
        isOpen={showLockedModal}
        onClose={() => setShowLockedModal(false)}
        featureName="Translate"
        loginUrl={loginUrl}
      />
    </>
  );
}
