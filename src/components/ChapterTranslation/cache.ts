/**
 * localStorage cache utilities for chapter translations.
 */

export interface ChapterTranslationCache {
  chapterId: string;
  english: string;
  urdu: string | null;
  translatedAt: number | null;
}

const CACHE_KEY_PREFIX = 'translation_cache_';
const CACHE_MAX_AGE_MS = 7 * 24 * 60 * 60 * 1000; // 7 days

function getCacheKey(chapterId: string): string {
  return `${CACHE_KEY_PREFIX}${chapterId}`;
}

export function getCachedTranslation(
  chapterId: string,
): ChapterTranslationCache | null {
  try {
    const raw = localStorage.getItem(getCacheKey(chapterId));
    if (!raw) return null;

    const cache = JSON.parse(raw) as ChapterTranslationCache;

    // Check expiry
    if (cache.translatedAt && Date.now() - cache.translatedAt > CACHE_MAX_AGE_MS) {
      localStorage.removeItem(getCacheKey(chapterId));
      return null;
    }

    return cache;
  } catch {
    return null;
  }
}

export function cacheTranslation(
  chapterId: string,
  english: string,
  urdu: string,
): void {
  const cache: ChapterTranslationCache = {
    chapterId,
    english,
    urdu,
    translatedAt: Date.now(),
  };
  try {
    localStorage.setItem(getCacheKey(chapterId), JSON.stringify(cache));
  } catch {
    // localStorage full — clear old translation caches and retry
    clearTranslationCache();
    try {
      localStorage.setItem(getCacheKey(chapterId), JSON.stringify(cache));
    } catch {
      // Still failed — silently ignore
    }
  }
}

export function clearTranslationCache(chapterId?: string): void {
  if (chapterId) {
    localStorage.removeItem(getCacheKey(chapterId));
  } else {
    const keys = Object.keys(localStorage).filter((key) =>
      key.startsWith(CACHE_KEY_PREFIX),
    );
    keys.forEach((key) => localStorage.removeItem(key));
  }
}
