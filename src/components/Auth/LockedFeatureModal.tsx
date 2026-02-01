import React, { useEffect, useCallback } from 'react';
import styles from './LockedFeatureModal.module.css';

interface LockedFeatureModalProps {
  isOpen: boolean;
  onClose: () => void;
  featureName: string;
  loginUrl: string;
}

export default function LockedFeatureModal({
  isOpen,
  onClose,
  featureName,
  loginUrl,
}: LockedFeatureModalProps): React.JSX.Element | null {
  const handleKeyDown = useCallback(
    (e: KeyboardEvent) => {
      if (e.key === 'Escape') onClose();
    },
    [onClose],
  );

  useEffect(() => {
    if (isOpen) {
      document.addEventListener('keydown', handleKeyDown);
      return () => document.removeEventListener('keydown', handleKeyDown);
    }
  }, [isOpen, handleKeyDown]);

  if (!isOpen) return null;

  const handleBackdropClick = (e: React.MouseEvent) => {
    if (e.target === e.currentTarget) onClose();
  };

  return (
    <div className={styles.overlay} onClick={handleBackdropClick}>
      <div className={styles.card} role="dialog" aria-modal="true">
        <span className={styles.lockIcon}>🔒</span>
        <h3 className={styles.title}>Sign in to unlock {featureName}</h3>
        <p className={styles.description}>
          This feature is available to signed-in users. Create a free account or
          sign in to get started.
        </p>
        <div className={styles.actions}>
          <a href={loginUrl} className={styles.signInButton}>
            Sign In
          </a>
          <button
            className={styles.dismissButton}
            onClick={onClose}
            type="button"
          >
            Maybe Later
          </button>
        </div>
      </div>
    </div>
  );
}
