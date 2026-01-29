import React, { useState, useEffect, useCallback } from 'react';
import styles from './styles.module.css';

interface TextSelectionPopupProps {
  onAskAI: (selectedText: string) => void;
}

/**
 * Shows a floating "Ask AI" button when user selects text on the page.
 * Clicking the button triggers the chat with the selected text.
 */
export function TextSelectionPopup({ onAskAI }: TextSelectionPopupProps): React.JSX.Element | null {
  const [selectedText, setSelectedText] = useState<string>('');
  const [position, setPosition] = useState<{ x: number; y: number } | null>(null);
  const [isVisible, setIsVisible] = useState(false);

  const handleMouseUp = useCallback(() => {
    // Small delay to ensure selection is complete
    setTimeout(() => {
      const selection = window.getSelection();
      const text = selection?.toString().trim() || '';

      if (text.length > 3 && text.length < 500) {
        const range = selection?.getRangeAt(0);
        if (range) {
          const rect = range.getBoundingClientRect();
          setPosition({
            x: rect.left + rect.width / 2,
            y: rect.top - 10,
          });
          setSelectedText(text);
          setIsVisible(true);
        }
      } else {
        setIsVisible(false);
        setSelectedText('');
      }
    }, 10);
  }, []);

  const handleMouseDown = useCallback((e: MouseEvent) => {
    // Hide popup when clicking elsewhere (but not on the popup itself)
    const target = e.target as HTMLElement;
    if (!target.closest(`.${styles.selectionPopup}`)) {
      setIsVisible(false);
    }
  }, []);

  const handleScroll = useCallback(() => {
    setIsVisible(false);
  }, []);

  useEffect(() => {
    document.addEventListener('mouseup', handleMouseUp);
    document.addEventListener('mousedown', handleMouseDown);
    document.addEventListener('scroll', handleScroll, true);

    return () => {
      document.removeEventListener('mouseup', handleMouseUp);
      document.removeEventListener('mousedown', handleMouseDown);
      document.removeEventListener('scroll', handleScroll, true);
    };
  }, [handleMouseUp, handleMouseDown, handleScroll]);

  const handleAskAI = () => {
    if (selectedText) {
      onAskAI(selectedText);
      setIsVisible(false);
      setSelectedText('');
      // Clear the selection
      window.getSelection()?.removeAllRanges();
    }
  };

  if (!isVisible || !position) {
    return null;
  }

  return (
    <div
      className={styles.selectionPopup}
      style={{
        left: `${position.x}px`,
        top: `${position.y}px`,
      }}
    >
      <button
        className={styles.askAIButton}
        onClick={handleAskAI}
        title="Ask AI to explain this text"
      >
        <span className={styles.askAIIcon}>&#10024;</span>
        Ask AI
      </button>
    </div>
  );
}

export default TextSelectionPopup;
