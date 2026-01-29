import React, { useState, useCallback } from 'react';
import { ChapterChat } from './index';
import { TextSelectionPopup } from './TextSelectionPopup';
import styles from './styles.module.css';

interface FloatingChatButtonProps {
  chapterId: string;
  chapterTitle: string;
}

export function FloatingChatButton({
  chapterId,
  chapterTitle,
}: FloatingChatButtonProps): React.JSX.Element {
  const [isOpen, setIsOpen] = useState(false);
  const [initialMessage, setInitialMessage] = useState<string | null>(null);

  const toggleChat = () => {
    setIsOpen((prev) => !prev);
    // Clear initial message when manually toggling
    if (isOpen) {
      setInitialMessage(null);
    }
  };

  const handleAskAI = useCallback((selectedText: string) => {
    // Set the initial message with the selected text
    const message = `Explain this to me: "${selectedText}"`;
    setInitialMessage(message);
    // Open the chat
    setIsOpen(true);
  }, []);

  const handleMessageSent = useCallback(() => {
    // Clear initial message after it's been sent
    setInitialMessage(null);
  }, []);

  return (
    <>
      {/* Text Selection Popup */}
      <TextSelectionPopup onAskAI={handleAskAI} />

      {/* Floating Action Button */}
      <button
        className={`${styles.floatingButton} ${isOpen ? styles.floatingButtonOpen : ''}`}
        onClick={toggleChat}
        aria-label={isOpen ? 'Close chat assistant' : 'Open chat assistant'}
        title={isOpen ? 'Close chat' : 'Ask about this chapter'}
      >
        {isOpen ? '\u2715' : '\uD83D\uDCAC'}
      </button>

      {/* Chat Panel */}
      {isOpen && (
        <div className={styles.chatPanel}>
          <ChapterChat
            chapterId={chapterId}
            chapterTitle={chapterTitle}
            initialMessage={initialMessage}
            onMessageSent={handleMessageSent}
          />
        </div>
      )}
    </>
  );
}

export default FloatingChatButton;
