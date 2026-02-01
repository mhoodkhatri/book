import React, { useState, useCallback } from 'react';
import { ChapterChat } from './index';
import { TextSelectionPopup } from './TextSelectionPopup';
import { useAuthGuard } from '@site/src/hooks/useAuthGuard';
import LockedFeatureModal from '@site/src/components/Auth/LockedFeatureModal';
import styles from './styles.module.css';

interface FloatingChatButtonProps {
  chapterId: string;
  chapterTitle: string;
}

export function FloatingChatButton({
  chapterId,
  chapterTitle,
}: FloatingChatButtonProps): React.JSX.Element {
  const { isAuthenticated, loginUrl } = useAuthGuard();
  const [isOpen, setIsOpen] = useState(false);
  const [showLockedModal, setShowLockedModal] = useState(false);
  const [initialMessage, setInitialMessage] = useState<string | null>(null);

  const toggleChat = () => {
    if (!isAuthenticated) {
      setShowLockedModal(true);
      return;
    }
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
        {!isAuthenticated && !isOpen && (
          <span className={styles.lockBadge}>🔒</span>
        )}
      </button>

      {/* Chat Panel */}
      {isOpen && isAuthenticated && (
        <div className={styles.chatPanel}>
          <ChapterChat
            chapterId={chapterId}
            chapterTitle={chapterTitle}
            initialMessage={initialMessage}
            onMessageSent={handleMessageSent}
          />
        </div>
      )}

      <LockedFeatureModal
        isOpen={showLockedModal}
        onClose={() => setShowLockedModal(false)}
        featureName="AI Chatbot"
        loginUrl={loginUrl}
      />
    </>
  );
}

export default FloatingChatButton;
