import React, { useState, useRef, useEffect, useCallback } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './styles.module.css';

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
}

interface ChapterChatProps {
  chapterId: string;
  chapterTitle: string;
  initialMessage?: string | null;
  onMessageSent?: () => void;
}

/**
 * Chapter-aware chat component that uses RAG to answer questions.
 *
 * Uses native fetch with SSE streaming for real-time responses.
 */
export function ChapterChat({
  chapterId,
  chapterTitle,
  initialMessage,
  onMessageSent,
}: ChapterChatProps): React.JSX.Element {
  const { siteConfig } = useDocusaurusContext();
  const chatApiUrl = (siteConfig.customFields?.chatApiUrl as string) || 'http://localhost:8000';

  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const messagesEndRef = useRef<HTMLDivElement>(null);
  const abortControllerRef = useRef<AbortController | null>(null);
  const initialMessageSentRef = useRef(false);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  const sendMessage = useCallback(async (userMessage: string) => {
    if (!userMessage.trim() || isLoading) return;

    // Add user message
    const userMsg: Message = {
      id: `user-${Date.now()}`,
      role: 'user',
      content: userMessage.trim(),
    };

    setMessages(prev => [...prev, userMsg]);
    setInput('');
    setIsLoading(true);
    setError(null);

    // Create assistant message placeholder
    const assistantId = `assistant-${Date.now()}`;
    setMessages(prev => [...prev, { id: assistantId, role: 'assistant', content: '' }]);

    try {
      // Create abort controller for this request
      abortControllerRef.current = new AbortController();

      const response = await fetch(`${chatApiUrl}/api/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        credentials: 'include',
        body: JSON.stringify({
          messages: [...messages, userMsg].map(m => ({ role: m.role, content: m.content })),
          chapterId,
          chapterTitle,
        }),
        signal: abortControllerRef.current.signal,
      });

      if (response.status === 401) {
        throw new Error('AUTH_REQUIRED');
      }

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const reader = response.body?.getReader();
      if (!reader) {
        throw new Error('No response body');
      }

      const decoder = new TextDecoder();
      let assistantContent = '';

      // Read the SSE stream
      while (true) {
        const { done, value } = await reader.read();
        if (done) break;

        const chunk = decoder.decode(value, { stream: true });
        const lines = chunk.split('\n').filter(line => line.trim());

        for (const line of lines) {
          // Parse Vercel AI SDK SSE format: 0:"text" for content
          if (line.startsWith('0:')) {
            try {
              const content = JSON.parse(line.slice(2));
              assistantContent += content;

              // Update the assistant message with streamed content
              setMessages(prev =>
                prev.map(m =>
                  m.id === assistantId
                    ? { ...m, content: assistantContent }
                    : m
                )
              );
            } catch {
              // Ignore parse errors for malformed chunks
            }
          }
        }
      }
    } catch (err) {
      if (err instanceof Error && err.name === 'AbortError') {
        // Request was cancelled, ignore
        return;
      }

      console.error('Chat error:', err);

      if (err instanceof Error && err.message === 'AUTH_REQUIRED') {
        // Session expired — preserve the query text and show auth message
        setError('Your session has expired. Please sign in again to continue.');
        // Keep the user message so it's not lost
      } else {
        setError('Something went wrong. Please try again.');
      }

      // Remove the empty assistant message on error
      setMessages(prev => prev.filter(m => m.id !== assistantId));
    } finally {
      setIsLoading(false);
      abortControllerRef.current = null;
    }
  }, [chatApiUrl, chapterId, chapterTitle, isLoading, messages]);

  // Auto-send initial message when provided
  useEffect(() => {
    if (initialMessage && !initialMessageSentRef.current && !isLoading) {
      initialMessageSentRef.current = true;
      sendMessage(initialMessage);
      onMessageSent?.();
    }
  }, [initialMessage, isLoading, sendMessage, onMessageSent]);

  // Reset the ref when initialMessage changes to null
  useEffect(() => {
    if (!initialMessage) {
      initialMessageSentRef.current = false;
    }
  }, [initialMessage]);

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    sendMessage(input);
  };

  const handleInputChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    setInput(e.target.value);
  };

  return (
    <div className={styles.chatContainer}>
      {/* Context Badge */}
      <div className={styles.contextBadge}>
        <span className={styles.contextIcon}>📖</span>
        <span className={styles.contextTitle}>{chapterTitle}</span>
      </div>

      {/* Messages */}
      <div className={styles.messages}>
        {messages.length === 0 && !isLoading && (
          <div className={styles.welcomeMessage}>
            <p>Hi! I'm your chapter assistant.</p>
            <p>Ask me anything about <strong>{chapterTitle}</strong>.</p>
            <p className={styles.selectionHint}>
              Tip: Select any text on the page and click "Ask AI" to get an explanation.
            </p>
          </div>
        )}

        {messages.map((message) => (
          <div
            key={message.id}
            className={`${styles.message} ${styles[message.role]}`}
          >
            <div className={styles.messageContent}>
              {message.content || (message.role === 'assistant' && isLoading ? '' : message.content)}
            </div>
          </div>
        ))}

        {isLoading && messages[messages.length - 1]?.content === '' && (
          <div className={`${styles.message} ${styles.assistant}`}>
            <div className={styles.typingIndicator}>
              <span></span>
              <span></span>
              <span></span>
            </div>
          </div>
        )}

        {error && (
          <div className={styles.errorMessage}>
            {error}
          </div>
        )}

        <div ref={messagesEndRef} />
      </div>

      {/* Input Form */}
      <form onSubmit={handleSubmit} className={styles.inputForm}>
        <input
          type="text"
          value={input}
          onChange={handleInputChange}
          placeholder="Ask about this chapter..."
          disabled={isLoading}
          className={styles.input}
        />
        <button
          type="submit"
          disabled={isLoading || !input.trim()}
          className={styles.submitBtn}
        >
          {isLoading ? '...' : '\u2192'}
        </button>
      </form>
    </div>
  );
}

export default ChapterChat;
