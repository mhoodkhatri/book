import React from 'react';
import Layout from '@theme-original/DocItem/Layout';
import type LayoutType from '@theme/DocItem/Layout';
import type { WrapperProps } from '@docusaurus/types';
import { useDoc } from '@docusaurus/plugin-content-docs/client';
import { FloatingChatButton } from '@site/src/components/ChapterChat/FloatingButton';
import { TranslateButton } from '@site/src/components/TranslateButton';

type Props = WrapperProps<typeof LayoutType>;

/**
 * Swizzled DocItem/Layout to inject the floating chat button and translate button.
 *
 * This wrapper adds:
 * - Chapter-aware chat assistant (floating button, bottom-right)
 * - Translate to Urdu button (top of content)
 */
export default function LayoutWrapper(props: Props): React.JSX.Element {
  const { metadata } = useDoc();

  // Extract chapter info from the doc metadata
  const chapterId = metadata.id;
  const chapterTitle = metadata.title;

  return (
    <>
      <div style={{ marginBottom: '16px', display: 'flex', justifyContent: 'flex-end' }}>
        <TranslateButton chapterTitle={chapterTitle} />
      </div>
      <Layout {...props} />
      <FloatingChatButton
        chapterId={chapterId}
        chapterTitle={chapterTitle}
      />
    </>
  );
}
