import React from 'react';
import Layout from '@theme-original/DocItem/Layout';
import type LayoutType from '@theme/DocItem/Layout';
import type { WrapperProps } from '@docusaurus/types';
import { useDoc } from '@docusaurus/plugin-content-docs/client';
import { FloatingChatButton } from '@site/src/components/ChapterChat/FloatingButton';

type Props = WrapperProps<typeof LayoutType>;

/**
 * Swizzled DocItem/Layout to inject the floating chat button.
 *
 * This wrapper adds the chapter-aware chat assistant to all doc pages.
 * The chat button appears as a floating action button in the bottom-right
 * corner and opens a chat panel for asking questions about the current chapter.
 */
export default function LayoutWrapper(props: Props): React.JSX.Element {
  const { metadata } = useDoc();

  // Extract chapter info from the doc metadata
  const chapterId = metadata.id;
  const chapterTitle = metadata.title;

  return (
    <>
      <Layout {...props} />
      <FloatingChatButton
        chapterId={chapterId}
        chapterTitle={chapterTitle}
      />
    </>
  );
}
