import React from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './styles.module.css';

export function CoverSection(): JSX.Element {
  const { siteConfig } = useDocusaurusContext();

  return (
    <header className={styles.coverSection}>
      <div className={styles.coverContent}>
        <div className={styles.bookVisual}>
          <div className={styles.bookCover}>
            <div className={styles.bookSpine} />
            <div className={styles.bookFront}>
              <div className={styles.bookTitle}>
                <span className={styles.titleLine1}>Physical AI</span>
                <span className={styles.titleLine2}>&</span>
                <span className={styles.titleLine3}>Humanoid Robotics</span>
              </div>
              <div className={styles.bookSubtitle}>A Comprehensive Textbook</div>
            </div>
          </div>
        </div>

        <div className={styles.heroText}>
          <h1 className={styles.title}>{siteConfig.title}</h1>
          <p className={styles.tagline}>{siteConfig.tagline}</p>
          <div className={styles.buttons}>
            <Link
              className={styles.primaryButton}
              to="/docs/intro"
            >
              Start Learning
            </Link>
            <Link
              className={styles.secondaryButton}
              to="/docs/module-1-ros2"
            >
              Jump to Module 1
            </Link>
          </div>
        </div>
      </div>

      <div className={styles.scrollIndicator}>
        <span>Scroll to explore modules</span>
        <div className={styles.scrollArrow} />
      </div>
    </header>
  );
}

export default CoverSection;
