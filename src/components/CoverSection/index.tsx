import React from 'react';
import Link from '@docusaurus/Link';
import useBaseUrl from '@docusaurus/useBaseUrl';
import styles from './styles.module.css';

export function CoverSection(): JSX.Element {
  return (
    <header className={styles.coverSection}>
      <div className={styles.coverContent}>
        {/* ── Left: Hero Text ── */}
        <div className={styles.heroText}>
          <div className={styles.eyebrow}>
            <span className={styles.eyebrowDot} />
            Open Textbook — 2026 Edition
          </div>

          <h1 className={styles.title}>
            Physical AI &<br />
            <span className={styles.titleHighlight}>Humanoid Robotics</span>
          </h1>

          <p className={styles.tagline}>
            A comprehensive textbook for humanoid robotics development — from
            ROS 2 fundamentals to Vision-Language-Action models.
          </p>

          <div className={styles.buttons}>
            <Link className={styles.primaryButton} to="/docs/intro">
              Start Reading
              <span className={styles.buttonArrow}>→</span>
            </Link>
            {/* <a className={styles.secondaryButton} href="#modules">
              Explore Modules
            </a> */}
          </div>

          {/* ── Author Badge ── */}
          <div className={styles.trustSection}>
            <span className={styles.trustDivider} />
            <a
              href="https://pk.linkedin.com/in/mhoodkhatri"
              target="_blank"
              rel="noopener noreferrer"
              className={styles.authorLink}
            >
              <div className={styles.authorBadge}>
                <span className={styles.authorBadgeLabel}>AUTHOR BY</span>
                <div className={styles.authorBadgeInfo}>
                  <img
                    src={useBaseUrl('/img/author-hood.jpeg')}
                    alt="Muhammad Hood"
                    className={styles.authorAvatar}
                  />
                  <span className={styles.authorBadgeName}>Muhammad Hood</span>
                </div>
              </div>
            </a>
          </div>

          {/* ── Scroll Indicator (inline on mobile) ── */}
          <div className={styles.scrollIndicatorInline} aria-hidden="true">
            <span className={styles.scrollText}>Scroll to explore</span>
            <div className={styles.scrollArrow} />
          </div>
        </div>

        {/* ── Right: Book Cover Image ── */}
        <div className={styles.bookVisual}>
          <div className={styles.bookGlow} />
          <div className={styles.bookCover}>
            <img
              src={useBaseUrl('/img/book-cover.png')}
              alt="Physical AI & Humanoid Robotics Book Cover"
              className={styles.bookImage}
            />
          </div>
        </div>
      </div>

      {/* ── Scroll Indicator (absolute on desktop) ── */}
      <div className={styles.scrollIndicator} aria-hidden="true">
        <span className={styles.scrollText}>Scroll to explore</span>
        <div className={styles.scrollArrow} />
      </div>
    </header>
  );
}

export default CoverSection;
