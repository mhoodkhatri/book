import React from 'react';
import Link from '@docusaurus/Link';
import { useScrollReveal } from '../../hooks/useScrollReveal';
import styles from './styles.module.css';

export function ClosingCTA() {
  const { ref, isVisible } = useScrollReveal<HTMLDivElement>({
    threshold: 0.3,
    triggerOnce: true,
  });

  return (
    <section className={styles.ctaSection}>
      <div
        ref={ref}
        className={`${styles.sectionInner} ${isVisible ? styles.visible : ''}`}
      >
        <h2 className={styles.title}>
          Ready to build the future<br />of robotics?
        </h2>
        <p className={styles.subtitle}>
          Join thousands of engineers, students, and researchers learning to build intelligent humanoid systems.
        </p>
        <div className={styles.actions}>
          <Link to="/docs/intro" className={styles.primaryBtn}>
            Start Reading — Free
            <span className={styles.arrow}>→</span>
          </Link>
          <a
            href="https://github.com/mhoodkhatri/book"
            target="_blank"
            rel="noopener noreferrer"
            className={styles.secondaryBtn}
          >
            View on GitHub
          </a>
        </div>
      </div>
    </section>
  );
}

export default ClosingCTA;
