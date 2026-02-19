import React from 'react';
import { useScrollReveal } from '../../hooks/useScrollReveal';
import styles from './styles.module.css';

const audiences = [
  {
    id: 'engineers',
    icon: (
      <svg viewBox="0 0 32 32" fill="none" className={styles.audienceIcon}>
        <rect x="4" y="8" width="24" height="16" rx="2" stroke="currentColor" strokeWidth="2" />
        <path d="M10 16h4m4 0h4" stroke="currentColor" strokeWidth="2" strokeLinecap="round" />
        <circle cx="16" cy="16" r="2" fill="currentColor" />
      </svg>
    ),
    title: 'Robotics Engineers',
    description: 'Integrate ROS 2, simulation, and AI into production-grade robotic systems.',
  },
  {
    id: 'students',
    icon: (
      <svg viewBox="0 0 32 32" fill="none" className={styles.audienceIcon}>
        <path d="M6 12l10-5 10 5-10 5-10-5z" stroke="currentColor" strokeWidth="2" strokeLinejoin="round" />
        <path d="M8 13.5v6c0 1 3.5 3.5 8 3.5s8-2.5 8-3.5v-6" stroke="currentColor" strokeWidth="2" />
        <path d="M26 12v8" stroke="currentColor" strokeWidth="2" strokeLinecap="round" />
      </svg>
    ),
    title: 'CS & ML Students',
    description: 'Build a portfolio of robotics projects from fundamentals through advanced AI.',
  },
  {
    id: 'researchers',
    icon: (
      <svg viewBox="0 0 32 32" fill="none" className={styles.audienceIcon}>
        <circle cx="16" cy="12" r="6" stroke="currentColor" strokeWidth="2" />
        <path d="M12 17l-2 9h12l-2-9" stroke="currentColor" strokeWidth="2" strokeLinejoin="round" />
        <path d="M13 12h6" stroke="currentColor" strokeWidth="2" strokeLinecap="round" />
        <path d="M16 9v6" stroke="currentColor" strokeWidth="2" strokeLinecap="round" />
      </svg>
    ),
    title: 'AI Researchers',
    description: 'Explore VLA models, sim-to-real transfer, and embodied intelligence.',
  },
];

export function AudienceSection() {
  const { ref, isVisible } = useScrollReveal<HTMLDivElement>({
    threshold: 0.2,
    triggerOnce: true,
  });

  return (
    <section className={styles.audienceSection}>
      <div
        ref={ref}
        className={`${styles.sectionInner} ${isVisible ? styles.visible : ''}`}
      >
        <span className={styles.eyebrow}>Who is this for?</span>
        <h2 className={styles.title}>
          Built for builders at every stage
        </h2>

        <div className={styles.cards}>
          {audiences.map((a, i) => (
            <div
              key={a.id}
              className={styles.card}
              style={{ '--delay': `${i * 0.1}s` } as React.CSSProperties}
            >
              <div className={styles.iconBox}>{a.icon}</div>
              <h3 className={styles.cardTitle}>{a.title}</h3>
              <p className={styles.cardDesc}>{a.description}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

export default AudienceSection;
