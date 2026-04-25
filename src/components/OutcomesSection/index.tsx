import React from 'react';
import { useScrollReveal } from '../../hooks/useScrollReveal';
import styles from './styles.module.css';

const outcomes = [
  'Deploy a full ROS 2 navigation stack on a simulated humanoid',
  'Build and test robot behaviors in Gazebo and Isaac Sim',
  'Train a Vision-Language-Action model for manipulation tasks',
  'Bridge the sim-to-real gap with domain randomization',
];

export function OutcomesSection() {
  const { ref, isVisible } = useScrollReveal<HTMLDivElement>({
    threshold: 0.2,
    triggerOnce: true,
  });

  return (
    <section className={styles.outcomesSection}>
      <div
        ref={ref}
        className={`${styles.sectionInner} ${isVisible ? styles.visible : ''}`}
      >
        <div className={styles.content}>
          <span className={styles.eyebrow}>Outcomes</span>
          <h2 className={styles.title}>
            What you'll be able to build
          </h2>
          <ul className={styles.list}>
            {outcomes.map((item, i) => (
              <li
                key={i}
                className={styles.item}
                style={{ '--delay': `${i * 0.08}s` } as React.CSSProperties}
              >
                <span className={styles.check}>
                  <svg viewBox="0 0 16 16" fill="none" width="16" height="16">
                    <path d="M3 8.5l3.5 3.5L13 5" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" />
                  </svg>
                </span>
                <span className={styles.itemText}>{item}</span>
              </li>
            ))}
          </ul>
        </div>
      </div>
    </section>
  );
}

export default OutcomesSection;
