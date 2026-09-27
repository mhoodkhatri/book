import React, { useEffect, useRef, useState } from 'react';
import Link from '@docusaurus/Link';
import useBaseUrl from '@docusaurus/useBaseUrl';
import { useScrollReveal } from '../../hooks/useScrollReveal';
import styles from './styles.module.css';

interface Module {
  id: string;
  number: number;
  title: string;
  subtitle: string;
  description: string;
  weeks: string;
  link: string;
  icon: string;
}

const modules: Module[] = [
  {
    id: 'ros2',
    number: 1,
    title: 'The Robotic Nervous System',
    subtitle: 'ROS 2 Fundamentals',
    description: 'Master the industry-standard middleware for robot development. Learn nodes, topics, services, and actions.',
    weeks: 'Weeks 3-5',
    link: '/docs/module-1-ros2',
    icon: '/img/module-icons/ros2.svg',
  },
  {
    id: 'simulation',
    number: 2,
    title: 'The Digital Twin',
    subtitle: 'Gazebo & Unity',
    description: 'Create realistic simulations to test and validate robot behaviors before deploying to hardware.',
    weeks: 'Weeks 6-7',
    link: '/docs/module-2-simulation',
    icon: '/img/module-icons/simulation.svg',
  },
  {
    id: 'isaac',
    number: 3,
    title: 'The AI-Robot Brain',
    subtitle: 'NVIDIA Isaac',
    description: 'Leverage GPU-accelerated AI for perception, localization, and autonomous navigation.',
    weeks: 'Weeks 8-10',
    link: '/docs/module-3-nvidia-isaac',
    icon: '/img/module-icons/isaac.svg',
  },
  {
    id: 'vla',
    number: 4,
    title: 'Vision-Language-Action',
    subtitle: 'VLA Models',
    description: 'Build robots that understand natural language and translate commands into physical actions.',
    weeks: 'Weeks 11-13',
    link: '/docs/module-4-vla',
    icon: '/img/module-icons/vla.svg',
  },
];

function useScrollHighlight<T extends HTMLElement>() {
  const ref = useRef<T>(null);
  const [isActive, setIsActive] = useState(false);

  useEffect(() => {
    const el = ref.current;
    if (!el) return;
    // Only enable on touch devices (no hover support)
    if (window.matchMedia('(hover: hover)').matches) return;

    const observer = new IntersectionObserver(
      ([entry]) => setIsActive(entry.isIntersecting),
      { threshold: 0.6 },
    );
    observer.observe(el);
    return () => observer.disconnect();
  }, []);

  return { ref, isActive };
}

function ModuleCard({ module, index }: { module: Module; index: number }) {
  const { ref: revealRef, isVisible } = useScrollReveal<HTMLDivElement>({
    threshold: 0.2,
    triggerOnce: true,
  });
  const { ref: highlightRef, isActive } = useScrollHighlight<HTMLDivElement>();
  const iconUrl = useBaseUrl(module.icon);

  return (
    <div
      ref={(node) => {
        (revealRef as React.MutableRefObject<HTMLDivElement | null>).current = node;
        (highlightRef as React.MutableRefObject<HTMLDivElement | null>).current = node;
      }}
      className={`${styles.moduleCard} ${isVisible ? styles.visible : ''}`}
      style={{ '--delay': `${index * 0.12}s` } as React.CSSProperties}
    >
      <div className={`${styles.cardInner} ${isActive ? styles.scrollActive : ''}`}>
        <div className={styles.cardHeader}>
          <div className={styles.iconWrapper}>
            <img src={iconUrl} alt={module.subtitle} className={styles.moduleIcon} />
          </div>
          <span className={styles.moduleNumber}>0{module.number}</span>
        </div>

        <span className={styles.moduleWeeks}>{module.weeks}</span>
        <h2 className={styles.moduleTitle}>{module.title}</h2>
        <h3 className={styles.moduleSubtitle}>{module.subtitle}</h3>
        <p className={styles.moduleDescription}>{module.description}</p>

        <Link to={module.link} className={styles.moduleLink}>
          Explore Module
          <span className={styles.linkArrow}>→</span>
        </Link>
      </div>
    </div>
  );
}

export function ModulesScrollytelling() {
  return (
    <section id="modules" className={styles.scrollSection}>
      <div className={styles.sectionInner}>
        <div className={styles.sectionHeader}>
          <span className={styles.sectionEyebrow}>Curriculum</span>
          <h2 className={styles.sectionTitle}>Course Modules</h2>
          <p className={styles.sectionSubtitle}>
            Four progressive modules that take you from fundamentals to the frontier of robotics AI
          </p>
        </div>

        <div className={styles.modulesGrid}>
          {modules.map((module, index) => (
            <ModuleCard key={module.id} module={module} index={index} />
          ))}
        </div>

      </div>
    </section>
  );
}

export default ModulesScrollytelling;
