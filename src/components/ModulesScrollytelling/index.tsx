import React from 'react';
import Link from '@docusaurus/Link';
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
  color: string;
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
    color: 'var(--module-1-color)',
  },
  {
    id: 'simulation',
    number: 2,
    title: 'The Digital Twin',
    subtitle: 'Gazebo & Unity',
    description: 'Create realistic simulations to test and validate robot behaviors before deploying to hardware.',
    weeks: 'Weeks 6-7',
    link: '/docs/module-2-simulation',
    color: 'var(--module-2-color)',
  },
  {
    id: 'isaac',
    number: 3,
    title: 'The AI-Robot Brain',
    subtitle: 'NVIDIA Isaac',
    description: 'Leverage GPU-accelerated AI for perception, localization, and autonomous navigation.',
    weeks: 'Weeks 8-10',
    link: '/docs/module-3-nvidia-isaac',
    color: 'var(--module-3-color)',
  },
  {
    id: 'vla',
    number: 4,
    title: 'Vision-Language-Action',
    subtitle: 'VLA Models',
    description: 'Build robots that understand natural language and translate commands into physical actions.',
    weeks: 'Weeks 11-13',
    link: '/docs/module-4-vla',
    color: 'var(--module-4-color)',
  },
];

function ModuleCard({ module, index }: { module: Module; index: number }) {
  const { ref, isVisible } = useScrollReveal<HTMLDivElement>({
    threshold: 0.3,
    triggerOnce: true,
  });

  return (
    <div
      ref={ref}
      className={`${styles.moduleCard} ${isVisible ? styles.visible : ''}`}
      style={{ '--module-color': module.color, '--delay': `${index * 0.1}s` } as React.CSSProperties}
    >
      <div className={styles.moduleNumber}>
        <span>{module.number}</span>
      </div>
      <div className={styles.moduleContent}>
        <span className={styles.moduleWeeks}>{module.weeks}</span>
        <h2 className={styles.moduleTitle}>{module.title}</h2>
        <h3 className={styles.moduleSubtitle}>{module.subtitle}</h3>
        <p className={styles.moduleDescription}>{module.description}</p>
        <Link to={module.link} className={styles.moduleLink}>
          Explore Module {module.number}
        </Link>
      </div>
    </div>
  );
}

export function ModulesScrollytelling(): JSX.Element {
  return (
    <section className={styles.scrollSection}>
      <div className={styles.sectionHeader}>
        <h2 className={styles.sectionTitle}>Course Modules</h2>
        <p className={styles.sectionSubtitle}>
          Four progressive modules that take you from fundamentals to the frontier of robotics AI
        </p>
      </div>

      <div className={styles.modulesContainer}>
        {modules.map((module, index) => (
          <ModuleCard key={module.id} module={module} index={index} />
        ))}
      </div>

      <div className={styles.ctaSection}>
        <Link to="/docs/intro" className={styles.ctaButton}>
          Start Your Journey
        </Link>
      </div>
    </section>
  );
}

export default ModulesScrollytelling;
