# Quickstart: Docusaurus Setup

**Feature**: 001-docusaurus-setup
**Prerequisites**: Node.js 18+, npm 9+, Git

## 1. Initialize Docusaurus Project

```bash
# From repository root
npx create-docusaurus@3 website classic --typescript

# Move contents to root (Docusaurus creates a subdirectory)
mv website/* .
mv website/.* . 2>/dev/null || true
rmdir website
```

## 2. Install Additional Dependencies

```bash
npm install @easyops-cn/docusaurus-search-local
npm install @docusaurus/plugin-ideal-image
```

## 3. Configure docusaurus.config.ts

Key settings to update:

```typescript
// docusaurus.config.ts
import type { Config } from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A comprehensive textbook for humanoid robotics development',
  url: 'https://YOUR_USERNAME.github.io',
  baseUrl: '/book/',
  organizationName: 'YOUR_USERNAME',
  projectName: 'book',

  themeConfig: {
    colorMode: {
      defaultMode: 'light',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI Textbook',
      logo: { alt: 'Logo', src: 'img/logo.svg' },
      items: [
        { type: 'docSidebar', sidebarId: 'docs', position: 'left', label: 'Textbook' },
        { href: 'https://github.com/YOUR_USERNAME/book', label: 'GitHub', position: 'right' },
      ],
    },
    prism: {
      additionalLanguages: ['python', 'bash', 'yaml', 'cpp', 'cmake', 'xml'],
    },
  } satisfies Preset.ThemeConfig,

  plugins: [
    '@docusaurus/plugin-ideal-image',
    [
      '@easyops-cn/docusaurus-search-local',
      {
        hashed: true,
        indexDocs: true,
        indexBlog: false,
        docsRouteBasePath: '/docs',
      },
    ],
  ],

  scripts: [
    {
      src: 'https://plausible.io/js/script.js',
      defer: true,
      'data-domain': 'YOUR_DOMAIN.github.io',
    },
  ],
};

export default config;
```

## 4. Create Sidebar Configuration

```typescript
// sidebars.ts
import type { SidebarsConfig } from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  docs: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: ROS 2',
      collapsed: false,
      link: { type: 'doc', id: 'module-1-ros2/index' },
      items: [
        'module-1-ros2/01-nodes-topics',
        'module-1-ros2/02-services-actions',
        'module-1-ros2/03-rclpy',
        'module-1-ros2/04-urdf',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Simulation',
      link: { type: 'doc', id: 'module-2-simulation/index' },
      items: [
        'module-2-simulation/01-gazebo-basics',
        'module-2-simulation/02-physics-sim',
        'module-2-simulation/03-unity-integration',
        'module-2-simulation/04-sensors',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac',
      link: { type: 'doc', id: 'module-3-nvidia-isaac/index' },
      items: [
        'module-3-nvidia-isaac/01-isaac-sim',
        'module-3-nvidia-isaac/02-isaac-ros',
        'module-3-nvidia-isaac/03-vslam',
        'module-3-nvidia-isaac/04-nav2',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: VLA',
      link: { type: 'doc', id: 'module-4-vla/index' },
      items: [
        'module-4-vla/01-whisper',
        'module-4-vla/02-cognitive-planning',
        'module-4-vla/03-capstone',
      ],
    },
    {
      type: 'category',
      label: 'Hardware',
      items: [
        'hardware/workstation',
        'hardware/edge-kit',
      ],
    },
    {
      type: 'category',
      label: 'Appendix',
      items: [
        'appendix/assessments',
        'appendix/resources',
      ],
    },
  ],
};

export default sidebars;
```

## 5. Create Custom Landing Page

```tsx
// src/pages/index.tsx
import React from 'react';
import Layout from '@theme/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { CoverSection } from '../components/CoverSection';
import { ModulesScrollytelling } from '../components/ModulesScrollytelling';

export default function Home(): JSX.Element {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout title={siteConfig.title} description={siteConfig.tagline}>
      <CoverSection />
      <ModulesScrollytelling />
    </Layout>
  );
}
```

## 6. Create Scrollytelling Component

```tsx
// src/components/ModulesScrollytelling/index.tsx
import React, { useEffect, useRef, useState } from 'react';
import styles from './styles.module.css';

const modules = [
  { id: 'ros2', title: 'Module 1: The Robotic Nervous System', subtitle: 'ROS 2 Fundamentals', color: '#2e8555' },
  { id: 'simulation', title: 'Module 2: The Digital Twin', subtitle: 'Gazebo & Unity', color: '#1abc9c' },
  { id: 'isaac', title: 'Module 3: The AI-Robot Brain', subtitle: 'NVIDIA Isaac', color: '#76b900' },
  { id: 'vla', title: 'Module 4: Vision-Language-Action', subtitle: 'VLA Models', color: '#9b59b6' },
];

export function ModulesScrollytelling() {
  return (
    <section className={styles.scrollSection}>
      {modules.map((module, index) => (
        <ModuleCard key={module.id} module={module} index={index} />
      ))}
    </section>
  );
}

function ModuleCard({ module, index }) {
  const [isVisible, setIsVisible] = useState(false);
  const ref = useRef<HTMLDivElement>(null);

  useEffect(() => {
    const observer = new IntersectionObserver(
      ([entry]) => setIsVisible(entry.isIntersecting),
      { threshold: 0.3 }
    );
    if (ref.current) observer.observe(ref.current);
    return () => observer.disconnect();
  }, []);

  return (
    <div
      ref={ref}
      className={`${styles.moduleCard} ${isVisible ? styles.visible : ''}`}
      style={{ '--module-color': module.color } as React.CSSProperties}
    >
      <span className={styles.moduleNumber}>{index + 1}</span>
      <h2>{module.title}</h2>
      <p>{module.subtitle}</p>
    </div>
  );
}
```

## 7. Add GitHub Actions Workflow

```yaml
# .github/workflows/deploy.yml
name: Deploy to GitHub Pages

on:
  push:
    branches: [main]
  workflow_dispatch:

permissions:
  contents: read
  pages: write
  id-token: write

concurrency:
  group: pages
  cancel-in-progress: false

jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-node@v4
        with:
          node-version: 18
          cache: npm
      - run: npm ci
      - run: npm run build
      - uses: actions/upload-pages-artifact@v3
        with:
          path: build

  deploy:
    environment:
      name: github-pages
      url: ${{ steps.deployment.outputs.page_url }}
    runs-on: ubuntu-latest
    needs: build
    steps:
      - uses: actions/deploy-pages@v4
        id: deployment
```

## 8. Create Placeholder Content

```bash
# Create directory structure
mkdir -p docs/module-1-ros2 docs/module-2-simulation docs/module-3-nvidia-isaac docs/module-4-vla docs/hardware docs/appendix

# Create placeholder files
for module in module-1-ros2 module-2-simulation module-3-nvidia-isaac module-4-vla; do
  echo "---
sidebar_position: 1
---
# ${module} Overview

Coming soon." > docs/${module}/index.md
done
```

## 9. Local Development

```bash
# Start development server
npm run start

# Build for production
npm run build

# Serve production build locally
npm run serve
```

## 10. Verification Checklist

- [ ] `npm run build` completes without errors
- [ ] All 4 modules visible in sidebar
- [ ] Theme toggle works (dark/light)
- [ ] Search shows results
- [ ] Cover page displays with scrollytelling
- [ ] Mobile responsive layout works
- [ ] Lighthouse score 90+ on mobile

## Common Issues

| Issue | Solution |
|-------|----------|
| Build fails on broken links | Check all internal links match file paths |
| Search not working | Ensure `@easyops-cn/docusaurus-search-local` is in plugins |
| Theme toggle missing | Verify `colorMode.disableSwitch: false` in config |
| GitHub Pages 404 | Check `baseUrl` matches repository name |
