# Data Model: Docusaurus Setup

**Feature**: 001-docusaurus-setup
**Date**: 2026-01-18

## Overview

This feature is primarily a static site setup. The "data model" consists of content structures (MDX files) and configuration objects rather than database entities.

## Content Entities

### Module

Represents a major section of the textbook (4 total).

```typescript
interface Module {
  id: string;              // e.g., "module-1-ros2"
  title: string;           // e.g., "The Robotic Nervous System"
  subtitle: string;        // e.g., "ROS 2 Fundamentals"
  description: string;     // Short description for cards
  icon: string;            // Icon identifier or path
  color: string;           // Theme color for the module
  chapters: Chapter[];     // Ordered list of chapters
  weekRange: string;       // e.g., "Weeks 3-5"
}
```

**Instances** (from Constitution):
| ID | Title | Weeks |
|----|-------|-------|
| module-1-ros2 | The Robotic Nervous System (ROS 2) | 3-5 |
| module-2-simulation | The Digital Twin (Gazebo & Unity) | 6-7 |
| module-3-nvidia-isaac | The AI-Robot Brain (NVIDIA Isaac) | 8-10 |
| module-4-vla | Vision-Language-Action (VLA) | 11-13 |

### Chapter

Represents an individual lesson within a module.

```typescript
interface Chapter {
  id: string;              // e.g., "01-nodes-topics"
  title: string;           // e.g., "ROS 2 Nodes and Topics"
  slug: string;            // URL-friendly path segment
  moduleId: string;        // Parent module reference
  order: number;           // Position within module
  status: 'published' | 'draft' | 'coming-soon';
  frontmatter: ChapterFrontmatter;
}

interface ChapterFrontmatter {
  sidebar_position: number;
  title: string;
  description: string;     // SEO description (150-160 chars)
  keywords: string[];      // SEO keywords
}
```

### Section

Represents a subsection within a chapter (optimized for RAG).

```typescript
interface Section {
  id: string;              // Auto-generated from heading
  heading: string;         // H2 or H3 heading text
  level: 2 | 3;            // Heading level
  content: string;         // MDX content
  tokenCount: number;      // Target: 500-1000 tokens
  keywords: string[];      // Extracted keywords for RAG
}
```

### CodeBlock

Represents an executable code example.

```typescript
interface CodeBlock {
  language: string;        // e.g., "python", "bash", "yaml"
  filename?: string;       // Optional file path
  content: string;         // Code content
  executable: boolean;     // true unless marked [Conceptual]
  prerequisites?: string[];// Required dependencies
  testedWith?: string;     // e.g., "Python 3.11, ROS 2 Humble"
}
```

## Configuration Entities

### SiteConfig

Main Docusaurus configuration.

```typescript
interface SiteConfig {
  title: string;                    // "Physical AI & Humanoid Robotics"
  tagline: string;                  // Course tagline
  url: string;                      // GitHub Pages URL
  baseUrl: string;                  // Usually "/"
  organizationName: string;         // GitHub org/user
  projectName: string;              // Repository name
  themeConfig: ThemeConfig;
  plugins: PluginConfig[];
}

interface ThemeConfig {
  colorMode: {
    defaultMode: 'light' | 'dark';
    disableSwitch: false;
    respectPrefersColorScheme: true;
  };
  navbar: NavbarConfig;
  footer: FooterConfig;
  prism: PrismConfig;
}
```

### SidebarConfig

Navigation structure configuration.

```typescript
interface SidebarConfig {
  docs: SidebarItem[];
}

type SidebarItem =
  | string                          // Simple doc reference
  | {
      type: 'category';
      label: string;
      items: SidebarItem[];
      collapsed?: boolean;
      link?: { type: 'doc'; id: string };
    }
  | {
      type: 'doc';
      id: string;
      label?: string;
    };
```

## State Management

### Theme Preference

Stored in localStorage for persistence.

```typescript
interface ThemePreference {
  key: 'theme';
  value: 'light' | 'dark';
  storage: 'localStorage';
  default: 'system';  // Respects OS preference initially
}
```

### Search Index

Generated at build time, loaded on client.

```typescript
interface SearchIndex {
  documents: SearchDocument[];
  index: LunrIndex;  // Serialized Lunr.js index
}

interface SearchDocument {
  id: string;        // Document path
  title: string;
  content: string;   // Stripped MDX content
  url: string;
}
```

## File Structure Mapping

```
docs/
├── intro.md                           → Chapter (standalone)
├── module-1-ros2/
│   ├── index.md                       → Module landing (category link)
│   ├── 01-nodes-topics.md             → Chapter
│   ├── 02-services-actions.md         → Chapter
│   ├── 03-rclpy.md                    → Chapter
│   └── 04-urdf.md                     → Chapter
├── module-2-simulation/
│   └── ...                            → (same pattern)
├── module-3-nvidia-isaac/
│   └── ...                            → (same pattern)
├── module-4-vla/
│   └── ...                            → (same pattern)
├── hardware/
│   ├── workstation.md                 → Chapter
│   └── edge-kit.md                    → Chapter
└── appendix/
    ├── assessments.md                 → Chapter
    └── resources.md                   → Chapter
```

## Validation Rules

### Module Validation
- Must have exactly 4 modules (constitution requirement)
- Each module must have at least 1 chapter
- Module IDs must be URL-safe (lowercase, hyphens only)

### Chapter Validation
- Frontmatter required: sidebar_position, title, description
- Description length: 150-160 characters
- Keywords: minimum 3 per chapter

### Section Validation
- Token count target: 500-1000 (warning if outside range)
- No ambiguous pronouns across section boundaries
- Clear semantic heading required

### CodeBlock Validation
- Language tag required
- If executable=true, must be syntactically valid
- Prerequisites must be documented if external dependencies
