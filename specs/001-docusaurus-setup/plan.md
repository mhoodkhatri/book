# Implementation Plan: Docusaurus Setup

**Branch**: `001-docusaurus-setup` | **Date**: 2026-01-18 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-docusaurus-setup/spec.md`

## Summary

Establish the foundational Docusaurus 3.x documentation platform for the Physical AI & Humanoid Robotics Textbook. The implementation includes a custom landing page with scrollytelling animation, dark/light theme support, local search, and automated GitHub Pages deployment. The platform will support the 4-module curriculum structure and prepare for future RAG chatbot, personalization, and translation features.

## Technical Context

**Language/Version**: TypeScript 5.x, Node.js 18+
**Primary Dependencies**: Docusaurus 3.x, React 18.x, @easyops-cn/docusaurus-search-local, @docusaurus/plugin-ideal-image
**Storage**: N/A (static site, localStorage for theme preference)
**Testing**: Docusaurus build validation, Lighthouse audits, manual testing
**Target Platform**: GitHub Pages (static hosting), Modern browsers (Chrome, Firefox, Safari, Edge)
**Project Type**: Web (static documentation site)
**Performance Goals**: Lighthouse 90+, <3s initial load, <1s search results
**Constraints**: Static-only (no server-side rendering), GitHub Pages compatible, GDPR-compliant analytics
**Scale/Scope**: 4 modules, ~20 chapters, estimated 50-100 pages at full content

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| I. Docusaurus-First | ✅ PASS | Using Docusaurus 3.x as mandated |
| II. Spec-Driven Development | ✅ PASS | Following SDD workflow |
| III. RAG-First Content Design | ✅ PASS | Structure supports RAG (separate feature) |
| IV. Modular Content Architecture | ✅ PASS | 4-module structure implemented |
| V. Code-Content Parity | ⏸️ DEFER | Applies to content writing (separate feature) |
| VI. Accessibility-First | ✅ PASS | Dark/light theme, responsive, semantic HTML |
| VII. Security and Data Integrity | ✅ PASS | No user data in this feature, GDPR-compliant analytics |

**Stack Compliance**:
| Component | Required | Implemented |
|-----------|----------|-------------|
| Documentation | Docusaurus | ✅ Docusaurus 3.x |
| Hosting | GitHub Pages | ✅ GitHub Actions deployment |

**Gate Result**: ✅ PASS - No violations

## Project Structure

### Documentation (this feature)

```text
specs/001-docusaurus-setup/
├── plan.md              # This file
├── spec.md              # Feature specification
├── research.md          # Phase 0 research decisions
├── data-model.md        # Content entity definitions
├── quickstart.md        # Implementation quickstart guide
├── contracts/           # Configuration schemas
│   ├── docusaurus-config.schema.json
│   ├── sidebar.schema.json
│   └── chapter-frontmatter.schema.json
└── tasks.md             # Phase 2 output (created by /sp.tasks)
```

### Source Code (repository root)

```text
# Docusaurus Static Site Structure
docs/
├── intro.md                        # Course overview
├── module-1-ros2/
│   ├── index.md                    # Module landing page
│   ├── 01-nodes-topics.md
│   ├── 02-services-actions.md
│   ├── 03-rclpy.md
│   └── 04-urdf.md
├── module-2-simulation/
│   ├── index.md
│   ├── 01-gazebo-basics.md
│   ├── 02-physics-sim.md
│   ├── 03-unity-integration.md
│   └── 04-sensors.md
├── module-3-nvidia-isaac/
│   ├── index.md
│   ├── 01-isaac-sim.md
│   ├── 02-isaac-ros.md
│   ├── 03-vslam.md
│   └── 04-nav2.md
├── module-4-vla/
│   ├── index.md
│   ├── 01-whisper.md
│   ├── 02-cognitive-planning.md
│   └── 03-capstone.md
├── hardware/
│   ├── workstation.md
│   └── edge-kit.md
└── appendix/
    ├── assessments.md
    └── resources.md

src/
├── components/
│   ├── CoverSection/
│   │   ├── index.tsx               # Book cover hero component
│   │   └── styles.module.css
│   ├── ModulesScrollytelling/
│   │   ├── index.tsx               # Scroll-triggered module reveal
│   │   └── styles.module.css
│   └── HomepageFeatures/
│       ├── index.tsx               # Feature cards (optional)
│       └── styles.module.css
├── pages/
│   └── index.tsx                   # Custom landing page
├── css/
│   └── custom.css                  # Theme customizations
└── theme/                          # Swizzled components (minimal)

static/
├── img/
│   ├── logo.svg                    # Site logo
│   ├── book-cover.png              # Book visual for cover page
│   └── module-icons/               # Icons for each module
└── favicon.ico

.github/
└── workflows/
    └── deploy.yml                  # GitHub Pages deployment

# Configuration files (root)
├── docusaurus.config.ts            # Main configuration
├── sidebars.ts                     # Navigation structure
├── package.json
├── tsconfig.json
└── babel.config.js
```

**Structure Decision**: Docusaurus standard structure with custom components in `src/components/` for the cover page and scrollytelling animation. Minimal theme swizzling to maintain upgrade compatibility.

## Architecture Decisions

### AD-001: Local Search over Algolia

**Decision**: Use `@easyops-cn/docusaurus-search-local` instead of Algolia DocSearch

**Rationale**:
- No external service dependency (works offline)
- Zero cost, no API key management
- Sufficient for textbook scale (~100 pages)
- GDPR-compliant (no external requests)

**Trade-offs**: Less powerful than Algolia for very large sites, but appropriate for this use case.

### AD-002: Native Intersection Observer for Scrollytelling

**Decision**: Use browser's Intersection Observer API with custom React hooks

**Rationale**:
- No additional dependencies
- Full control over animation timing
- Lightweight bundle impact
- Works with React 18 concurrent features

**Trade-offs**: More custom code than using a library, but avoids external dependency.

### AD-003: Plausible for Analytics

**Decision**: Use Plausible Analytics (cloud or self-hosted)

**Rationale**:
- No cookies required (GDPR-compliant)
- Lightweight (~1KB script)
- Simple integration
- Respects user privacy

**Trade-offs**: Less detailed than Google Analytics, but sufficient for page view insights.

### AD-004: CSS Custom Properties for Theming

**Decision**: Use CSS custom properties (variables) for dark/light theme

**Rationale**:
- Native browser support
- Works with Docusaurus's built-in color mode
- Easy to maintain and extend
- No runtime JavaScript for theme switching

## Component Dependencies

```
┌─────────────────────────────────────────────────────────────┐
│                     docusaurus.config.ts                      │
│   (Site configuration, plugins, theme settings)              │
└─────────────────────┬───────────────────────────────────────┘
                      │
        ┌─────────────┼─────────────┐
        ▼             ▼             ▼
┌───────────┐  ┌───────────┐  ┌───────────┐
│ sidebars  │  │  plugins  │  │  theme    │
│   .ts     │  │           │  │  config   │
└─────┬─────┘  └─────┬─────┘  └─────┬─────┘
      │              │              │
      ▼              ▼              ▼
┌───────────┐  ┌───────────┐  ┌───────────┐
│   docs/   │  │  search   │  │ src/css/  │
│  content  │  │  ideal-   │  │ custom.css│
│           │  │  image    │  │           │
└───────────┘  └───────────┘  └───────────┘

┌─────────────────────────────────────────────────────────────┐
│                    src/pages/index.tsx                       │
│              (Custom Landing Page)                           │
└─────────────────────┬───────────────────────────────────────┘
                      │
        ┌─────────────┼─────────────┐
        ▼             ▼             ▼
┌───────────┐  ┌───────────┐  ┌───────────┐
│  Cover    │  │ Scrolly-  │  │ Homepage  │
│  Section  │  │ telling   │  │ Features  │
└───────────┘  └───────────┘  └───────────┘
```

## Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| Scrollytelling performance on mobile | Medium | Medium | Use CSS transforms, test on low-end devices |
| Search index too large | Low | Low | Docusaurus handles chunking automatically |
| GitHub Pages build timeout | Low | High | Optimize build, use caching in CI |
| Theme customization breaks on upgrade | Medium | Medium | Minimize swizzling, use CSS variables |

## Success Metrics

| Metric | Target | Measurement |
|--------|--------|-------------|
| Build time | <5 minutes | GitHub Actions timing |
| Lighthouse Performance | 90+ | Lighthouse CI |
| Lighthouse Accessibility | 90+ | Lighthouse CI |
| First Contentful Paint | <1.5s | Lighthouse CI |
| Time to Interactive | <3s | Lighthouse CI |
| Search response | <1s | Manual testing |
| Mobile usability | 100% | Lighthouse CI |

## Phase 2 Preparation

The next step is `/sp.tasks` which will generate the task breakdown. Key implementation areas:

1. **Project Initialization** - Docusaurus setup, dependencies
2. **Configuration** - docusaurus.config.ts, sidebars.ts
3. **Custom Components** - Cover page, scrollytelling
4. **Content Structure** - docs/ directory, placeholder chapters
5. **Styling** - Theme customization, dark/light mode
6. **Deployment** - GitHub Actions workflow
7. **Validation** - Build verification, Lighthouse audit

## Artifacts Generated

| Artifact | Path | Purpose |
|----------|------|---------|
| Research | [research.md](./research.md) | Technology decisions |
| Data Model | [data-model.md](./data-model.md) | Content entity definitions |
| Config Schema | [contracts/docusaurus-config.schema.json](./contracts/docusaurus-config.schema.json) | Configuration validation |
| Sidebar Schema | [contracts/sidebar.schema.json](./contracts/sidebar.schema.json) | Navigation validation |
| Frontmatter Schema | [contracts/chapter-frontmatter.schema.json](./contracts/chapter-frontmatter.schema.json) | Chapter validation |
| Quickstart | [quickstart.md](./quickstart.md) | Implementation guide |
