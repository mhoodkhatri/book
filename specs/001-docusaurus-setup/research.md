# Research: Docusaurus Setup

**Feature**: 001-docusaurus-setup
**Date**: 2026-01-18
**Status**: Complete

## Research Tasks

### 1. Docusaurus v3 Best Practices

**Decision**: Use Docusaurus 3.x with TypeScript configuration

**Rationale**:
- Docusaurus 3.x is the latest stable version with improved performance
- TypeScript config provides better IDE support and type safety
- Native MDX 3.0 support with improved component handling
- Built-in dark mode support simplifies FR-011 implementation

**Alternatives Considered**:
- Docusaurus 2.x: Older, missing performance improvements
- Other SSGs (Astro, Next.js): Not mandated by constitution, would require ADR

### 2. Local Search Implementation

**Decision**: Use `@easyops-cn/docusaurus-search-local` plugin

**Rationale**:
- Most popular local search plugin for Docusaurus
- Zero external dependencies (works offline per FR-003)
- Supports Chinese/CJK characters (future-proofing for translations)
- Generates search index at build time
- Compatible with GitHub Pages static hosting

**Alternatives Considered**:
- Algolia DocSearch: Requires external API, rejected per clarification
- Lunr.js manual integration: More work, less maintained
- FlexSearch: Less Docusaurus integration support

### 3. Scrollytelling Animation Library

**Decision**: Use Intersection Observer API with custom React components

**Rationale**:
- Native browser API, no external dependency
- Full control over animation timing and behavior
- Lightweight (no library overhead)
- Works well with Docusaurus's React-based architecture
- Can be progressively enhanced (graceful degradation)

**Alternatives Considered**:
- ScrollMagic: Large library, overkill for simple reveal animations
- GSAP ScrollTrigger: Commercial license concerns, heavy
- Framer Motion: Good option but adds bundle size
- AOS (Animate on Scroll): Simpler but less control

**Implementation Approach**:
```jsx
// Custom hook for scroll-triggered animations
const useScrollReveal = (threshold = 0.1) => {
  const [isVisible, setIsVisible] = useState(false);
  const ref = useRef(null);

  useEffect(() => {
    const observer = new IntersectionObserver(
      ([entry]) => setIsVisible(entry.isIntersecting),
      { threshold }
    );
    if (ref.current) observer.observe(ref.current);
    return () => observer.disconnect();
  }, [threshold]);

  return [ref, isVisible];
};
```

### 4. Privacy-Friendly Analytics

**Decision**: Use Plausible Analytics (self-hosted script or cloud)

**Rationale**:
- No cookies required (GDPR-compliant without consent banner)
- Lightweight script (~1KB)
- Simple dashboard for page views
- Can be self-hosted or use cloud (free tier available)
- Easy Docusaurus integration via plugin or script injection

**Alternatives Considered**:
- Umami: Good alternative, requires self-hosting
- Fathom: Paid only
- Simple Analytics: More expensive
- GoatCounter: Less polished UI

**Implementation**:
```js
// docusaurus.config.js
scripts: [
  {
    src: 'https://plausible.io/js/script.js',
    defer: true,
    'data-domain': 'your-domain.github.io',
  },
],
```

### 5. Cover Page Design Pattern

**Decision**: Custom landing page component replacing default Docusaurus homepage

**Rationale**:
- Docusaurus supports custom pages in `src/pages/`
- Full control over hero section with book visual
- Can integrate scrollytelling directly
- Maintains Docusaurus routing and navigation

**Implementation Approach**:
- Create `src/pages/index.tsx` as custom landing
- Hero section with book cover image/3D visual
- Scrollytelling section with 4 module cards
- CTA buttons to enter documentation

### 6. Theme Customization Architecture

**Decision**: Swizzle key components + CSS custom properties

**Rationale**:
- Docusaurus "swizzling" allows component customization
- CSS custom properties enable dark/light theme switching
- Minimal swizzling reduces upgrade friction
- Theme toggle already built into Docusaurus (just needs styling)

**Components to Swizzle**:
- `Navbar` - Add custom logo and theme toggle styling
- `Footer` - Custom footer design
- `DocItem` - Add PersonalizeButton and TranslateButton placeholders (for future features)

**CSS Strategy**:
```css
:root {
  --ifm-color-primary: #2e8555;  /* Light theme */
  --book-cover-shadow: 0 4px 20px rgba(0,0,0,0.1);
}

[data-theme='dark'] {
  --ifm-color-primary: #25c2a0;  /* Dark theme */
  --book-cover-shadow: 0 4px 20px rgba(0,0,0,0.5);
}
```

### 7. GitHub Actions Deployment

**Decision**: Use official `peaceiris/actions-gh-pages` action

**Rationale**:
- Most popular GitHub Pages deployment action
- Handles branch deployment automatically
- Supports custom domains
- Well-documented and maintained

**Workflow Structure**:
```yaml
name: Deploy to GitHub Pages
on:
  push:
    branches: [main]
jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-node@v4
      - run: npm ci
      - run: npm run build
      - uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./build
```

### 8. Image Optimization

**Decision**: Use `@docusaurus/plugin-ideal-image` + WebP format

**Rationale**:
- Official Docusaurus plugin
- Automatic responsive image generation
- Lazy loading built-in
- WebP conversion for smaller file sizes
- Fallback for older browsers

**Configuration**:
```js
plugins: [
  [
    '@docusaurus/plugin-ideal-image',
    {
      quality: 70,
      max: 1030,
      min: 640,
      steps: 2,
      disableInDev: false,
    },
  ],
],
```

## Resolved Unknowns Summary

| Unknown | Resolution |
|---------|------------|
| Search plugin | @easyops-cn/docusaurus-search-local |
| Scrollytelling | Native Intersection Observer API |
| Analytics | Plausible Analytics |
| Cover page | Custom src/pages/index.tsx |
| Theme system | CSS custom properties + minimal swizzling |
| Deployment | peaceiris/actions-gh-pages |
| Image optimization | @docusaurus/plugin-ideal-image |

## Dependencies Identified

### NPM Packages
- `@docusaurus/core` ^3.x
- `@docusaurus/preset-classic` ^3.x
- `@easyops-cn/docusaurus-search-local` ^0.40.x
- `@docusaurus/plugin-ideal-image` ^3.x

### External Services
- GitHub Pages (hosting)
- Plausible Analytics (optional cloud or self-host)

### Development Tools
- Node.js 18+ (LTS)
- npm 9+
