# Tasks: Docusaurus Setup

**Input**: Design documents from `/specs/001-docusaurus-setup/`
**Prerequisites**: plan.md (required), spec.md (required), research.md, data-model.md, contracts/, quickstart.md

**Tests**: No automated tests requested in the specification. Validation via build verification and Lighthouse audits.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

Based on plan.md structure:
- **Root configuration**: `docusaurus.config.ts`, `sidebars.ts`, `package.json`, `tsconfig.json`
- **Documentation content**: `docs/`
- **Custom components**: `src/components/`
- **Custom pages**: `src/pages/`
- **Styling**: `src/css/`
- **Static assets**: `static/`
- **CI/CD**: `.github/workflows/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Initialize Docusaurus project and install all dependencies

- [X] T001 Initialize Docusaurus 3.x project with TypeScript template at repository root
- [X] T002 Install search plugin `@easyops-cn/docusaurus-search-local` via npm
- [X] T003 [P] Install image optimization plugin `@docusaurus/plugin-ideal-image` via npm
- [X] T004 [P] Create directory structure: `docs/`, `src/components/`, `src/pages/`, `src/css/`, `static/img/`
- [X] T005 [P] Configure TypeScript in tsconfig.json with strict mode and JSX support

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core configuration that MUST be complete before ANY user story can be implemented

**CRITICAL**: No user story work can begin until this phase is complete

- [X] T006 Configure docusaurus.config.ts with site metadata (title, tagline, url, baseUrl, organizationName, projectName)
- [X] T007 Configure themeConfig in docusaurus.config.ts (colorMode, navbar, prism syntax highlighting)
- [X] T008 [P] Configure plugins array in docusaurus.config.ts (search-local, ideal-image)
- [X] T009 [P] Add Plausible Analytics script configuration to docusaurus.config.ts
- [X] T010 Create sidebars.ts with 4-module structure per data-model.md
- [X] T011 [P] Create base CSS custom properties in src/css/custom.css for theming
- [X] T012 [P] Add placeholder logo.svg to static/img/logo.svg
- [X] T013 [P] Add favicon.ico to static/favicon.ico

**Checkpoint**: Foundation ready - user story implementation can now begin

---

## Phase 3: User Story 1 - View Textbook Content (Priority: P1)

**Goal**: Students and instructors can browse course modules, navigate chapters, and read content with a visually engaging cover page and scrollytelling animation

**Independent Test**: Deploy site locally with `npm run start`, verify cover page loads, scroll through module reveals, navigate to any chapter, toggle theme

### Content Structure for User Story 1

- [X] T014 [US1] Create docs/intro.md with course overview and frontmatter per contracts/chapter-frontmatter.schema.json
- [X] T015 [P] [US1] Create docs/module-1-ros2/index.md (Module 1 landing page)
- [X] T016 [P] [US1] Create docs/module-2-simulation/index.md (Module 2 landing page)
- [X] T017 [P] [US1] Create docs/module-3-nvidia-isaac/index.md (Module 3 landing page)
- [X] T018 [P] [US1] Create docs/module-4-vla/index.md (Module 4 landing page)
- [X] T019 [P] [US1] Create placeholder chapters for Module 1 in docs/module-1-ros2/ (nodes-topics.md, services-actions.md, rclpy.md, urdf.md)
- [X] T020 [P] [US1] Create placeholder chapters for Module 2 in docs/module-2-simulation/ (gazebo-basics.md, physics-sim.md, unity-integration.md, sensors.md)
- [X] T021 [P] [US1] Create placeholder chapters for Module 3 in docs/module-3-nvidia-isaac/ (isaac-sim.md, isaac-ros.md, vslam.md, nav2.md)
- [X] T022 [P] [US1] Create placeholder chapters for Module 4 in docs/module-4-vla/ (whisper.md, cognitive-planning.md, capstone.md)
- [X] T023 [P] [US1] Create docs/hardware/workstation.md and docs/hardware/edge-kit.md
- [X] T024 [P] [US1] Create docs/appendix/assessments.md and docs/appendix/resources.md

### Custom Components for User Story 1

- [X] T025 [US1] Create CoverSection component in src/components/CoverSection/index.tsx with book visual hero
- [X] T026 [US1] Create CoverSection styles in src/components/CoverSection/styles.module.css
- [X] T027 [US1] Create useScrollReveal hook using Intersection Observer API in src/hooks/useScrollReveal.ts
- [X] T028 [US1] Create ModulesScrollytelling component in src/components/ModulesScrollytelling/index.tsx
- [X] T029 [US1] Create ModulesScrollytelling styles in src/components/ModulesScrollytelling/styles.module.css
- [X] T030 [US1] Create custom landing page in src/pages/index.tsx integrating CoverSection and ModulesScrollytelling

### Static Assets for User Story 1

- [X] T031 [P] [US1] Add book cover placeholder image to static/img/book-cover.png
- [X] T032 [P] [US1] Create module icon placeholders in static/img/module-icons/ (ros2.svg, simulation.svg, isaac.svg, vla.svg)

### Theme Support for User Story 1

- [X] T033 [US1] Add dark/light theme CSS variables to src/css/custom.css for custom components
- [X] T034 [US1] Ensure theme toggle persists preference via Docusaurus built-in colorMode

**Checkpoint**: User Story 1 complete - cover page with scrollytelling, all modules navigable, theme toggle works

---

## Phase 4: User Story 2 - Search Content (Priority: P2)

**Goal**: Users can find specific topics using built-in local search functionality

**Independent Test**: Start dev server, press Ctrl+K or click search, type "ROS 2", verify results appear and link to correct chapters

### Search Configuration for User Story 2

- [X] T035 [US2] Verify @easyops-cn/docusaurus-search-local plugin configuration in docusaurus.config.ts
- [X] T036 [US2] Configure search plugin options (hashed: true, indexDocs: true, indexBlog: false, docsRouteBasePath)
- [X] T037 [US2] Add search-related keywords to chapter frontmatter in docs/**/*.md files
- [X] T038 [US2] Test search index generation during build process with `npm run build`

**Checkpoint**: User Story 2 complete - search dialog opens, returns relevant results, navigation works

---

## Phase 5: User Story 3 - Access on Mobile Devices (Priority: P2)

**Goal**: Students can read content comfortably on smartphones and tablets with responsive layout

**Independent Test**: Open site on mobile device or browser DevTools mobile emulation, verify layout adapts, hamburger menu works, code blocks scroll horizontally

### Mobile Responsiveness for User Story 3

- [X] T039 [US3] Add responsive breakpoints to src/css/custom.css for CoverSection component
- [X] T040 [US3] Add responsive styles to src/components/CoverSection/styles.module.css
- [X] T041 [P] [US3] Add responsive styles to src/components/ModulesScrollytelling/styles.module.css
- [X] T042 [US3] Ensure code block horizontal scrolling in src/css/custom.css
- [X] T043 [US3] Test hamburger menu navigation on mobile viewport
- [X] T044 [US3] Verify touch interactions for scrollytelling on mobile

**Checkpoint**: User Story 3 complete - responsive layout works, mobile navigation functional

---

## Phase 6: User Story 4 - Deploy Updates (Priority: P3)

**Goal**: Content updates automatically build and deploy to GitHub Pages when pushed to main branch

**Independent Test**: Push a change to main branch, verify GitHub Actions pipeline runs, check deployed site reflects changes

### CI/CD Pipeline for User Story 4

- [X] T045 [US4] Create .github/workflows/ directory structure
- [X] T046 [US4] Create deploy.yml GitHub Actions workflow in .github/workflows/deploy.yml
- [X] T047 [US4] Configure build job with Node.js 18, npm ci, npm run build
- [X] T048 [US4] Configure deploy job with actions/deploy-pages for GitHub Pages
- [X] T049 [US4] Add workflow permissions (contents: read, pages: write, id-token: write)
- [X] T050 [US4] Add concurrency settings to prevent conflicting deployments

**Checkpoint**: User Story 4 complete - push triggers build, successful builds deploy automatically

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Validation, optimization, and improvements affecting multiple user stories

- [X] T051 Run `npm run build` and verify zero errors
- [X] T052 [P] Verify all 4 module landing pages render correctly
- [X] T053 [P] Verify all internal links work (no broken links in build output)
- [ ] T054 Run Lighthouse audit on mobile and verify 90+ performance score
- [ ] T055 [P] Run Lighthouse audit and verify 90+ accessibility score
- [ ] T056 Verify First Contentful Paint < 1.5s target
- [ ] T057 Verify Time to Interactive < 3s target
- [X] T058 [P] Add meta tags for SEO in docusaurus.config.ts if missing
- [ ] T059 Test complete user journey: cover page → scroll → module → chapter → search → theme toggle
- [ ] T060 Document any deviations from quickstart.md in README or implementation notes

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phases 3-6)**: All depend on Foundational phase completion
  - User Story 1 (P1): Can proceed immediately after Foundational
  - User Story 2 (P2): Depends on US1 content being in place (for search indexing)
  - User Story 3 (P2): Depends on US1 components existing (for responsive styling)
  - User Story 4 (P3): Can proceed in parallel with US1-US3
- **Polish (Phase 7)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational - No dependencies on other stories
- **User Story 2 (P2)**: Requires content from US1 to test search (can start config in parallel)
- **User Story 3 (P2)**: Requires components from US1 to add responsive styles
- **User Story 4 (P3)**: Independent - can run in parallel with other stories

### Within Each User Story

- Content structure tasks marked [P] can run in parallel
- Components: Create component → Create styles → Integrate into page
- Configuration before implementation
- Build verification before deployment tasks

### Parallel Opportunities

**Phase 1 (Setup)**:
```
T002 (search plugin) | T003 (image plugin) - parallel
T004 (directories) | T005 (tsconfig) - parallel
```

**Phase 2 (Foundational)**:
```
T008 (plugins) | T009 (analytics) | T011 (CSS) | T012 (logo) | T013 (favicon) - parallel after T006, T007
```

**Phase 3 (US1 Content)**:
```
T015-T024 - All placeholder content files can be created in parallel
T031 | T032 - Static assets in parallel
```

**Phase 4-6 (US2, US3, US4)**:
```
US4 can run entirely in parallel with US2 and US3
T040 | T041 - Mobile styles in parallel
```

---

## Parallel Example: User Story 1 Content

```bash
# Launch all module landing pages in parallel:
Task: "Create docs/module-1-ros2/index.md"
Task: "Create docs/module-2-simulation/index.md"
Task: "Create docs/module-3-nvidia-isaac/index.md"
Task: "Create docs/module-4-vla/index.md"

# Launch all chapter placeholders in parallel:
Task: "Create placeholder chapters for Module 1"
Task: "Create placeholder chapters for Module 2"
Task: "Create placeholder chapters for Module 3"
Task: "Create placeholder chapters for Module 4"

# Launch static assets in parallel:
Task: "Add book cover placeholder"
Task: "Create module icon placeholders"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T005)
2. Complete Phase 2: Foundational (T006-T013)
3. Complete Phase 3: User Story 1 (T014-T034)
4. **STOP and VALIDATE**: Run `npm run start`, verify cover page, scrollytelling, navigation, theme toggle
5. Can deploy with just US1 as functional MVP

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → **MVP Deployed** (viewable textbook)
3. Add User Story 2 → Test independently → Deploy (searchable textbook)
4. Add User Story 3 → Test independently → Deploy (mobile-friendly textbook)
5. Add User Story 4 → Test independently → Deploy (auto-deploying textbook)
6. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (cover page, scrollytelling, content)
   - Developer B: User Story 4 (CI/CD pipeline) - fully independent
3. After US1 content exists:
   - Developer A: User Story 2 (search)
   - Developer B: User Story 3 (mobile responsiveness)
4. All stories integrate and validate in Polish phase

---

## Summary

| Phase | Task Count | Parallel Opportunities |
|-------|------------|----------------------|
| Phase 1: Setup | 5 | 4 tasks parallelizable |
| Phase 2: Foundational | 8 | 6 tasks parallelizable |
| Phase 3: US1 - View Content | 21 | 14 tasks parallelizable |
| Phase 4: US2 - Search | 4 | 0 tasks parallelizable |
| Phase 5: US3 - Mobile | 6 | 1 task parallelizable |
| Phase 6: US4 - Deploy | 6 | 0 tasks parallelizable |
| Phase 7: Polish | 10 | 4 tasks parallelizable |
| **Total** | **60** | **29 parallel opportunities** |

### Tasks per User Story

- User Story 1: 21 tasks (content, components, assets, theming)
- User Story 2: 4 tasks (search configuration)
- User Story 3: 6 tasks (responsive styling)
- User Story 4: 6 tasks (CI/CD pipeline)

### Suggested MVP Scope

Complete Phases 1-3 (Setup + Foundational + User Story 1) for a deployable MVP with:
- Cover page with book visual
- Scrollytelling module reveal
- All 4 modules navigable with placeholder content
- Dark/light theme toggle
- Basic navigation and content rendering

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Build validation via `npm run build` - should complete with zero errors
- Performance validation via Lighthouse audits
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
