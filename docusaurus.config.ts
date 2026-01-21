import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A comprehensive textbook for humanoid robotics development',
  favicon: 'img/favicon.ico',

  future: {
    v4: true,
  },

  url: 'https://your-username.github.io',
  baseUrl: '/book/',

  organizationName: 'your-username',
  projectName: 'book',

  onBrokenLinks: 'throw',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          editUrl: 'https://github.com/your-username/book/tree/main/',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  plugins: [
    '@docusaurus/plugin-ideal-image',
    [
      '@easyops-cn/docusaurus-search-local',
      {
        hashed: true,
        indexDocs: true,
        indexBlog: false,
        docsRouteBasePath: '/docs',
        language: ['en'],
        highlightSearchTermsOnTargetPage: true,
        explicitSearchResultPath: true,
      },
    ],
  ],

  scripts: [
    {
      src: 'https://plausible.io/js/script.js',
      defer: true,
      'data-domain': 'your-username.github.io',
    },
  ],

  themeConfig: {
    image: 'img/book-cover.png',
    colorMode: {
      defaultMode: 'light',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI Textbook',
      logo: {
        alt: 'Physical AI & Humanoid Robotics Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'docs',
          position: 'left',
          label: 'Textbook',
        },
        {
          href: 'https://github.com/your-username/book',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Textbook',
          items: [
            {
              label: 'Introduction',
              to: '/docs/intro',
            },
            {
              label: 'Module 1: ROS 2',
              to: '/docs/module-1-ros2',
            },
            {
              label: 'Module 2: Simulation',
              to: '/docs/module-2-simulation',
            },
          ],
        },
        {
          title: 'Advanced',
          items: [
            {
              label: 'Module 3: NVIDIA Isaac',
              to: '/docs/module-3-nvidia-isaac',
            },
            {
              label: 'Module 4: VLA',
              to: '/docs/module-4-vla',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'Hardware',
              to: '/docs/hardware/workstation',
            },
            {
              label: 'GitHub',
              href: 'https://github.com/your-username/book',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python', 'bash', 'yaml', 'cpp', 'cmake', 'markup', 'json'],
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
