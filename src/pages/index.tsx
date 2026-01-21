import React from 'react';
import Layout from '@theme/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { CoverSection } from '../components/CoverSection';
import { ModulesScrollytelling } from '../components/ModulesScrollytelling';

export default function Home(): JSX.Element {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout
      title={siteConfig.title}
      description={siteConfig.tagline}
    >
      <CoverSection />
      <ModulesScrollytelling />
    </Layout>
  );
}
