import React from 'react';
import Layout from '@theme/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { CoverSection } from '../components/CoverSection';
import { AudienceSection } from '../components/AudienceSection';
import { ModulesScrollytelling } from '../components/ModulesScrollytelling';
import { OutcomesSection } from '../components/OutcomesSection';
import { ClosingCTA } from '../components/ClosingCTA';

export default function Home(): JSX.Element {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout
      title={siteConfig.title}
      description={siteConfig.tagline}
    >
      <CoverSection />
      <AudienceSection />
      <ModulesScrollytelling />
      <OutcomesSection />
      <ClosingCTA />
    </Layout>
  );
}
