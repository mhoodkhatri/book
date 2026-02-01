import React from "react";
import Layout from "@theme/Layout";
import BrowserOnly from "@docusaurus/BrowserOnly";
import useBaseUrl from "@docusaurus/useBaseUrl";
import AccountSettings from "@site/src/components/Profile/AccountSettings";
import { useAuth } from "@site/src/contexts/AuthContext";

function SettingsPageContent(): React.JSX.Element {
  const { user, isPending } = useAuth();
  const authUrl = useBaseUrl("/auth");
  const settingsUrl = useBaseUrl("/profile/settings");

  if (isPending) {
    return <div className="auth-page__loading">Loading...</div>;
  }

  if (!user) {
    window.location.href = `${authUrl}?tab=signin&redirect=${encodeURIComponent(settingsUrl)}`;
    return <div className="auth-page__loading">Redirecting to sign in...</div>;
  }

  return (
    <div className="auth-page">
      <div className="auth-page__card" style={{ maxWidth: 700 }}>
        <h1 className="auth-page__title">Account Settings</h1>
        <AccountSettings />
      </div>
    </div>
  );
}

export default function SettingsPage(): React.JSX.Element {
  return (
    <Layout title="Account Settings" description="Manage your account">
      <BrowserOnly>{() => <SettingsPageContent />}</BrowserOnly>
    </Layout>
  );
}
