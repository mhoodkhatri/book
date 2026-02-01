import React from "react";
import Layout from "@theme/Layout";
import BrowserOnly from "@docusaurus/BrowserOnly";
import useBaseUrl from "@docusaurus/useBaseUrl";
import BackgroundForm from "@site/src/components/Profile/BackgroundForm";
import { useAuth } from "@site/src/contexts/AuthContext";

function BackgroundPageContent(): React.JSX.Element {
  const { user, isPending } = useAuth();
  const authUrl = useBaseUrl("/auth");
  const backgroundUrl = useBaseUrl("/profile/background");

  if (isPending) {
    return <div className="auth-page__loading">Loading...</div>;
  }

  if (!user) {
    window.location.href = `${authUrl}?tab=signup&redirect=${encodeURIComponent(backgroundUrl)}`;
    return <div className="auth-page__loading">Redirecting to sign in...</div>;
  }

  const existingSoftware = (() => {
    try {
      const val = user.softwareBackground;
      return typeof val === "string" ? JSON.parse(val) : Array.isArray(val) ? val : [];
    } catch {
      return [];
    }
  })();

  const existingHardware = (() => {
    try {
      const val = user.hardwareBackground;
      return typeof val === "string" ? JSON.parse(val) : Array.isArray(val) ? val : [];
    } catch {
      return [];
    }
  })();

  return (
    <div className="auth-page">
      <div className="auth-page__card" style={{ maxWidth: 600 }}>
        <h1 className="auth-page__title">Tell us about yourself</h1>
        <p className="auth-page__subtitle">
          Help us personalize your learning experience by sharing your
          software and hardware background.
        </p>
        <BackgroundForm
          initialSoftware={existingSoftware}
          initialHardware={existingHardware}
          initialSoftwareOther={(user.softwareOther as string) || ""}
          initialHardwareOther={(user.hardwareOther as string) || ""}
          onSave={() => {
            window.location.href = "/";
          }}
          onSkip={() => {
            window.location.href = "/";
          }}
          showSkip={!user.backgroundCompleted}
        />
      </div>
    </div>
  );
}

export default function BackgroundPage(): React.JSX.Element {
  return (
    <Layout title="Your Background" description="Tell us about your experience">
      <BrowserOnly>{() => <BackgroundPageContent />}</BrowserOnly>
    </Layout>
  );
}
