import React, { useMemo, useState, useEffect } from "react";
import Layout from "@theme/Layout";
import BrowserOnly from "@docusaurus/BrowserOnly";
import useBaseUrl from "@docusaurus/useBaseUrl";
import AuthForm from "@site/src/components/Auth/AuthForm";
import ResetPasswordForm from "@site/src/components/Auth/ResetPasswordForm";
import { useAuth } from "@site/src/contexts/AuthContext";

type Tab = "signup" | "signin" | "reset";

function AuthPageContent(): React.JSX.Element {
  const { user, refetch } = useAuth();
  const baseUrl = useBaseUrl("/");
  const [verifiedBanner, setVerifiedBanner] = useState(false);

  const { tab, redirect, token, verified } = useMemo(() => {
    const params = new URLSearchParams(window.location.search);
    return {
      tab: (params.get("tab") as Tab) || "signin",
      redirect: params.get("redirect") || baseUrl,
      token: params.get("token") || "",
      verified: params.get("verified") === "true",
    };
  }, [baseUrl]);

  // Handle verification callback — refetch session and show banner
  useEffect(() => {
    if (verified) {
      setVerifiedBanner(true);
      refetch();
    }
  }, [verified, refetch]);

  const handleSuccess = () => {
    window.location.href = redirect;
  };

  // Only auto-redirect if user is authenticated AND email is verified
  const isVerified = user && (user as Record<string, unknown>).emailVerified === true;

  if (isVerified) {
    window.location.href = redirect;
    return <div className="auth-page__loading">Redirecting...</div>;
  }

  return (
    <div className="auth-page">
      <div className="auth-page__card">
        {verifiedBanner && (
          <div className="auth-page__verified-banner" role="status">
            Email verified successfully! You can now sign in.
          </div>
        )}
        <h1 className="auth-page__title">
          {tab === "reset" ? "Reset Password" : "Welcome"}
        </h1>
        <p className="auth-page__subtitle">
          {tab === "reset"
            ? "Enter your new password below."
            : "Sign in to access interactive features like the AI chatbot and translation."}
        </p>
        {tab === "reset" ? (
          <ResetPasswordForm token={token} />
        ) : (
          <AuthForm initialTab={tab === "signup" ? "signup" : "signin"} onSuccess={handleSuccess} />
        )}
      </div>
    </div>
  );
}

export default function AuthPage(): React.JSX.Element {
  return (
    <Layout title="Sign In" description="Sign in or create an account">
      <BrowserOnly>{() => <AuthPageContent />}</BrowserOnly>
    </Layout>
  );
}
