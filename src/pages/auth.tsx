import React, { useMemo } from "react";
import Layout from "@theme/Layout";
import BrowserOnly from "@docusaurus/BrowserOnly";
import useBaseUrl from "@docusaurus/useBaseUrl";
import AuthForm from "@site/src/components/Auth/AuthForm";
import ResetPasswordForm from "@site/src/components/Auth/ResetPasswordForm";
import { useAuth } from "@site/src/contexts/AuthContext";

type Tab = "signup" | "signin" | "reset";

function AuthPageContent(): React.JSX.Element {
  const { user } = useAuth();
  const baseUrl = useBaseUrl("/");

  const { tab, redirect, token } = useMemo(() => {
    const params = new URLSearchParams(window.location.search);
    return {
      tab: (params.get("tab") as Tab) || "signin",
      redirect: params.get("redirect") || baseUrl,
      token: params.get("token") || "",
    };
  }, [baseUrl]);

  const handleSuccess = () => {
    window.location.href = redirect;
  };

  // If already authenticated, redirect
  if (user) {
    window.location.href = redirect;
    return <div className="auth-page__loading">Redirecting...</div>;
  }

  return (
    <div className="auth-page">
      <div className="auth-page__card">
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
