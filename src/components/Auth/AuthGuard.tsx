import React from "react";
import { useAuthGuard } from "@site/src/hooks/useAuthGuard";
import { useAuth } from "@site/src/contexts/AuthContext";

interface Props {
  children: React.ReactNode;
  fallback?: React.ReactNode;
  featureName?: string;
}

/**
 * Wrapper that gates content behind authentication.
 * Shows a login prompt when the user is not authenticated.
 * Shows a service unavailable message when the auth service is down.
 */
export default function AuthGuard({
  children,
  fallback,
  featureName = "this feature",
}: Props): React.JSX.Element {
  const { isAuthenticated, isPending, loginUrl, error } = useAuthGuard();

  if (isPending) {
    return <div className="auth-guard__loading">Loading...</div>;
  }

  // Auth service unavailable
  if (error) {
    return (
      <div className="auth-guard__unavailable">
        <p>Service temporarily unavailable. Please try again later.</p>
      </div>
    );
  }

  if (isAuthenticated) {
    return <>{children}</>;
  }

  if (fallback) {
    return <>{fallback}</>;
  }

  return (
    <div className="auth-guard">
      <div className="auth-guard__prompt">
        <h3>Sign in required</h3>
        <p>Sign in to use {featureName}.</p>
        <div className="auth-guard__actions">
          <a href={loginUrl} className="auth-guard__btn auth-guard__btn--primary">
            Sign In
          </a>
          <a
            href={loginUrl.replace("tab=signin", "tab=signup")}
            className="auth-guard__btn auth-guard__btn--secondary"
          >
            Sign Up
          </a>
        </div>
      </div>
    </div>
  );
}
