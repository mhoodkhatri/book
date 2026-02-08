import React, { useState, useCallback } from "react";
import { useAuthGuard } from "@site/src/hooks/useAuthGuard";
import { authClient } from "@site/src/lib/auth-client";

interface Props {
  children: React.ReactNode;
  fallback?: React.ReactNode;
  featureName?: string;
}

/**
 * Wrapper that gates content behind authentication.
 * Shows a login prompt when the user is not authenticated.
 * Shows a verification prompt when the user is not email-verified.
 * Shows a service unavailable message when the auth service is down.
 */
export default function AuthGuard({
  children,
  fallback,
  featureName = "this feature",
}: Props): React.JSX.Element {
  const { isAuthenticated, isPending, loginUrl, error, user } = useAuthGuard();
  const [resendCooldown, setResendCooldown] = useState(0);
  const [resendStatus, setResendStatus] = useState<"idle" | "sent" | "failed">("idle");

  const handleResendVerification = useCallback(async () => {
    if (resendCooldown > 0 || !user?.email) return;

    try {
      await authClient.sendVerificationEmail({
        email: String(user.email),
      });
      setResendStatus("sent");
      setResendCooldown(60);
      const interval = setInterval(() => {
        setResendCooldown((prev) => {
          if (prev <= 1) {
            clearInterval(interval);
            return 0;
          }
          return prev - 1;
        });
      }, 1000);
    } catch {
      setResendStatus("failed");
    }
  }, [user, resendCooldown]);

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

  // User exists but email not verified
  if (user && user.emailVerified !== true) {
    return (
      <div className="auth-guard" data-testid="auth-guard-unverified">
        <div className="auth-guard__prompt">
          <h3>Email verification required</h3>
          <p>Please verify your email to use {featureName}.</p>
          {resendStatus === "sent" && (
            <p className="auth-guard__resend-success">Verification email sent!</p>
          )}
          {resendStatus === "failed" && (
            <p className="auth-guard__resend-error">Failed to send verification email. Please try again.</p>
          )}
          <div className="auth-guard__actions">
            <button
              className="auth-guard__btn auth-guard__btn--primary"
              onClick={handleResendVerification}
              disabled={resendCooldown > 0}
            >
              {resendCooldown > 0
                ? `Resend verification email (${resendCooldown}s)`
                : "Resend verification email"}
            </button>
          </div>
        </div>
      </div>
    );
  }

  if (fallback) {
    return <>{fallback}</>;
  }

  return (
    <div className="auth-guard" data-testid="auth-guard-unauthenticated">
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
