import React, { useState, useCallback, useEffect, useRef } from "react";
import { authClient, AUTH_BASE_URL } from "@site/src/lib/auth-client";
import useBaseUrl from "@docusaurus/useBaseUrl";
import PasswordStrength from "./PasswordStrength";
import { useToast } from "./Toast";

type Tab = "signup" | "signin";
type SignupState = "form" | "email_sent" | "verified" | "error";

const POLL_INTERVAL_MS = 4000; // Poll every 4 seconds

interface Props {
  initialTab?: Tab;
  onSuccess?: () => void;
  /** Set by parent when verification callback detected */
  verificationStatus?: "success" | "error" | null;
}

export default function AuthForm({ initialTab = "signin", onSuccess, verificationStatus }: Props): React.JSX.Element {
  const [tab, setTab] = useState<Tab>(initialTab);
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [showResendVerification, setShowResendVerification] = useState(false);
  const [resendCooldown, setResendCooldown] = useState(0);
  const { showToast } = useToast();
  const authUrl = useBaseUrl("/auth");

  // Sign-up state management (3 visual states)
  const [signupState, setSignupState] = useState<SignupState>(
    verificationStatus === "success" ? "verified" :
    verificationStatus === "error" ? "error" : "form"
  );
  const [sentEmail, setSentEmail] = useState("");

  // Sign-up fields
  const [signupName, setSignupName] = useState("");
  const [signupEmail, setSignupEmail] = useState("");
  const [signupPassword, setSignupPassword] = useState("");
  const [showPassword, setShowPassword] = useState(false);

  // Sign-in fields
  const [signinEmail, setSigninEmail] = useState("");
  const [signinPassword, setSigninPassword] = useState("");
  const [rememberMe, setRememberMe] = useState(false);

  // Ref to hold signup credentials for polling auto-sign-in
  const pendingCredentials = useRef<{ email: string; password: string } | null>(null);

  // Poll for verification: while on "email_sent" screen, check verification status
  // via a lightweight endpoint. Once verified, sign in once in Chrome automatically.
  useEffect(() => {
    if (signupState !== "email_sent" || !pendingCredentials.current) return;

    const { email, password } = pendingCredentials.current;
    let cancelled = false;

    const poll = async () => {
      try {
        // Lightweight check — no failed-login side effects
        const res = await fetch(
          `${AUTH_BASE_URL}/api/auth/custom/verification-status?email=${encodeURIComponent(email)}`
        );
        const data = await res.json();
        if (data.verified && !cancelled) {
          // Email verified — now sign in once in Chrome
          const result = await authClient.signIn.email({ email, password });
          if (!result.error && !cancelled) {
            setSignupState("verified");
            pendingCredentials.current = null;
            showToast("Email verified and signed in!", "success");
            setTimeout(() => { onSuccess?.(); }, 1200);
          }
        }
      } catch {
        // Network error — keep polling
      }
    };

    const id = setInterval(poll, POLL_INTERVAL_MS);
    poll();

    // Mobile browsers freeze timers on background tabs.
    // When user switches back from Gmail, poll immediately.
    const onVisible = () => {
      if (document.visibilityState === "visible" && !cancelled) {
        poll();
      }
    };
    document.addEventListener("visibilitychange", onVisible);

    return () => {
      cancelled = true;
      clearInterval(id);
      document.removeEventListener("visibilitychange", onVisible);
    };
  }, [signupState, showToast, onSuccess]);

  const handleSignUp = useCallback(
    async (e: React.FormEvent) => {
      e.preventDefault();
      setIsSubmitting(true);
      setError(null);

      const email = signupEmail.trim().toLowerCase();

      try {
        // Build callbackURL to redirect back to this page after verification
        const origin = window.location.origin;
        const basePath = authUrl.replace(/\/$/, "");
        const callbackURL = `${origin}${basePath}?tab=signup&verified=true`;

        const result = await authClient.signUp.email({
          name: signupName.trim(),
          email,
          password: signupPassword,
          callbackURL,
        });

        if (result.error) {
          setError(result.error.message || "Sign up failed. Please try again.");
          return;
        }

        // Store credentials for auto-sign-in polling
        pendingCredentials.current = { email, password: signupPassword };
        // Switch to email_sent state instead of redirecting
        setSentEmail(email);
        setSignupState("email_sent");
      } catch {
        setError("Sign up failed. Please try again.");
      } finally {
        setIsSubmitting(false);
      }
    },
    [signupName, signupEmail, signupPassword, authUrl]
  );

  const handleSignIn = useCallback(
    async (e: React.FormEvent) => {
      e.preventDefault();
      setIsSubmitting(true);
      setError(null);
      setShowResendVerification(false);

      try {
        const result = await authClient.signIn.email({
          email: signinEmail.trim().toLowerCase(),
          password: signinPassword,
          rememberMe,
        });

        if (result.error) {
          const msg = result.error.message || "";
          if (msg.toLowerCase().includes("verify") || msg.toLowerCase().includes("verification")) {
            setError("Please verify your email first.");
            setShowResendVerification(true);
          } else {
            setError("Invalid email or password.");
          }
          return;
        }

        showToast("Signed in successfully!", "success");
        onSuccess?.();
      } catch {
        setError("Invalid email or password.");
      } finally {
        setIsSubmitting(false);
      }
    },
    [signinEmail, signinPassword, rememberMe, showToast, onSuccess]
  );

  const handleResendVerification = useCallback(async () => {
    if (resendCooldown > 0) return;

    try {
      await authClient.sendVerificationEmail({
        email: signinEmail.trim().toLowerCase(),
      });
      showToast("Verification email sent!", "success");

      // Start 60-second cooldown
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
      showToast("Failed to resend verification email.", "error");
    }
  }, [signinEmail, resendCooldown, showToast]);

  return (
    <div className="auth-form">
      {/* Tab toggle — hidden when showing post-signup status */}
      {(tab !== "signup" || signupState === "form") && (
        <div className="auth-form__tabs" role="tablist">
          <button
            className={`auth-form__tab ${tab === "signin" ? "auth-form__tab--active" : ""}`}
            onClick={() => { setTab("signin"); setError(null); }}
            role="tab"
            aria-selected={tab === "signin"}
          >
            Sign In
          </button>
          <button
            className={`auth-form__tab ${tab === "signup" ? "auth-form__tab--active" : ""}`}
            onClick={() => { setTab("signup"); setError(null); }}
            role="tab"
            aria-selected={tab === "signup"}
          >
            Sign Up
          </button>
        </div>
      )}

      {error && (
        <div className="auth-form__error" role="alert">
          {error}
          {showResendVerification && (
            <button
              className="auth-form__resend-link"
              onClick={handleResendVerification}
              disabled={resendCooldown > 0}
            >
              {resendCooldown > 0
                ? `Resend verification (${resendCooldown}s)`
                : "Resend verification email"}
            </button>
          )}
        </div>
      )}

      {/* Sign-In Form */}
      {tab === "signin" && (
        <form onSubmit={handleSignIn} className="auth-form__fields">
          <div className="auth-form__field">
            <label htmlFor="signin-email">Email</label>
            <input
              id="signin-email"
              type="email"
              autoComplete="email"
              required
              value={signinEmail}
              onChange={(e) => setSigninEmail(e.target.value)}
              placeholder="you@example.com"
            />
          </div>
          <div className="auth-form__field">
            <label htmlFor="signin-password">Password</label>
            <input
              id="signin-password"
              type="password"
              autoComplete="current-password"
              required
              value={signinPassword}
              onChange={(e) => setSigninPassword(e.target.value)}
              placeholder="Enter your password"
            />
          </div>
          <div className="auth-form__options">
            <label className="auth-form__checkbox">
              <input
                type="checkbox"
                checked={rememberMe}
                onChange={(e) => setRememberMe(e.target.checked)}
              />
              Remember me
            </label>
            <a href={`${authUrl}?tab=reset`} className="auth-form__forgot-link">
              Forgot password?
            </a>
          </div>
          <button
            type="submit"
            className="auth-form__submit"
            disabled={isSubmitting}
          >
            {isSubmitting ? "Signing in..." : "Sign In"}
          </button>
        </form>
      )}

      {/* Sign-Up: 3 visual states */}
      {tab === "signup" && signupState === "form" && (
        <form onSubmit={handleSignUp} className="auth-form__fields auth-form__fade-in">
          <div className="auth-form__field">
            <label htmlFor="signup-name">Name</label>
            <input
              id="signup-name"
              type="text"
              autoComplete="name"
              required
              value={signupName}
              onChange={(e) => setSignupName(e.target.value)}
              placeholder="Your full name"
            />
          </div>
          <div className="auth-form__field">
            <label htmlFor="signup-email">Email</label>
            <input
              id="signup-email"
              type="email"
              autoComplete="email"
              required
              value={signupEmail}
              onChange={(e) => setSignupEmail(e.target.value)}
              placeholder="you@example.com"
            />
          </div>
          <div className="auth-form__field">
            <label htmlFor="signup-password">Password</label>
            <div className="auth-form__password-wrapper">
              <input
                id="signup-password"
                type={showPassword ? "text" : "password"}
                autoComplete="new-password"
                required
                minLength={8}
                maxLength={128}
                value={signupPassword}
                onChange={(e) => setSignupPassword(e.target.value)}
                placeholder="Minimum 8 characters"
              />
              <button
                type="button"
                className="auth-form__toggle-password"
                onClick={() => setShowPassword(!showPassword)}
                aria-label={showPassword ? "Hide password" : "Show password"}
              >
                {showPassword ? "Hide" : "Show"}
              </button>
            </div>
            <PasswordStrength password={signupPassword} />
          </div>
          <button
            type="submit"
            className="auth-form__submit"
            disabled={isSubmitting}
          >
            {isSubmitting ? "Creating account..." : "Sign Up"}
          </button>
        </form>
      )}

      {/* Sign-Up: Email Sent State */}
      {tab === "signup" && signupState === "email_sent" && (
        <div className="auth-form__status auth-form__fade-in" role="status" aria-live="polite">
          <div className="auth-form__status-icon auth-form__status-icon--email" aria-hidden="true">
            <svg width="48" height="48" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round">
              <rect x="2" y="4" width="20" height="16" rx="2" />
              <path d="M22 4L12 13L2 4" />
            </svg>
          </div>
          <h2 className="auth-form__status-heading">Check your email</h2>
          <p className="auth-form__status-text">
            We've sent a verification link to{" "}
            <strong>{sentEmail}</strong>.
            Please check your inbox and spam folder.
          </p>
          <p className="auth-form__status-hint">
            This page will automatically sign you in once you verify.
          </p>
          <p className="auth-form__status-hint">
            Didn't receive it? Check your spam folder or{" "}
            <button
              className="auth-form__resend-link"
              onClick={async () => {
                try {
                  await authClient.sendVerificationEmail({ email: sentEmail });
                  showToast("Verification email resent!", "success");
                } catch {
                  showToast("Failed to resend. Please try again.", "error");
                }
              }}
            >
              resend the email
            </button>.
          </p>
          <button
            className="auth-form__back-link"
            onClick={() => { setSignupState("form"); setError(null); }}
          >
            Back to sign up
          </button>
        </div>
      )}

      {/* Sign-Up: Verified State */}
      {tab === "signup" && signupState === "verified" && (
        <div className="auth-form__status auth-form__fade-in" role="status" aria-live="polite">
          <div className="auth-form__status-icon auth-form__status-icon--success" aria-hidden="true">
            <svg width="48" height="48" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
              <circle cx="12" cy="12" r="10" />
              <path d="M8 12l3 3 5-5" />
            </svg>
          </div>
          <h2 className="auth-form__status-heading">Email Verified Successfully!</h2>
          <p className="auth-form__status-text">
            You're signed in. Redirecting you now...
          </p>
          <div className="auth-form__redirect-bar" aria-hidden="true" />
        </div>
      )}

      {/* Sign-Up: Verification Error State */}
      {tab === "signup" && signupState === "error" && (
        <div className="auth-form__status auth-form__fade-in" role="alert">
          <div className="auth-form__status-icon auth-form__status-icon--error" aria-hidden="true">
            <svg width="48" height="48" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
              <circle cx="12" cy="12" r="10" />
              <path d="M15 9l-6 6M9 9l6 6" />
            </svg>
          </div>
          <h2 className="auth-form__status-heading">Verification Failed</h2>
          <p className="auth-form__status-text">
            The verification link is invalid or has expired.
          </p>
          <button
            className="auth-form__submit"
            onClick={() => { setSignupState("form"); setError(null); }}
          >
            Try signing up again
          </button>
        </div>
      )}
    </div>
  );
}
