import React, { useState, useCallback } from "react";
import { authClient } from "@site/src/lib/auth-client";
import useBaseUrl from "@docusaurus/useBaseUrl";
import PasswordStrength from "./PasswordStrength";
import { useToast } from "./Toast";

type Tab = "signup" | "signin";

interface Props {
  initialTab?: Tab;
  onSuccess?: () => void;
}

export default function AuthForm({ initialTab = "signin", onSuccess }: Props): React.JSX.Element {
  const [tab, setTab] = useState<Tab>(initialTab);
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [showResendVerification, setShowResendVerification] = useState(false);
  const [resendCooldown, setResendCooldown] = useState(0);
  const { showToast } = useToast();
  const authUrl = useBaseUrl("/auth");

  // Sign-up fields
  const [signupName, setSignupName] = useState("");
  const [signupEmail, setSignupEmail] = useState("");
  const [signupPassword, setSignupPassword] = useState("");
  const [showPassword, setShowPassword] = useState(false);

  // Sign-in fields
  const [signinEmail, setSigninEmail] = useState("");
  const [signinPassword, setSigninPassword] = useState("");
  const [rememberMe, setRememberMe] = useState(false);

  const handleSignUp = useCallback(
    async (e: React.FormEvent) => {
      e.preventDefault();
      setIsSubmitting(true);
      setError(null);

      try {
        const result = await authClient.signUp.email({
          name: signupName.trim(),
          email: signupEmail.trim().toLowerCase(),
          password: signupPassword,
        });

        if (result.error) {
          setError(result.error.message || "Sign up failed. Please try again.");
          return;
        }

        showToast("Verification email sent — check your inbox!", "success");
        onSuccess?.();
      } catch {
        setError("Sign up failed. Please try again.");
      } finally {
        setIsSubmitting(false);
      }
    },
    [signupName, signupEmail, signupPassword, showToast, onSuccess]
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
      {/* Tab toggle */}
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

      {/* Sign-Up Form */}
      {tab === "signup" && (
        <form onSubmit={handleSignUp} className="auth-form__fields">
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
    </div>
  );
}
