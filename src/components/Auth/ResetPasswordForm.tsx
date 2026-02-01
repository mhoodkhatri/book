import React, { useState, useCallback } from "react";
import { authClient, resetPassword as resetPasswordFn } from "@site/src/lib/auth-client";
import useBaseUrl from "@docusaurus/useBaseUrl";
import PasswordStrength from "./PasswordStrength";
import { useToast } from "./Toast";

interface Props {
  token: string;
}

export default function ResetPasswordForm({ token }: Props): React.JSX.Element {
  const { showToast } = useToast();
  const authUrl = useBaseUrl("/auth");
  const [newPassword, setNewPassword] = useState("");
  const [confirmPassword, setConfirmPassword] = useState("");
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [success, setSuccess] = useState(false);

  // If no token, show the "request reset" form
  const [requestEmail, setRequestEmail] = useState("");
  const [requestSent, setRequestSent] = useState(false);

  const handleRequestReset = useCallback(
    async (e: React.FormEvent) => {
      e.preventDefault();
      setIsSubmitting(true);
      setError(null);

      try {
        await authClient.requestPasswordReset({
          email: requestEmail.trim().toLowerCase(),
          redirectTo: `${authUrl}?tab=reset`,
        });
        setRequestSent(true);
        showToast(
          "If an account exists with that email, a reset link has been sent.",
          "success"
        );
      } catch {
        // Always show success to prevent account enumeration
        setRequestSent(true);
        showToast(
          "If an account exists with that email, a reset link has been sent.",
          "success"
        );
      } finally {
        setIsSubmitting(false);
      }
    },
    [requestEmail, showToast]
  );

  const handleResetPassword = useCallback(
    async (e: React.FormEvent) => {
      e.preventDefault();
      setError(null);

      if (newPassword !== confirmPassword) {
        setError("Passwords do not match.");
        return;
      }

      if (newPassword.length < 8) {
        setError("Password must be at least 8 characters.");
        return;
      }

      setIsSubmitting(true);

      try {
        const result = await resetPasswordFn({
          newPassword,
          token,
        });

        if (result.error) {
          setError(result.error.message || "Failed to reset password.");
          return;
        }

        setSuccess(true);
        showToast("Password reset successfully! Please sign in.", "success");
        setTimeout(() => {
          window.location.href = `${authUrl}?tab=signin`;
        }, 2000);
      } catch {
        setError("Failed to reset password. The link may have expired.");
      } finally {
        setIsSubmitting(false);
      }
    },
    [newPassword, confirmPassword, token, showToast]
  );

  // No token — show request form
  if (!token) {
    if (requestSent) {
      return (
        <div className="auth-form__success">
          <p>
            If an account exists with that email, a password reset link has been
            sent. Check your inbox.
          </p>
          <a href={`${authUrl}?tab=signin`} className="auth-form__back-link">
            Back to Sign In
          </a>
        </div>
      );
    }

    return (
      <form onSubmit={handleRequestReset} className="auth-form__fields">
        <p>Enter your email address and we will send you a reset link.</p>
        <div className="auth-form__field">
          <label htmlFor="reset-email">Email</label>
          <input
            id="reset-email"
            type="email"
            autoComplete="email"
            required
            value={requestEmail}
            onChange={(e) => setRequestEmail(e.target.value)}
            placeholder="you@example.com"
          />
        </div>
        <button
          type="submit"
          className="auth-form__submit"
          disabled={isSubmitting}
        >
          {isSubmitting ? "Sending..." : "Send Reset Link"}
        </button>
        <a href={`${authUrl}?tab=signin`} className="auth-form__back-link">
          Back to Sign In
        </a>
      </form>
    );
  }

  // Has token — show reset form
  if (success) {
    return (
      <div className="auth-form__success">
        <p>Password reset successfully! Redirecting to sign in...</p>
      </div>
    );
  }

  return (
    <form onSubmit={handleResetPassword} className="auth-form__fields">
      {error && (
        <div className="auth-form__error" role="alert">
          {error}
        </div>
      )}
      <div className="auth-form__field">
        <label htmlFor="new-password">New Password</label>
        <input
          id="new-password"
          type="password"
          autoComplete="new-password"
          required
          minLength={8}
          maxLength={128}
          value={newPassword}
          onChange={(e) => setNewPassword(e.target.value)}
          placeholder="Minimum 8 characters"
        />
        <PasswordStrength password={newPassword} />
      </div>
      <div className="auth-form__field">
        <label htmlFor="confirm-password">Confirm Password</label>
        <input
          id="confirm-password"
          type="password"
          autoComplete="new-password"
          required
          value={confirmPassword}
          onChange={(e) => setConfirmPassword(e.target.value)}
          placeholder="Re-enter your password"
        />
      </div>
      <button
        type="submit"
        className="auth-form__submit"
        disabled={isSubmitting}
      >
        {isSubmitting ? "Resetting..." : "Reset Password"}
      </button>
    </form>
  );
}
