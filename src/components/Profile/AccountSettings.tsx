import React, { useState, useCallback } from "react";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import { useAuth } from "@site/src/contexts/AuthContext";
import { useToast } from "@site/src/components/Auth/Toast";
import BackgroundForm from "./BackgroundForm";

export default function AccountSettings(): React.JSX.Element {
  const { siteConfig } = useDocusaurusContext();
  const authApiUrl = (siteConfig.customFields?.authApiUrl as string) || "http://localhost:3005";
  const { user, signOut } = useAuth();
  const { showToast } = useToast();
  const [showDeleteConfirm, setShowDeleteConfirm] = useState(false);
  const [deletePassword, setDeletePassword] = useState("");
  const [isDeleting, setIsDeleting] = useState(false);

  const existingSoftware = (() => {
    try {
      const val = user?.softwareBackground;
      return typeof val === "string" ? JSON.parse(val) : Array.isArray(val) ? val : [];
    } catch {
      return [];
    }
  })();

  const existingHardware = (() => {
    try {
      const val = user?.hardwareBackground;
      return typeof val === "string" ? JSON.parse(val) : Array.isArray(val) ? val : [];
    } catch {
      return [];
    }
  })();

  const handleDeleteAccount = useCallback(async () => {
    if (!deletePassword) {
      showToast("Please enter your password to confirm.", "error");
      return;
    }

    setIsDeleting(true);
    try {
      const response = await fetch(
        `${authApiUrl}/api/auth/custom/delete-account`,
        {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          credentials: "include",
          body: JSON.stringify({ password: deletePassword }),
        }
      );

      if (!response.ok) {
        const data = await response.json().catch(() => null);
        throw new Error(data?.error || "Failed to delete account");
      }

      showToast("Account deleted.", "success");
      await signOut();
      window.location.href = "/";
    } catch (err) {
      showToast(
        err instanceof Error ? err.message : "Failed to delete account",
        "error"
      );
    } finally {
      setIsDeleting(false);
    }
  }, [deletePassword, showToast, signOut]);

  if (!user) return <div>Not authenticated</div>;

  return (
    <div className="account-settings">
      <section className="account-settings__section">
        <h2>Account Info</h2>
        <div className="account-settings__info">
          <p>
            <strong>Name:</strong> {user.name as string}
          </p>
          <p>
            <strong>Email:</strong> {user.email as string}
          </p>
          <p>
            <strong>Email Verified:</strong>{" "}
            {user.emailVerified ? "Yes" : "No"}
          </p>
        </div>
      </section>

      <section className="account-settings__section">
        <h2>Background</h2>
        <BackgroundForm
          initialSoftware={existingSoftware}
          initialHardware={existingHardware}
          initialSoftwareOther={(user.softwareOther as string) || ""}
          initialHardwareOther={(user.hardwareOther as string) || ""}
          onSave={() => showToast("Background updated!", "success")}
          showSkip={false}
        />
      </section>

      <section className="account-settings__section account-settings__danger">
        <h2>Danger Zone</h2>
        {!showDeleteConfirm ? (
          <button
            className="account-settings__delete-btn"
            onClick={() => setShowDeleteConfirm(true)}
          >
            Delete Account
          </button>
        ) : (
          <div className="account-settings__delete-confirm">
            <p>
              This action is permanent. Enter your password to confirm account
              deletion.
            </p>
            <input
              type="password"
              autoComplete="current-password"
              value={deletePassword}
              onChange={(e) => setDeletePassword(e.target.value)}
              placeholder="Enter your password"
            />
            <div className="account-settings__delete-actions">
              <button
                className="account-settings__delete-btn account-settings__delete-btn--confirm"
                onClick={handleDeleteAccount}
                disabled={isDeleting}
              >
                {isDeleting ? "Deleting..." : "Confirm Delete"}
              </button>
              <button
                className="account-settings__delete-btn account-settings__delete-btn--cancel"
                onClick={() => {
                  setShowDeleteConfirm(false);
                  setDeletePassword("");
                }}
              >
                Cancel
              </button>
            </div>
          </div>
        )}
      </section>
    </div>
  );
}
