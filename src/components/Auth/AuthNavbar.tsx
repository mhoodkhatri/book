import React, { useState, useRef, useEffect } from "react";
import { createPortal } from "react-dom";
import { useAuth } from "@site/src/contexts/AuthContext";
import useBaseUrl from "@docusaurus/useBaseUrl";
import { useToast } from "./Toast";

export default function AuthNavbar(): React.JSX.Element {
  const { user, isPending, signOut } = useAuth();
  const { showToast } = useToast();
  const authUrl = useBaseUrl("/auth");
  const profileUrl = useBaseUrl("/profile/settings");
  const [dropdownOpen, setDropdownOpen] = useState(false);
  const dropdownRef = useRef<HTMLDivElement>(null);
  const [portalTarget, setPortalTarget] = useState<HTMLElement | null>(null);

  // Find the right navbar items container to portal into
  useEffect(() => {
    const target = document.querySelector<HTMLElement>(
      ".navbar__items.navbar__items--right"
    );
    if (target) setPortalTarget(target);
  }, []);

  // Close dropdown on outside click
  useEffect(() => {
    const handleClick = (e: MouseEvent) => {
      if (dropdownRef.current && !dropdownRef.current.contains(e.target as Node)) {
        setDropdownOpen(false);
      }
    };
    document.addEventListener("mousedown", handleClick);
    return () => document.removeEventListener("mousedown", handleClick);
  }, []);

  const handleSignOut = async () => {
    setDropdownOpen(false);
    await signOut();
    showToast("Signed out successfully.", "success");
  };

  // Read cached auth state to prevent sign-in button flash on page load
  const wasPreviouslySignedIn = (() => {
    try {
      return localStorage.getItem("auth-cached-signed-in") === "true";
    } catch {
      return false;
    }
  })();

  let content: React.JSX.Element;

  if (isPending) {
    // While loading, show placeholder matching the expected final state
    // If user was previously signed in, show avatar-shaped placeholder
    // If not, show sign-in button-shaped placeholder
    content = (
      <div className="auth-navbar">
        <div
          className={`auth-navbar__placeholder ${wasPreviouslySignedIn ? "auth-navbar__placeholder--avatar" : "auth-navbar__placeholder--btn"}`}
          aria-label="Loading auth state"
        />
      </div>
    );
  } else if (!user) {
    content = (
      <div className="auth-navbar auth-navbar--ready">
        <a href={`${authUrl}?tab=signin`} className="auth-navbar__signin-btn">
          Sign In
        </a>
      </div>
    );
  } else {
    const displayName = (user.name as string) || (user.email as string) || "User";
    content = (
      <div className="auth-navbar auth-navbar--ready" ref={dropdownRef}>
        <button
          className="auth-navbar__user-btn"
          onClick={() => setDropdownOpen(!dropdownOpen)}
          aria-expanded={dropdownOpen}
          aria-haspopup="menu"
        >
          <span className="auth-navbar__avatar">
            {displayName.charAt(0).toUpperCase()}
          </span>
          <span className="auth-navbar__name">{displayName}</span>
        </button>
        {dropdownOpen && (
          <div className="auth-navbar__dropdown" role="menu">
            <a
              href={profileUrl}
              className="auth-navbar__dropdown-item"
              role="menuitem"
              onClick={() => setDropdownOpen(false)}
            >
              Account Settings
            </a>
            <button
              className="auth-navbar__dropdown-item auth-navbar__dropdown-item--danger"
              role="menuitem"
              onClick={handleSignOut}
            >
              Sign Out
            </button>
          </div>
        )}
      </div>
    );
  }

  // Portal into the right navbar items container so it appears inline
  if (portalTarget) {
    return createPortal(content, portalTarget);
  }
  // Fallback: render in place (before portal target is found)
  return content;
}
