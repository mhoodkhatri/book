import React, { useState, useRef, useEffect } from "react";
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

  // Loading skeleton
  if (isPending) {
    return (
      <div className="auth-navbar">
        <div className="auth-navbar__skeleton" aria-label="Loading auth state" />
      </div>
    );
  }

  // Unauthenticated
  if (!user) {
    return (
      <div className="auth-navbar">
        <a href={`${authUrl}?tab=signin`} className="auth-navbar__signin-btn">
          Sign In
        </a>
      </div>
    );
  }

  // Authenticated
  const displayName = (user.name as string) || (user.email as string) || "User";

  return (
    <div className="auth-navbar" ref={dropdownRef}>
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
