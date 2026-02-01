import { createAuthClient } from "better-auth/react";

/**
 * Resolve the auth service base URL.
 * In Docusaurus, siteConfig.customFields is not available at module scope,
 * so we use a known fallback. For production, update the baseURL in
 * docusaurus.config.ts customFields.authApiUrl and this default.
 */
const AUTH_BASE_URL =
  typeof window !== "undefined" &&
  (window as any).__DOCUSAURUS_SITE_CONFIG__?.customFields?.authApiUrl
    ? (window as any).__DOCUSAURUS_SITE_CONFIG__.customFields.authApiUrl
    : "http://localhost:3005";

/**
 * Better-Auth React client for the Docusaurus frontend.
 */
export const authClient = createAuthClient({
  baseURL: AUTH_BASE_URL,
});

export const {
  useSession,
  signIn,
  signUp,
  signOut,
  resetPassword,
} = authClient;
