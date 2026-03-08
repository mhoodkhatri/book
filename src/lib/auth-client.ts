import { createAuthClient } from "better-auth/react";
import siteConfig from "@generated/docusaurus.config";

export const AUTH_BASE_URL =
  (siteConfig.customFields?.authApiUrl as string) || "http://localhost:3005";

export const authClient = createAuthClient({
  baseURL: AUTH_BASE_URL,
  fetchOptions: {
    credentials: "include" as RequestCredentials,
  },
});

export const {
  useSession,
  signIn,
  signUp,
  signOut,
  resetPassword,
} = authClient;
