import React, { createContext, useContext, useEffect, useCallback } from "react";
import { useSession, signOut as authSignOut } from "@site/src/lib/auth-client";

interface AuthContextValue {
  user: Record<string, unknown> | null;
  session: Record<string, unknown> | null;
  isPending: boolean;
  error: Error | null;
  signOut: () => Promise<void>;
  refetch: () => void;
}

const AuthContext = createContext<AuthContextValue>({
  user: null,
  session: null,
  isPending: true,
  error: null,
  signOut: async () => {},
  refetch: () => {},
});

export function AuthContextProvider({
  children,
}: {
  children: React.ReactNode;
}): React.JSX.Element {
  const sessionQuery = useSession();

  const user = (sessionQuery.data?.user as Record<string, unknown>) || null;
  const session = (sessionQuery.data?.session as Record<string, unknown>) || null;
  const isPending = sessionQuery.isPending;
  const error = sessionQuery.error ? new Error(String(sessionQuery.error)) : null;

  const refetch = useCallback(() => {
    sessionQuery.refetch?.();
  }, [sessionQuery]);

  const handleSignOut = useCallback(async () => {
    await authSignOut();
    refetch();
  }, [refetch]);

  // Cross-tab sync via storage events
  useEffect(() => {
    const handleStorage = (e: StorageEvent) => {
      if (
        e.key === "better-auth-session-update" ||
        e.key?.startsWith("better-auth")
      ) {
        refetch();
      }
    };

    window.addEventListener("storage", handleStorage);
    return () => window.removeEventListener("storage", handleStorage);
  }, [refetch]);

  // Notify other tabs when auth state changes
  useEffect(() => {
    if (!isPending) {
      try {
        localStorage.setItem(
          "better-auth-session-update",
          String(Date.now())
        );
      } catch {
        // localStorage may be unavailable in SSR or private browsing
      }
    }
  }, [user, isPending]);

  return (
    <AuthContext.Provider
      value={{
        user,
        session,
        isPending,
        error,
        signOut: handleSignOut,
        refetch,
      }}
    >
      {children}
    </AuthContext.Provider>
  );
}

export function useAuth(): AuthContextValue {
  return useContext(AuthContext);
}

export default AuthContext;
