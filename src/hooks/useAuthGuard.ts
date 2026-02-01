import { useAuth } from "@site/src/contexts/AuthContext";
import useBaseUrl from "@docusaurus/useBaseUrl";

interface AuthGuardResult {
  isAuthenticated: boolean;
  isPending: boolean;
  user: Record<string, unknown> | null;
  error: Error | null;
  loginUrl: string;
}

/**
 * Hook for checking auth state and generating login URL with redirect.
 */
export function useAuthGuard(): AuthGuardResult {
  const { user, isPending, error } = useAuth();
  const baseAuthUrl = useBaseUrl("/auth");

  const currentPath =
    typeof window !== "undefined" ? window.location.pathname : "/";
  const loginUrl = `${baseAuthUrl}?tab=signin&redirect=${encodeURIComponent(currentPath)}`;

  return {
    isAuthenticated: !!user,
    isPending,
    user,
    error,
    loginUrl,
  };
}
