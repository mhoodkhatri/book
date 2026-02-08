import { describe, it, expect, vi, beforeEach } from "vitest";
import { renderHook } from "@testing-library/react";
import { useAuthGuard } from "./useAuthGuard";

// Mock the AuthContext
const mockUseAuth = vi.fn();
vi.mock("@site/src/contexts/AuthContext", () => ({
  useAuth: () => mockUseAuth(),
}));

describe("useAuthGuard", () => {
  beforeEach(() => {
    vi.clearAllMocks();
    // Set window.location.pathname for loginUrl generation
    Object.defineProperty(window, "location", {
      value: { pathname: "/docs/chapter-1" },
      writable: true,
    });
  });

  it("returns isAuthenticated=false and isVerified=false when no user", () => {
    mockUseAuth.mockReturnValue({
      user: null,
      isPending: false,
      error: null,
    });

    const { result } = renderHook(() => useAuthGuard());

    expect(result.current.isAuthenticated).toBe(false);
    expect(result.current.isVerified).toBe(false);
    expect(result.current.user).toBeNull();
  });

  it("returns isAuthenticated=false when user exists but emailVerified is false", () => {
    mockUseAuth.mockReturnValue({
      user: { id: "1", email: "test@test.com", emailVerified: false },
      isPending: false,
      error: null,
    });

    const { result } = renderHook(() => useAuthGuard());

    expect(result.current.isAuthenticated).toBe(false);
    expect(result.current.isVerified).toBe(false);
    expect(result.current.user).not.toBeNull();
  });

  it("returns isAuthenticated=false when user exists but emailVerified is null/undefined", () => {
    mockUseAuth.mockReturnValue({
      user: { id: "1", email: "test@test.com" },
      isPending: false,
      error: null,
    });

    const { result } = renderHook(() => useAuthGuard());

    expect(result.current.isAuthenticated).toBe(false);
    expect(result.current.isVerified).toBe(false);
  });

  it("returns isAuthenticated=true and isVerified=true when user has emailVerified=true", () => {
    mockUseAuth.mockReturnValue({
      user: { id: "1", email: "test@test.com", emailVerified: true },
      isPending: false,
      error: null,
    });

    const { result } = renderHook(() => useAuthGuard());

    expect(result.current.isAuthenticated).toBe(true);
    expect(result.current.isVerified).toBe(true);
  });

  it("returns isPending=true while session is loading", () => {
    mockUseAuth.mockReturnValue({
      user: null,
      isPending: true,
      error: null,
    });

    const { result } = renderHook(() => useAuthGuard());

    expect(result.current.isPending).toBe(true);
    expect(result.current.isAuthenticated).toBe(false);
  });

  it("passes through error from auth context", () => {
    const error = new Error("Auth service down");
    mockUseAuth.mockReturnValue({
      user: null,
      isPending: false,
      error,
    });

    const { result } = renderHook(() => useAuthGuard());

    expect(result.current.error).toBe(error);
  });

  it("generates loginUrl with redirect to current path", () => {
    mockUseAuth.mockReturnValue({
      user: null,
      isPending: false,
      error: null,
    });

    const { result } = renderHook(() => useAuthGuard());

    expect(result.current.loginUrl).toContain("/auth?tab=signin&redirect=");
    expect(result.current.loginUrl).toContain(
      encodeURIComponent("/docs/chapter-1")
    );
  });
});
