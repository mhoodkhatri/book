import React from "react";
import { describe, it, expect, vi, beforeEach } from "vitest";
import { render, screen, fireEvent, waitFor } from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import AuthForm from "./AuthForm";

// Mock auth client
const mockSignUpEmail = vi.fn();
const mockSignInEmail = vi.fn();
const mockSendVerificationEmail = vi.fn();

vi.mock("@site/src/lib/auth-client", () => ({
  authClient: {
    signUp: {
      email: (...args: unknown[]) => mockSignUpEmail(...args),
    },
    signIn: {
      email: (...args: unknown[]) => mockSignInEmail(...args),
    },
    sendVerificationEmail: (...args: unknown[]) =>
      mockSendVerificationEmail(...args),
  },
}));

// Mock Toast
const mockShowToast = vi.fn();
vi.mock("./Toast", () => ({
  useToast: () => ({ showToast: mockShowToast }),
}));

// Mock PasswordStrength
vi.mock("./PasswordStrength", () => ({
  default: () => React.createElement("div", { "data-testid": "password-strength" }),
}));

describe("AuthForm", () => {
  beforeEach(() => {
    vi.clearAllMocks();
  });

  describe("Signup flow", () => {
    it("shows verification sent panel after successful signup (does NOT call onSuccess)", async () => {
      const onSuccess = vi.fn();
      mockSignUpEmail.mockResolvedValue({ data: { user: { id: "1" } } });

      render(<AuthForm initialTab="signup" onSuccess={onSuccess} />);

      // Fill form
      await userEvent.type(screen.getByLabelText("Name"), "John Doe");
      await userEvent.type(
        screen.getByLabelText("Email"),
        "john@example.com"
      );
      await userEvent.type(
        screen.getByLabelText("Password"),
        "StrongPass123!"
      );

      // Submit
      fireEvent.click(screen.getByRole("button", { name: "Sign Up" }));

      await waitFor(() => {
        expect(screen.getByText("Check your email")).toBeInTheDocument();
      });

      // Verification panel should show the email
      expect(screen.getByText(/john@example.com/)).toBeInTheDocument();

      // onSuccess should NOT have been called
      expect(onSuccess).not.toHaveBeenCalled();

      // Resend button should be present
      expect(
        screen.getByRole("button", { name: "Resend verification email" })
      ).toBeInTheDocument();

      // Back to Sign In button should be present
      expect(
        screen.getByRole("button", { name: "Back to Sign In" })
      ).toBeInTheDocument();
    });

    it("shows verification failed panel when signup returns verification error", async () => {
      mockSignUpEmail.mockResolvedValue({
        error: { message: "Failed to send verification email" },
      });

      render(<AuthForm initialTab="signup" />);

      await userEvent.type(screen.getByLabelText("Name"), "John Doe");
      await userEvent.type(
        screen.getByLabelText("Email"),
        "john@example.com"
      );
      await userEvent.type(
        screen.getByLabelText("Password"),
        "StrongPass123!"
      );

      fireEvent.click(screen.getByRole("button", { name: "Sign Up" }));

      await waitFor(() => {
        expect(
          screen.getByText("Verification email failed")
        ).toBeInTheDocument();
      });

      expect(
        screen.getByText(
          /couldn't send the verification email/
        )
      ).toBeInTheDocument();
    });

    it("shows generic error for non-verification signup failures", async () => {
      mockSignUpEmail.mockResolvedValue({
        error: { message: "User already exists" },
      });

      render(<AuthForm initialTab="signup" />);

      await userEvent.type(screen.getByLabelText("Name"), "John Doe");
      await userEvent.type(
        screen.getByLabelText("Email"),
        "john@example.com"
      );
      await userEvent.type(
        screen.getByLabelText("Password"),
        "StrongPass123!"
      );

      fireEvent.click(screen.getByRole("button", { name: "Sign Up" }));

      await waitFor(() => {
        expect(screen.getByRole("alert")).toHaveTextContent(
          "User already exists"
        );
      });
    });

    it("shows generic error on signup exception", async () => {
      mockSignUpEmail.mockRejectedValue(new Error("Network error"));

      render(<AuthForm initialTab="signup" />);

      await userEvent.type(screen.getByLabelText("Name"), "John Doe");
      await userEvent.type(
        screen.getByLabelText("Email"),
        "john@example.com"
      );
      await userEvent.type(
        screen.getByLabelText("Password"),
        "StrongPass123!"
      );

      fireEvent.click(screen.getByRole("button", { name: "Sign Up" }));

      await waitFor(() => {
        expect(screen.getByRole("alert")).toHaveTextContent(
          "Sign up failed. Please try again."
        );
      });
    });

    it("Back to Sign In button returns to the sign-in form", async () => {
      mockSignUpEmail.mockResolvedValue({ data: { user: { id: "1" } } });

      render(<AuthForm initialTab="signup" />);

      await userEvent.type(screen.getByLabelText("Name"), "John Doe");
      await userEvent.type(
        screen.getByLabelText("Email"),
        "john@example.com"
      );
      await userEvent.type(
        screen.getByLabelText("Password"),
        "StrongPass123!"
      );

      fireEvent.click(screen.getByRole("button", { name: "Sign Up" }));

      await waitFor(() => {
        expect(screen.getByText("Check your email")).toBeInTheDocument();
      });

      // Click back
      fireEvent.click(
        screen.getByRole("button", { name: "Back to Sign In" })
      );

      // Should show the sign-in form now
      await waitFor(() => {
        expect(screen.getByLabelText("Email")).toBeInTheDocument();
        expect(
          screen.getByRole("button", { name: "Sign In" })
        ).toBeInTheDocument();
      });
    });

    it("resend button calls sendVerificationEmail with the signup email", async () => {
      mockSignUpEmail.mockResolvedValue({ data: { user: { id: "1" } } });
      mockSendVerificationEmail.mockResolvedValue({});

      render(<AuthForm initialTab="signup" />);

      await userEvent.type(screen.getByLabelText("Name"), "John Doe");
      await userEvent.type(
        screen.getByLabelText("Email"),
        "JOHN@Example.COM"
      );
      await userEvent.type(
        screen.getByLabelText("Password"),
        "StrongPass123!"
      );

      fireEvent.click(screen.getByRole("button", { name: "Sign Up" }));

      await waitFor(() => {
        expect(screen.getByText("Check your email")).toBeInTheDocument();
      });

      fireEvent.click(
        screen.getByRole("button", { name: "Resend verification email" })
      );

      await waitFor(() => {
        expect(mockSendVerificationEmail).toHaveBeenCalledWith({
          email: "john@example.com",
        });
      });
    });
  });

  describe("Signin flow", () => {
    it("calls onSuccess after successful sign-in", async () => {
      const onSuccess = vi.fn();
      mockSignInEmail.mockResolvedValue({
        data: { user: { id: "1" } },
      });

      render(<AuthForm initialTab="signin" onSuccess={onSuccess} />);

      await userEvent.type(screen.getByLabelText("Email"), "john@example.com");
      await userEvent.type(
        screen.getByLabelText("Password"),
        "StrongPass123!"
      );

      fireEvent.click(screen.getByRole("button", { name: "Sign In" }));

      await waitFor(() => {
        expect(onSuccess).toHaveBeenCalled();
      });
    });

    it("shows verification error with resend link on verification-related sign-in error", async () => {
      mockSignInEmail.mockResolvedValue({
        error: { message: "Please verify your email" },
      });

      render(<AuthForm initialTab="signin" />);

      await userEvent.type(screen.getByLabelText("Email"), "john@example.com");
      await userEvent.type(
        screen.getByLabelText("Password"),
        "StrongPass123!"
      );

      fireEvent.click(screen.getByRole("button", { name: "Sign In" }));

      await waitFor(() => {
        expect(
          screen.getByText("Please verify your email first.")
        ).toBeInTheDocument();
        expect(
          screen.getByText("Resend verification email")
        ).toBeInTheDocument();
      });
    });

    it("shows generic error for non-verification sign-in failures", async () => {
      mockSignInEmail.mockResolvedValue({
        error: { message: "Invalid credentials" },
      });

      render(<AuthForm initialTab="signin" />);

      await userEvent.type(screen.getByLabelText("Email"), "john@example.com");
      await userEvent.type(
        screen.getByLabelText("Password"),
        "StrongPass123!"
      );

      fireEvent.click(screen.getByRole("button", { name: "Sign In" }));

      await waitFor(() => {
        expect(screen.getByRole("alert")).toHaveTextContent(
          "Invalid email or password."
        );
      });
    });
  });

  describe("Tab switching", () => {
    it("switches between signin and signup tabs", async () => {
      render(<AuthForm initialTab="signin" />);

      // Should start on sign-in
      expect(
        screen.getByRole("button", { name: "Sign In" })
      ).toBeInTheDocument();

      // Switch to signup
      fireEvent.click(screen.getByRole("tab", { name: "Sign Up" }));

      expect(
        screen.getByRole("button", { name: "Sign Up" })
      ).toBeInTheDocument();
      expect(screen.getByLabelText("Name")).toBeInTheDocument();
    });
  });
});
