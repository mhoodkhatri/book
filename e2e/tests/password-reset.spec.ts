import { test, expect } from "@playwright/test";
import { AUTH_URL } from "../helpers/constants";

test.describe("Password Reset", () => {
  test("request reset form shown when no token", async ({ page }) => {
    await page.goto(`${AUTH_URL}?tab=reset`);

    // Should show the request reset form with email input
    await expect(page.locator("#reset-email")).toBeVisible();
    await expect(page.locator('button[type="submit"]')).toHaveText("Send Reset Link");
  });

  test("submitting request shows success message", async ({ page }) => {
    await page.goto(`${AUTH_URL}?tab=reset`);
    await page.locator("#reset-email").fill("anyone@example.com");
    await page.locator('button[type="submit"]').click();

    await expect(page.locator('[data-testid="reset-request-sent"]')).toBeVisible({
      timeout: 10_000,
    });
    await expect(page.locator('[data-testid="reset-request-sent"]')).toContainText(
      "password reset link"
    );
  });

  test("token form shows password fields", async ({ page }) => {
    // When a token param is present, the reset form should show password fields
    await page.goto(`${AUTH_URL}?tab=reset&token=fake-token-123`);

    await expect(page.locator("#new-password")).toBeVisible();
    await expect(page.locator("#confirm-password")).toBeVisible();
    await expect(page.locator('button[type="submit"]')).toHaveText("Reset Password");
  });

  test("password mismatch shows error", async ({ page }) => {
    await page.goto(`${AUTH_URL}?tab=reset&token=fake-token-123`);

    await page.locator("#new-password").fill("NewPass123!");
    await page.locator("#confirm-password").fill("DifferentPass123!");
    await page.locator('button[type="submit"]').click();

    await expect(page.locator('[role="alert"]')).toBeVisible({ timeout: 5_000 });
    await expect(page.locator('[role="alert"]')).toContainText("do not match");
  });

  test("forgot password link navigates to reset form", async ({ page }) => {
    await page.goto(`${AUTH_URL}?tab=signin`);

    const forgotLink = page.locator(".auth-form__forgot-link");
    await expect(forgotLink).toBeVisible();
    await expect(forgotLink).toHaveText("Forgot password?");

    await forgotLink.click();
    await page.waitForURL(/tab=reset/, { timeout: 5_000 });
    await expect(page.locator("#reset-email")).toBeVisible();
  });
});
