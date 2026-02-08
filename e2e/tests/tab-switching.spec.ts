import { test, expect } from "@playwright/test";
import { AUTH_URL } from "../helpers/constants";

test.describe("Auth Tab Switching", () => {
  test("defaults to signin tab", async ({ page }) => {
    await page.goto(`${AUTH_URL}?tab=signin`);
    const signinTab = page.locator('[role="tab"][aria-selected="true"]');
    await expect(signinTab).toHaveText("Sign In");
    await expect(page.locator("#signin-email")).toBeVisible();
  });

  test("tab=signup activates signup form", async ({ page }) => {
    await page.goto(`${AUTH_URL}?tab=signup`);
    const signupTab = page.locator('[role="tab"][aria-selected="true"]');
    await expect(signupTab).toHaveText("Sign Up");
    await expect(page.locator("#signup-name")).toBeVisible();
    await expect(page.locator("#signup-email")).toBeVisible();
    await expect(page.locator("#signup-password")).toBeVisible();
  });

  test("clicking tabs switches forms", async ({ page }) => {
    await page.goto(`${AUTH_URL}?tab=signin`);
    await expect(page.locator("#signin-email")).toBeVisible();

    // Switch to signup
    await page.locator('[role="tab"]', { hasText: "Sign Up" }).click();
    await expect(page.locator("#signup-name")).toBeVisible();
    await expect(page.locator("#signin-email")).not.toBeVisible();

    // Switch back to signin
    await page.locator('[role="tab"]', { hasText: "Sign In" }).click();
    await expect(page.locator("#signin-email")).toBeVisible();
    await expect(page.locator("#signup-name")).not.toBeVisible();
  });

  test("switching tabs clears error messages", async ({ page }) => {
    await page.goto(`${AUTH_URL}?tab=signin`);

    // Submit empty form to trigger validation, then fill invalid creds
    await page.locator("#signin-email").fill("bad@example.com");
    await page.locator("#signin-password").fill("wrongpass");
    await page.locator('button[type="submit"]').click();

    // Wait for error to appear
    await expect(page.locator('[role="alert"]')).toBeVisible({ timeout: 10_000 });

    // Switch tab — error should clear
    await page.locator('[role="tab"]', { hasText: "Sign Up" }).click();
    await expect(page.locator('[role="alert"]')).not.toBeVisible();
  });
});
