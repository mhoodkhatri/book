import { test, expect } from "../fixtures/auth.fixture";
import { AUTH_URL, BASE_URL } from "../helpers/constants";

test.describe("Auth Guard Behavior", () => {
  test("unauthenticated user sees Sign In button in navbar", async ({ page }) => {
    // Visit a docs page — unauthenticated users should see Sign In in navbar
    await page.goto(`${BASE_URL}docs/intro`);

    const signInBtn = page.locator('[data-testid="navbar-signin"]');
    await expect(signInBtn).toBeVisible({ timeout: 10_000 });
    await expect(signInBtn).toHaveText("Sign In");
  });

  test("Sign In navbar link navigates to auth page", async ({ page }) => {
    await page.goto(`${BASE_URL}docs/intro`);

    const signInBtn = page.locator('[data-testid="navbar-signin"]');
    await expect(signInBtn).toBeVisible({ timeout: 10_000 });
    await signInBtn.click();

    await page.waitForURL(/auth/, { timeout: 10_000 });
    // Should see the auth form
    await expect(page.locator("#signin-email")).toBeVisible();
  });

  test("authenticated user does not see Sign In button", async ({ authenticatedPage }) => {
    await authenticatedPage.goto(`${BASE_URL}docs/intro`);

    // Should NOT see Sign In button
    await expect(
      authenticatedPage.locator('[data-testid="navbar-signin"]')
    ).not.toBeVisible({ timeout: 10_000 });

    // Should see user menu instead
    await expect(
      authenticatedPage.locator('[data-testid="navbar-user-menu"]')
    ).toBeVisible({ timeout: 10_000 });
  });
});
