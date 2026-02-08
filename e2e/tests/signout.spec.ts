import { test, expect } from "../fixtures/auth.fixture";
import { BASE_URL } from "../helpers/constants";

test.describe("Sign Out", () => {
  test("sign out clears session and shows toast", async ({ authenticatedPage }) => {
    await authenticatedPage.goto(BASE_URL);

    // Open user menu
    const userBtn = authenticatedPage.locator('[data-testid="navbar-user-menu"]');
    await expect(userBtn).toBeVisible({ timeout: 10_000 });
    await userBtn.click();

    // Click Sign Out
    await authenticatedPage.locator('[role="menuitem"]', { hasText: "Sign Out" }).click();

    // Should show Sign In button again (session cleared)
    await expect(authenticatedPage.locator('[data-testid="navbar-signin"]')).toBeVisible({
      timeout: 10_000,
    });
  });

  test("after sign out, navbar reverts to Sign In button", async ({ authenticatedPage }) => {
    await authenticatedPage.goto(BASE_URL);

    // Verify authenticated state
    const userBtn = authenticatedPage.locator('[data-testid="navbar-user-menu"]');
    await expect(userBtn).toBeVisible({ timeout: 10_000 });

    // Sign out
    await userBtn.click();
    await authenticatedPage.locator('[role="menuitem"]', { hasText: "Sign Out" }).click();

    // Navbar should now show Sign In
    await expect(authenticatedPage.locator('[data-testid="navbar-signin"]')).toBeVisible({
      timeout: 10_000,
    });

    // User menu button should not be visible
    await expect(authenticatedPage.locator('[data-testid="navbar-user-menu"]')).not.toBeVisible();
  });
});
