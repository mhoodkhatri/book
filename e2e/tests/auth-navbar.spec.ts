import { test, expect } from "../fixtures/auth.fixture";
import { AUTH_URL, BASE_URL } from "../helpers/constants";

test.describe("Auth Navbar", () => {
  test("unauthenticated user sees Sign In button", async ({ page }) => {
    await page.goto(BASE_URL);
    await expect(page.locator('[data-testid="navbar-signin"]')).toBeVisible();
    await expect(page.locator('[data-testid="navbar-signin"]')).toHaveText("Sign In");
  });

  test("authenticated user sees name and avatar", async ({ authenticatedPage }) => {
    await authenticatedPage.goto(BASE_URL);
    const userBtn = authenticatedPage.locator('[data-testid="navbar-user-menu"]');
    await expect(userBtn).toBeVisible({ timeout: 10_000 });

    // Should show the user's display name
    await expect(userBtn.locator(".auth-navbar__name")).toBeVisible();
    // Should show avatar initial
    await expect(userBtn.locator(".auth-navbar__avatar")).toBeVisible();
  });

  test("clicking user button toggles dropdown", async ({ authenticatedPage }) => {
    await authenticatedPage.goto(BASE_URL);
    const userBtn = authenticatedPage.locator('[data-testid="navbar-user-menu"]');
    await expect(userBtn).toBeVisible({ timeout: 10_000 });

    // Open dropdown
    await userBtn.click();
    const dropdown = authenticatedPage.locator('[role="menu"]');
    await expect(dropdown).toBeVisible();

    // Should have Account Settings and Sign Out
    await expect(dropdown.locator('[role="menuitem"]', { hasText: "Account Settings" })).toBeVisible();
    await expect(dropdown.locator('[role="menuitem"]', { hasText: "Sign Out" })).toBeVisible();

    // Click user button again to close
    await userBtn.click();
    await expect(dropdown).not.toBeVisible();
  });

  test("Sign Out option is present in dropdown menu", async ({ authenticatedPage }) => {
    await authenticatedPage.goto(BASE_URL);
    const userBtn = authenticatedPage.locator('[data-testid="navbar-user-menu"]');
    await expect(userBtn).toBeVisible({ timeout: 10_000 });
    await userBtn.click();

    const signOutBtn = authenticatedPage.locator('[role="menuitem"]', { hasText: "Sign Out" });
    await expect(signOutBtn).toBeVisible();
  });
});
