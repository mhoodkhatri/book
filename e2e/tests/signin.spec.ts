import { test, expect } from "../fixtures/auth.fixture";
import { AUTH_URL, TEST_USER } from "../helpers/constants";

test.describe("Sign In Flow", () => {
  test("verified user can sign in and is redirected", async ({ page }) => {
    await page.goto(`${AUTH_URL}?tab=signin`);
    await page.locator("#signin-email").fill(TEST_USER.email);
    await page.locator("#signin-password").fill(TEST_USER.password);
    await page.locator('button[type="submit"]').click();

    // Should redirect away from auth page after success
    await page.waitForURL((url) => !url.pathname.endsWith("/auth"), {
      timeout: 10_000,
    });
  });

  test("wrong password shows error", async ({ page }) => {
    await page.goto(`${AUTH_URL}?tab=signin`);
    await page.locator("#signin-email").fill(TEST_USER.email);
    await page.locator("#signin-password").fill("WrongPassword99!");
    await page.locator('button[type="submit"]').click();

    await expect(page.locator('[role="alert"]')).toBeVisible({ timeout: 10_000 });
    await expect(page.locator('[role="alert"]')).toContainText("Invalid email or password");
  });

  test("unverified user sees verification prompt", async ({ page, createTestUser }) => {
    const user = await createTestUser({ verified: false });

    await page.goto(`${AUTH_URL}?tab=signin`);
    await page.locator("#signin-email").fill(user.email);
    await page.locator("#signin-password").fill(user.password);
    await page.locator('button[type="submit"]').click();

    await expect(page.locator('[role="alert"]')).toBeVisible({ timeout: 10_000 });
    await expect(page.locator('[role="alert"]')).toContainText("verify");
  });

  test("remember me checkbox can be toggled", async ({ page }) => {
    await page.goto(`${AUTH_URL}?tab=signin`);
    const checkbox = page.locator('input[type="checkbox"]');

    await expect(checkbox).not.toBeChecked();
    await checkbox.check();
    await expect(checkbox).toBeChecked();
    await checkbox.uncheck();
    await expect(checkbox).not.toBeChecked();
  });

  test("redirect param is respected after signin", async ({ page }) => {
    const redirectPath = "/book/docs/intro";
    await page.goto(`${AUTH_URL}?tab=signin&redirect=${encodeURIComponent(redirectPath)}`);
    await page.locator("#signin-email").fill(TEST_USER.email);
    await page.locator("#signin-password").fill(TEST_USER.password);
    await page.locator('button[type="submit"]').click();

    await page.waitForURL((url) => url.pathname.includes("/docs/intro"), {
      timeout: 10_000,
    });
  });
});
