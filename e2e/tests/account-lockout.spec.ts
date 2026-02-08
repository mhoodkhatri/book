import { test, expect } from "../fixtures/auth.fixture";
import { AUTH_URL, LOCKOUT_THRESHOLD } from "../helpers/constants";
import { resetLockout, deleteUser } from "../helpers/db-helpers";

test.describe("Account Lockout", () => {
  // These tests make many sequential API calls to a remote DB, so they need more time
  test.describe.configure({ timeout: 60_000 });

  test("5 failed attempts triggers lockout message", async ({ page, createTestUser }) => {
    const user = await createTestUser({
      email: `e2e-lockout-${Date.now()}@playwright-test.local`,
      verified: true,
    });

    await page.goto(`${AUTH_URL}?tab=signin`);

    // Make 5 failed login attempts
    for (let i = 0; i < LOCKOUT_THRESHOLD; i++) {
      await page.locator("#signin-email").fill(user.email);
      await page.locator("#signin-password").fill("WrongPassword!");
      await page.locator('button[type="submit"]').click();

      // Wait for error to appear before next attempt
      await expect(page.locator('[role="alert"]')).toBeVisible({ timeout: 10_000 });

      // Clear fields for next attempt (if not locked out yet)
      if (i < LOCKOUT_THRESHOLD - 1) {
        await page.locator("#signin-email").clear();
        await page.locator("#signin-password").clear();
      }
    }

    // After threshold, try again — should see lockout message
    await page.locator("#signin-email").clear();
    await page.locator("#signin-password").clear();
    await page.locator("#signin-email").fill(user.email);
    await page.locator("#signin-password").fill("WrongPassword!");
    await page.locator('button[type="submit"]').click();

    await expect(page.locator('[role="alert"]')).toContainText(/locked|try again/i, {
      timeout: 10_000,
    });
  });

  test("lockout message shows remaining time", async ({ page, createTestUser }) => {
    const user = await createTestUser({
      email: `e2e-locktime-${Date.now()}@playwright-test.local`,
      verified: true,
    });

    await page.goto(`${AUTH_URL}?tab=signin`);

    // Exceed lockout threshold
    for (let i = 0; i <= LOCKOUT_THRESHOLD; i++) {
      await page.locator("#signin-email").fill(user.email);
      await page.locator("#signin-password").fill("WrongPassword!");
      await page.locator('button[type="submit"]').click();
      await expect(page.locator('[role="alert"]')).toBeVisible({ timeout: 10_000 });

      if (i < LOCKOUT_THRESHOLD) {
        await page.locator("#signin-email").clear();
        await page.locator("#signin-password").clear();
      }
    }

    // Lockout message should mention minutes
    await expect(page.locator('[role="alert"]')).toContainText(/minute/i, {
      timeout: 10_000,
    });
  });
});
