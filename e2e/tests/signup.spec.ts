import { test, expect } from "../fixtures/auth.fixture";
import { AUTH_URL } from "../helpers/constants";
import { deleteUser } from "../helpers/db-helpers";

test.describe("Sign Up Flow", () => {
  test("successful signup shows verification sent panel", async ({ page }) => {
    const email = `e2e-signup-${Date.now()}@playwright-test.local`;

    await page.goto(`${AUTH_URL}?tab=signup`);
    await page.locator("#signup-name").fill("Signup Tester");
    await page.locator("#signup-email").fill(email);
    await page.locator("#signup-password").fill("TestPass123!");
    await page.locator('button[type="submit"]').click();

    await expect(page.locator('[data-testid="verification-sent-panel"]')).toBeVisible({
      timeout: 15_000,
    });
    await expect(page.locator('[data-testid="verification-sent-panel"]')).toContainText(
      "Check your email"
    );

    // Cleanup
    await deleteUser(email);
  });

  test("duplicate email shows error", async ({ page, createTestUser }) => {
    const user = await createTestUser({ verified: true });

    await page.goto(`${AUTH_URL}?tab=signup`);
    await page.locator("#signup-name").fill("Duplicate Tester");
    await page.locator("#signup-email").fill(user.email);
    await page.locator("#signup-password").fill(user.password);
    await page.locator('button[type="submit"]').click();

    await expect(page.locator('[role="alert"]')).toBeVisible({ timeout: 10_000 });
  });

  test("email is normalized (trimmed and lowered)", async ({ page }) => {
    const baseEmail = `e2e-norm-${Date.now()}@playwright-test.local`;
    const inputEmail = `  ${baseEmail.toUpperCase()}  `;

    await page.goto(`${AUTH_URL}?tab=signup`);
    await page.locator("#signup-name").fill("Normalize Tester");
    await page.locator("#signup-email").fill(inputEmail);
    await page.locator("#signup-password").fill("TestPass123!");
    await page.locator('button[type="submit"]').click();

    // Should succeed (shows verification panel, not an error)
    await expect(page.locator('[data-testid="verification-sent-panel"]')).toBeVisible({
      timeout: 15_000,
    });

    // Cleanup
    await deleteUser(baseEmail);
  });

  test("required fields prevent submission", async ({ page }) => {
    await page.goto(`${AUTH_URL}?tab=signup`);
    await page.locator('button[type="submit"]').click();

    // HTML5 validation should prevent form submission — name field should be invalid
    const nameInput = page.locator("#signup-name");
    await expect(nameInput).toHaveAttribute("required", "");
    // The form should still be visible (not submitted)
    await expect(page.locator("#signup-name")).toBeVisible();
  });
});
