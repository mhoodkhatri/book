import { test, expect } from "@playwright/test";
import { AUTH_URL } from "../helpers/constants";

test.describe("Password Strength Meter", () => {
  test.beforeEach(async ({ page }) => {
    await page.goto(`${AUTH_URL}?tab=signup`);
  });

  test("no meter shown when password is empty", async ({ page }) => {
    await expect(page.locator('[data-testid="password-strength-label"]')).not.toBeVisible();
  });

  test("short password shows Weak", async ({ page }) => {
    await page.locator("#signup-password").fill("abc");
    await expect(page.locator('[data-testid="password-strength-label"]')).toHaveText("Weak");
  });

  test("mixed-case with digit shows Strong", async ({ page }) => {
    await page.locator("#signup-password").fill("Abcdefgh1");
    await expect(page.locator('[data-testid="password-strength-label"]')).toHaveText("Strong");
  });

  test("long mixed password with special char shows Very Strong", async ({ page }) => {
    await page.locator("#signup-password").fill("Abcdefgh1!xx");
    await expect(page.locator('[data-testid="password-strength-label"]')).toHaveText("Very Strong");
  });
});
