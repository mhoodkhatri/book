import { test as base, type Page } from "@playwright/test";
import {
  createUserViaAPI,
  verifyUserEmail,
  deleteUser,
  resetLockout,
} from "../helpers/db-helpers";
import { AUTH_URL, TEST_USER } from "../helpers/constants";

type AuthFixtures = {
  /** Creates a verified test user, returns credentials, deletes after test */
  createTestUser: (opts?: {
    name?: string;
    email?: string;
    password?: string;
    verified?: boolean;
  }) => Promise<{ name: string; email: string; password: string }>;

  /** A page already signed in as the default verified test user */
  authenticatedPage: Page;
};

export const test = base.extend<AuthFixtures>({
  createTestUser: async ({}, use) => {
    const createdEmails: string[] = [];

    await use(async (opts = {}) => {
      const name = opts.name ?? `Test User ${Date.now()}`;
      const email = opts.email ?? `e2e-${Date.now()}@playwright-test.local`;
      const password = opts.password ?? "TestPass123!";
      const verified = opts.verified ?? true;

      await createUserViaAPI(name, email, password);
      if (verified) {
        await verifyUserEmail(email);
      }
      createdEmails.push(email);

      return { name, email, password };
    });

    // Cleanup
    for (const email of createdEmails) {
      await resetLockout(email).catch(() => {});
      await deleteUser(email).catch(() => {});
    }
  },

  authenticatedPage: async ({ page }, use) => {
    // Sign in as the seeded verified test user
    await page.goto(`${AUTH_URL}?tab=signin`);
    await page.locator("#signin-email").fill(TEST_USER.email);
    await page.locator("#signin-password").fill(TEST_USER.password);
    await page.locator('button[type="submit"]').click();

    // Wait for successful navigation (redirect after sign-in)
    await page.waitForURL((url) => !url.pathname.endsWith("/auth"), {
      timeout: 15_000,
    });

    await use(page);
  },
});

export { expect } from "@playwright/test";
