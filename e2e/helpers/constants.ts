export const BASE_URL = "http://localhost:3000/book/";
export const AUTH_URL = `${BASE_URL}auth`;
export const AUTH_API_URL = "http://localhost:3005";

export const TEST_USER = {
  name: "E2E Test User",
  email: "e2e-test@playwright-test.local",
  password: "TestPass123!",
};

export const UNVERIFIED_USER = {
  name: "Unverified Test User",
  email: "e2e-unverified@playwright-test.local",
  password: "TestPass123!",
};

/** Prefix used to identify test users for cleanup */
export const TEST_EMAIL_DOMAIN = "playwright-test.local";

export const LOCKOUT_THRESHOLD = 5;
