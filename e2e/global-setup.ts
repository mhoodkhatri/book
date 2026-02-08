import {
  healthCheck,
  deleteAllTestUsers,
  createUserViaAPI,
  verifyUserEmail,
} from "./helpers/db-helpers";
import { TEST_USER, TEST_EMAIL_DOMAIN } from "./helpers/constants";

async function globalSetup(): Promise<void> {
  // 1. Health check — wait for auth service
  let healthy = false;
  for (let i = 0; i < 30; i++) {
    healthy = await healthCheck();
    if (healthy) break;
    await new Promise((r) => setTimeout(r, 1000));
  }
  if (!healthy) {
    throw new Error("Auth service not reachable at http://localhost:3005 after 30s");
  }

  // 2. Clean up any leftover test users
  await deleteAllTestUsers(TEST_EMAIL_DOMAIN);

  // 3. Seed a verified test user for signin tests
  try {
    await createUserViaAPI(TEST_USER.name, TEST_USER.email, TEST_USER.password);
    await verifyUserEmail(TEST_USER.email);
  } catch (err) {
    console.warn("Seed user creation warning (may already exist):", err);
  }
}

export default globalSetup;
