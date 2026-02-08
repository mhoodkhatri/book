import { deleteAllTestUsers, closePool } from "./helpers/db-helpers";
import { TEST_EMAIL_DOMAIN } from "./helpers/constants";

async function globalTeardown(): Promise<void> {
  await deleteAllTestUsers(TEST_EMAIL_DOMAIN);
  await closePool();
}

export default globalTeardown;
