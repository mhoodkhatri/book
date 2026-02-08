import { Pool } from "pg";
import dotenv from "dotenv";
import path from "path";

dotenv.config({ path: path.resolve(__dirname, "..", "..", "auth-service", ".env") });

let pool: Pool | null = null;

export function getPool(): Pool {
  if (!pool) {
    pool = new Pool({
      connectionString: process.env.DATABASE_URL,
    });
  }
  return pool;
}

export async function closePool(): Promise<void> {
  if (pool) {
    await pool.end();
    pool = null;
  }
}

/** Mark a user's email as verified in the database */
export async function verifyUserEmail(email: string): Promise<void> {
  const db = getPool();
  await db.query(
    `UPDATE "user" SET "emailVerified" = true WHERE email = $1`,
    [email]
  );
}

/** Delete a test user and their sessions/accounts by email */
export async function deleteUser(email: string): Promise<void> {
  const db = getPool();
  const userResult = await db.query(
    `SELECT id FROM "user" WHERE email = $1`,
    [email]
  );
  if (userResult.rows.length === 0) return;

  const userId = userResult.rows[0].id;
  await db.query(`DELETE FROM "session" WHERE "userId" = $1`, [userId]);
  await db.query(`DELETE FROM "account" WHERE "userId" = $1`, [userId]);
  await db.query(`DELETE FROM "verification" WHERE "identifier" = $1`, [email]);
  await db.query(`DELETE FROM "user" WHERE id = $1`, [userId]);
}

/** Delete all test users matching the test email domain */
export async function deleteAllTestUsers(domain: string): Promise<void> {
  const db = getPool();
  const users = await db.query(
    `SELECT id, email FROM "user" WHERE email LIKE $1`,
    [`%@${domain}`]
  );
  for (const user of users.rows) {
    await deleteUser(user.email);
  }
}

/** Reset lockout fields for a user */
export async function resetLockout(email: string): Promise<void> {
  const db = getPool();
  await db.query(
    `UPDATE "user" SET "failedLoginAttempts" = 0, "lockoutUntil" = NULL WHERE email = $1`,
    [email]
  );
}

/** Create a user via the auth API (signup endpoint) */
export async function createUserViaAPI(
  name: string,
  email: string,
  password: string
): Promise<void> {
  const response = await fetch("http://localhost:3005/api/auth/sign-up/email", {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
      "Origin": "http://localhost:3000",
    },
    body: JSON.stringify({ name, email, password }),
  });
  if (!response.ok) {
    const text = await response.text();
    throw new Error(`Failed to create user ${email}: ${response.status} ${text}`);
  }
}

/** Check if auth service is healthy */
export async function healthCheck(): Promise<boolean> {
  try {
    const res = await fetch("http://localhost:3005/api/auth/ok");
    return res.ok;
  } catch {
    return false;
  }
}
