/**
 * Account lockout middleware.
 * Checks and enforces account lockout after 5 failed login attempts.
 * Lockout duration: 15 minutes.
 */

import type { Pool } from "pg";
import { logAuthEvent } from "../lib/audit.js";

const MAX_FAILED_ATTEMPTS = 5;
const LOCKOUT_DURATION_MS = 15 * 60 * 1000; // 15 minutes

interface LockoutCheckResult {
  locked: boolean;
  remainingMs?: number;
  userId?: string;
}

export async function checkLockout(
  pool: Pool,
  email: string
): Promise<LockoutCheckResult> {
  const result = await pool.query(
    `SELECT id, "failedLoginAttempts", "lockoutUntil" FROM "user" WHERE email = $1`,
    [email.toLowerCase().trim()]
  );

  if (result.rows.length === 0) {
    // User not found — don't reveal this, just allow the attempt to fail normally
    return { locked: false };
  }

  const user = result.rows[0];
  const failedAttempts = user.failedLoginAttempts || 0;
  const lockoutUntil = user.lockoutUntil ? new Date(user.lockoutUntil) : null;

  if (failedAttempts >= MAX_FAILED_ATTEMPTS && lockoutUntil && lockoutUntil > new Date()) {
    const remainingMs = lockoutUntil.getTime() - Date.now();
    return { locked: true, remainingMs, userId: user.id };
  }

  // If lockout has expired, reset the counter
  if (lockoutUntil && lockoutUntil <= new Date()) {
    await pool.query(
      `UPDATE "user" SET "failedLoginAttempts" = 0, "lockoutUntil" = NULL WHERE id = $1`,
      [user.id]
    );
  }

  return { locked: false, userId: user.id };
}

export async function recordFailedLogin(
  pool: Pool,
  email: string,
  ipAddress: string | null,
  userAgent: string | null
): Promise<void> {
  const result = await pool.query(
    `UPDATE "user"
     SET "failedLoginAttempts" = COALESCE("failedLoginAttempts", 0) + 1,
         "updatedAt" = NOW()
     WHERE email = $1
     RETURNING id, "failedLoginAttempts"`,
    [email.toLowerCase().trim()]
  );

  if (result.rows.length === 0) return;

  const user = result.rows[0];

  await logAuthEvent(pool, {
    userId: user.id,
    eventType: "login_failed",
    ipAddress,
    userAgent,
    success: false,
  });

  // If threshold reached, set lockout
  if (user.failedLoginAttempts >= MAX_FAILED_ATTEMPTS) {
    const lockoutUntil = new Date(Date.now() + LOCKOUT_DURATION_MS).toISOString();
    await pool.query(
      `UPDATE "user" SET "lockoutUntil" = $1 WHERE id = $2`,
      [lockoutUntil, user.id]
    );

    await logAuthEvent(pool, {
      userId: user.id,
      eventType: "lockout",
      ipAddress,
      userAgent,
      success: false,
      metadata: { lockoutUntil },
    });
  }
}
