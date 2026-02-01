import { randomUUID } from "crypto";
import type { Request, Response } from "express";
import { auth, pool } from "../auth.js";
import { fromNodeHeaders } from "better-auth/node";
import { verifyPassword } from "better-auth/crypto";
import { logAuthEvent } from "../lib/audit.js";

export async function deleteAccountHandler(req: Request, res: Response): Promise<void> {
  try {
    // Validate session
    const session = await auth.api.getSession({
      headers: fromNodeHeaders(req.headers),
    });

    if (!session?.user) {
      res.status(401).json({ error: "Authentication required" });
      return;
    }

    const { password } = req.body;
    if (!password || typeof password !== "string") {
      res.status(400).json({ error: "Password is required to confirm deletion" });
      return;
    }

    // Verify password directly against the stored hash
    const accountResult = await pool.query(
      `SELECT password FROM account WHERE "userId" = $1 AND "providerId" = 'credential'`,
      [session.user.id]
    );

    if (accountResult.rows.length === 0) {
      res.status(400).json({ error: "No credential account found" });
      return;
    }

    const storedHash = accountResult.rows[0].password;
    const passwordValid = await verifyPassword({
      hash: storedHash,
      password,
    });

    if (!passwordValid) {
      res.status(401).json({ error: "Incorrect password" });
      return;
    }

    const userId = session.user.id;
    const anonymizedEmail = `deleted-${randomUUID()}@anonymized.local`;

    // Soft-delete: anonymize user data
    await pool.query(
      `UPDATE "user"
       SET name = 'Deleted User',
           email = $1,
           "emailVerified" = false,
           image = NULL,
           "softwareBackground" = NULL,
           "hardwareBackground" = NULL,
           "softwareOther" = NULL,
           "hardwareOther" = NULL,
           "backgroundCompleted" = false,
           "updatedAt" = NOW()
       WHERE id = $2`,
      [anonymizedEmail, userId]
    );

    // Revoke all sessions for this user
    await pool.query(`DELETE FROM session WHERE "userId" = $1`, [userId]);

    // Log the deletion event
    const forwarded = req.headers["x-forwarded-for"];
    const ipAddress = Array.isArray(forwarded) ? forwarded[0] : forwarded || req.ip || null;

    await logAuthEvent(pool, {
      userId,
      eventType: "account_delete",
      ipAddress,
      userAgent: req.headers["user-agent"] || null,
      success: true,
    });

    res.json({ success: true, message: "Account deleted" });
  } catch (error) {
    console.error("Account deletion error:", error);
    res.status(500).json({ error: "Internal server error" });
  }
}
