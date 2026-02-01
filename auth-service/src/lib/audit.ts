/**
 * Authentication audit log service.
 * Records auth events to the auth_audit_log table for security monitoring.
 */

import type { Pool } from "pg";

export type AuthEventType =
  | "signup"
  | "login"
  | "logout"
  | "password_reset"
  | "account_delete"
  | "email_verify"
  | "login_failed"
  | "lockout";

interface AuditLogEntry {
  userId: string | null;
  eventType: AuthEventType;
  ipAddress: string | null;
  userAgent: string | null;
  success: boolean;
  metadata?: Record<string, unknown>;
}

export async function logAuthEvent(
  pool: Pool,
  entry: AuditLogEntry
): Promise<void> {
  try {
    await pool.query(
      `INSERT INTO auth_audit_log ("userId", "eventType", "ipAddress", "userAgent", success, metadata)
       VALUES ($1, $2, $3, $4, $5, $6)`,
      [
        entry.userId,
        entry.eventType,
        entry.ipAddress,
        entry.userAgent,
        entry.success,
        entry.metadata ? JSON.stringify(entry.metadata) : null,
      ]
    );
  } catch (error) {
    // Audit logging should never break the main flow
    console.error("Failed to log auth event:", error);
  }
}
