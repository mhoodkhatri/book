-- Migration: Create auth_audit_log table
-- Feature: 005-better-auth
-- Date: 2026-01-31

CREATE TABLE IF NOT EXISTS auth_audit_log (
  id SERIAL PRIMARY KEY,
  "userId" VARCHAR(36) REFERENCES "user"(id) ON DELETE SET NULL,
  "eventType" VARCHAR(50) NOT NULL,
  "ipAddress" VARCHAR(45),
  "userAgent" TEXT,
  success BOOLEAN NOT NULL DEFAULT true,
  metadata JSONB,
  "createdAt" TIMESTAMP NOT NULL DEFAULT NOW()
);

CREATE INDEX IF NOT EXISTS idx_audit_user_event
  ON auth_audit_log("userId", "eventType", "createdAt");
