# Data Model: Better-Auth Authentication

**Feature**: 005-better-auth | **Date**: 2026-01-30 | **Branch**: `005-better-auth`

## Entity Relationship Diagram (Textual)

```
┌──────────────┐       ┌──────────────┐       ┌──────────────────┐
│     user     │──1:N──│   session    │       │   verification   │
│              │       │              │       │                  │
│ id (PK)      │       │ id (PK)      │       │ id (PK)          │
│ name         │       │ token        │       │ identifier       │
│ email (UQ)   │       │ userId (FK)  │       │ value            │
│ emailVerified│       │ expiresAt    │       │ expiresAt        │
│ image        │       │ ipAddress    │       │ createdAt        │
│ password*    │       │ userAgent    │       │ updatedAt        │
│ createdAt    │       │ createdAt    │       └──────────────────┘
│ updatedAt    │       │ updatedAt    │
│              │       └──────────────┘       ┌──────────────────┐
│ [additional] │                              │    account       │
│ softwareBg   │──1:N──────────────────────── │ id (PK)          │
│ hardwareBg   │                              │ userId (FK)      │
│ softwareOther│       ┌──────────────┐       │ accountId        │
│ hardwareOther│       │  auth_audit  │       │ providerId       │
│ bgCompleted  │──1:N──│              │       │ password (hash)  │
│ failedLogins │       │ id (PK)      │       │ createdAt        │
│ lockoutUntil │       │ userId (FK)  │       │ updatedAt        │
└──────────────┘       │ eventType    │       └──────────────────┘
                       │ ipAddress    │
                       │ userAgent    │
                       │ success      │
                       │ createdAt    │
                       └──────────────┘
```

## Tables

### 1. `user` (Better-Auth Core + Additional Fields)

Better-Auth creates this table automatically. We extend it with additional fields.

| Column | Type | Constraints | Notes |
|--------|------|-------------|-------|
| `id` | `VARCHAR(36)` | PK | Better-Auth generated UUID |
| `name` | `VARCHAR(255)` | NOT NULL | Display name |
| `email` | `VARCHAR(255)` | NOT NULL, UNIQUE | Normalized to lowercase (FR-007) |
| `emailVerified` | `BOOLEAN` | NOT NULL, DEFAULT false | Set true after verification (FR-003) |
| `image` | `TEXT` | NULLABLE | Profile image URL (optional) |
| `createdAt` | `TIMESTAMP` | NOT NULL | Auto-set by Better-Auth |
| `updatedAt` | `TIMESTAMP` | NOT NULL | Auto-set by Better-Auth |
| **Additional Fields** | | | |
| `softwareBackground` | `TEXT` | NULLABLE | JSON array: `["Python","ROS 2","C++"]` |
| `hardwareBackground` | `TEXT` | NULLABLE | JSON array: `["Jetson Orin","Laptop"]` |
| `softwareOther` | `VARCHAR(255)` | NULLABLE | Free-text "Other" entry |
| `hardwareOther` | `VARCHAR(255)` | NULLABLE | Free-text "Other" entry |
| `backgroundCompleted` | `BOOLEAN` | DEFAULT false | True after background form submit |
| `failedLoginAttempts` | `INTEGER` | DEFAULT 0 | Counter for lockout (FR-006) |
| `lockoutUntil` | `TIMESTAMP` | NULLABLE | Lockout expiry time (FR-006) |

**Validation Rules**:
- `email`: Must be valid email format, normalized to lowercase before storage
- `name`: 1-255 characters
- `softwareBackground`: JSON array of predefined values: `["Python", "ROS 2", "C++", "JavaScript", "MATLAB", "Bash/Shell"]`
- `hardwareBackground`: JSON array of predefined values: `["Jetson Orin", "Desktop Workstation", "Laptop", "Raspberry Pi", "Cloud/VM"]`

### 2. `session` (Better-Auth Core)

Fully managed by Better-Auth. No modifications needed.

| Column | Type | Constraints | Notes |
|--------|------|-------------|-------|
| `id` | `VARCHAR(36)` | PK | Session ID |
| `token` | `VARCHAR(255)` | NOT NULL, UNIQUE | Cookie value for auth |
| `userId` | `VARCHAR(36)` | FK → user.id | Session owner |
| `expiresAt` | `TIMESTAMP` | NOT NULL | 30 min (default) or 30 days (remember-me) |
| `ipAddress` | `VARCHAR(255)` | NULLABLE | Client IP |
| `userAgent` | `TEXT` | NULLABLE | Browser user agent |
| `createdAt` | `TIMESTAMP` | NOT NULL | |
| `updatedAt` | `TIMESTAMP` | NOT NULL | |

**Session Configuration**:
- Default expiry: 30 minutes idle (FR-009)
- Remember-me expiry: 30 days (FR-009)
- Token rotation on login (FR-010)
- Cookie: `better-auth.session_token`, HttpOnly, Secure, SameSite=None (FR-032)

### 3. `account` (Better-Auth Core)

Managed by Better-Auth. Stores credential info per auth provider.

| Column | Type | Constraints | Notes |
|--------|------|-------------|-------|
| `id` | `VARCHAR(36)` | PK | Account ID |
| `userId` | `VARCHAR(36)` | FK → user.id | Owner |
| `accountId` | `VARCHAR(255)` | NOT NULL | Provider-specific user ID |
| `providerId` | `VARCHAR(255)` | NOT NULL | `"credential"` for email/password |
| `password` | `TEXT` | NULLABLE | Hashed (scrypt) for credential provider |
| `accessToken` | `TEXT` | NULLABLE | For OAuth providers (future) |
| `refreshToken` | `TEXT` | NULLABLE | For OAuth providers (future) |
| `createdAt` | `TIMESTAMP` | NOT NULL | |
| `updatedAt` | `TIMESTAMP` | NOT NULL | |

### 4. `verification` (Better-Auth Core)

Managed by Better-Auth. Used for email verification and password reset tokens.

| Column | Type | Constraints | Notes |
|--------|------|-------------|-------|
| `id` | `VARCHAR(36)` | PK | Token ID |
| `identifier` | `VARCHAR(255)` | NOT NULL | Email or user identifier |
| `value` | `VARCHAR(255)` | NOT NULL | Token value (single-use, FR-017) |
| `expiresAt` | `TIMESTAMP` | NOT NULL | 24h for verification, 1h for reset (FR-016) |
| `createdAt` | `TIMESTAMP` | NULLABLE | |
| `updatedAt` | `TIMESTAMP` | NULLABLE | |

### 5. `auth_audit_log` (Custom Table)

Custom table for FR-035 (authentication event logging).

| Column | Type | Constraints | Notes |
|--------|------|-------------|-------|
| `id` | `SERIAL` | PK | Auto-increment |
| `userId` | `VARCHAR(36)` | NULLABLE, FK → user.id | NULL for failed attempts with unknown user |
| `eventType` | `VARCHAR(50)` | NOT NULL | Enum: signup, login, logout, password_reset, account_delete, email_verify, login_failed, lockout |
| `ipAddress` | `VARCHAR(45)` | NULLABLE | IPv4 or IPv6 |
| `userAgent` | `TEXT` | NULLABLE | Browser user agent |
| `success` | `BOOLEAN` | NOT NULL | True if action succeeded |
| `metadata` | `JSONB` | NULLABLE | Additional context (e.g., failure reason) |
| `createdAt` | `TIMESTAMP` | NOT NULL, DEFAULT NOW() | Event timestamp |

**Index**: `CREATE INDEX idx_audit_user_event ON auth_audit_log(userId, eventType, createdAt);`

## State Transitions

### User Account Lifecycle

```
[New Visitor] → signup → [Unverified]
                            │
              click verify link
                            │
                            ▼
                    [Verified, No Background]
                            │
              fill background OR skip
                            │
                            ▼
                    [Active User]
                            │
              delete account
                            │
                            ▼
                    [Soft-Deleted] ──(24h cooldown)──→ [Purgeable / Re-registrable]
```

### Session Lifecycle

```
[Login] → create session → [Active]
                              │
           ┌──────────────────┼──────────────────┐
           │                  │                   │
     idle timeout       user logout        password change
     (30m / 30d)              │                   │
           │                  │                   │
           ▼                  ▼                   ▼
       [Expired]         [Revoked]        [All Sessions Revoked]
```

### Account Lockout Flow

```
[Login Attempt] → check lockout → [Locked?]
                                     │ No
                                     ▼
                            validate credentials
                                     │
                        ┌────────────┼────────────┐
                        │ Success    │             │ Failure
                        ▼            │             ▼
                  reset counter      │      increment counter
                  create session     │             │
                                     │      counter >= 5?
                                     │      ┌──────┼──────┐
                                     │      │ No   │      │ Yes
                                     │      │      │      ▼
                                     │      │      │  set lockoutUntil
                                     │      │      │  = now + 15min
                                     │      └──────┘
```
