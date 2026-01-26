# Data Model: Better-Auth Authentication

**Feature**: 004-better-auth | **Date**: 2026-01-26 | **Database**: Neon Serverless Postgres

## Entity Relationship Diagram

```
┌─────────────────┐       ┌─────────────────┐       ┌─────────────────┐
│      user       │       │   user_profile  │       │     session     │
├─────────────────┤       ├─────────────────┤       ├─────────────────┤
│ id (PK)         │──1:1──│ user_id (FK)    │       │ id (PK)         │
│ email           │       │ id (PK)         │       │ user_id (FK)    │──M:1──┐
│ email_verified  │       │ background_type │       │ token           │       │
│ name            │       │ created_at      │       │ expires_at      │       │
│ image           │       │ updated_at      │       │ ip_address      │       │
│ created_at      │       └─────────────────┘       │ user_agent      │       │
│ updated_at      │                                 │ created_at      │       │
└─────────────────┘                                 │ updated_at      │       │
        │                                           └─────────────────┘       │
        └─────────────────────────────────────────────────────────────────────┘
```

## Tables

### Table: user (Better-Auth managed)

Core user account table. Created and managed by Better-Auth.

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| id | TEXT | PRIMARY KEY | Unique user identifier (CUID) |
| email | TEXT | UNIQUE, NOT NULL | User email address |
| email_verified | BOOLEAN | DEFAULT FALSE | Email verification status |
| name | TEXT | NULL | Display name (optional) |
| image | TEXT | NULL | Profile image URL (optional) |
| created_at | TIMESTAMP | DEFAULT NOW() | Account creation time |
| updated_at | TIMESTAMP | DEFAULT NOW() | Last update time |

**Indexes**:
- `user_email_idx` on `email` (unique)

### Table: user_profile (Custom extension)

Stores user background information captured during signup. Custom table for our feature.

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| id | TEXT | PRIMARY KEY | Unique profile identifier (CUID) |
| user_id | TEXT | FOREIGN KEY, UNIQUE | References user.id |
| background_type | TEXT | NOT NULL | User's background category |
| created_at | TIMESTAMP | DEFAULT NOW() | Profile creation time |
| updated_at | TIMESTAMP | DEFAULT NOW() | Last update time |

**Foreign Key**: `user_id` REFERENCES `user(id)` ON DELETE CASCADE

**Indexes**:
- `user_profile_user_id_idx` on `user_id` (unique)

**background_type Enum Values**:
| Value | Display Label |
|-------|---------------|
| `beginner_robotics` | Beginner in robotics |
| `experienced_programmer` | Experienced programmer |
| `ai_ml_background` | AI/ML background |
| `hardware_electronics` | Hardware/electronics background |

### Table: session (Better-Auth managed)

Active user sessions. Created and managed by Better-Auth.

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| id | TEXT | PRIMARY KEY | Unique session identifier |
| user_id | TEXT | FOREIGN KEY | References user.id |
| token | TEXT | UNIQUE, NOT NULL | Session token (JWT or opaque) |
| expires_at | TIMESTAMP | NOT NULL | Session expiration time |
| ip_address | TEXT | NULL | Client IP address |
| user_agent | TEXT | NULL | Client user agent |
| created_at | TIMESTAMP | DEFAULT NOW() | Session creation time |
| updated_at | TIMESTAMP | DEFAULT NOW() | Last activity time |

**Foreign Key**: `user_id` REFERENCES `user(id)` ON DELETE CASCADE

**Indexes**:
- `session_token_idx` on `token` (unique)
- `session_user_id_idx` on `user_id`
- `session_expires_at_idx` on `expires_at`

## SQL DDL

```sql
-- Better-Auth core tables (auto-created by Better-Auth, shown for reference)
CREATE TABLE IF NOT EXISTS "user" (
    id TEXT PRIMARY KEY,
    email TEXT UNIQUE NOT NULL,
    email_verified BOOLEAN DEFAULT FALSE,
    name TEXT,
    image TEXT,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

CREATE TABLE IF NOT EXISTS session (
    id TEXT PRIMARY KEY,
    user_id TEXT NOT NULL REFERENCES "user"(id) ON DELETE CASCADE,
    token TEXT UNIQUE NOT NULL,
    expires_at TIMESTAMP WITH TIME ZONE NOT NULL,
    ip_address TEXT,
    user_agent TEXT,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

CREATE INDEX IF NOT EXISTS session_user_id_idx ON session(user_id);
CREATE INDEX IF NOT EXISTS session_expires_at_idx ON session(expires_at);

-- Custom: User profile table (must be created manually)
CREATE TABLE IF NOT EXISTS user_profile (
    id TEXT PRIMARY KEY,
    user_id TEXT UNIQUE NOT NULL REFERENCES "user"(id) ON DELETE CASCADE,
    background_type TEXT NOT NULL CHECK (
        background_type IN (
            'beginner_robotics',
            'experienced_programmer',
            'ai_ml_background',
            'hardware_electronics'
        )
    ),
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

CREATE INDEX IF NOT EXISTS user_profile_user_id_idx ON user_profile(user_id);
```

## Pydantic Models (Python)

```python
# backend/src/models/auth.py
from datetime import datetime
from enum import Enum
from pydantic import BaseModel, EmailStr

class BackgroundType(str, Enum):
    BEGINNER_ROBOTICS = "beginner_robotics"
    EXPERIENCED_PROGRAMMER = "experienced_programmer"
    AI_ML_BACKGROUND = "ai_ml_background"
    HARDWARE_ELECTRONICS = "hardware_electronics"

class User(BaseModel):
    id: str
    email: EmailStr
    email_verified: bool = False
    name: str | None = None
    image: str | None = None
    created_at: datetime
    updated_at: datetime

class UserProfile(BaseModel):
    id: str
    user_id: str
    background_type: BackgroundType
    created_at: datetime
    updated_at: datetime

class Session(BaseModel):
    id: str
    user_id: str
    token: str
    expires_at: datetime
    ip_address: str | None = None
    user_agent: str | None = None
    created_at: datetime
    updated_at: datetime

# Request/Response models
class SignUpRequest(BaseModel):
    email: EmailStr
    password: str
    name: str | None = None
    background_type: BackgroundType

class SignInRequest(BaseModel):
    email: EmailStr
    password: str

class AuthResponse(BaseModel):
    user: User
    token: str
    expires_at: datetime

class TokenPayload(BaseModel):
    """JWT payload structure."""
    sub: str  # user_id
    email: str
    exp: int  # expiration timestamp
    iat: int  # issued at timestamp
```

## TypeScript Types (Frontend)

```typescript
// src/types/auth.ts
export type BackgroundType =
  | 'beginner_robotics'
  | 'experienced_programmer'
  | 'ai_ml_background'
  | 'hardware_electronics';

export interface User {
  id: string;
  email: string;
  emailVerified: boolean;
  name?: string;
  image?: string;
  createdAt: Date;
  updatedAt: Date;
}

export interface UserProfile {
  id: string;
  userId: string;
  backgroundType: BackgroundType;
  createdAt: Date;
  updatedAt: Date;
}

export interface Session {
  user: User;
  token: string;
  expiresAt: Date;
}

// Form data types
export interface SignUpData {
  email: string;
  password: string;
  name?: string;
  backgroundType: BackgroundType;
}

export interface SignInData {
  email: string;
  password: string;
}
```

## Data Validation Rules

### User Table
- `email`: Valid email format, max 255 characters, unique
- `name`: Max 100 characters, optional
- `image`: Valid URL format, optional

### User Profile Table
- `background_type`: Must be one of the enum values
- `user_id`: Must reference existing user

### Session Table
- `token`: Non-empty, unique
- `expires_at`: Must be in the future when created
- `ip_address`: Valid IPv4 or IPv6 format

## State Transitions

### User Lifecycle
```
[Created] -> [Email Verified] -> [Active] -> [Deleted]
                                    |
                                    v
                            [Session Created]
                                    |
                                    v
                            [Session Expired/Revoked]
```

### Session Lifecycle
```
[Created] -> [Active] -> [Expired] -> [Deleted]
                |
                v
           [Revoked (Sign Out)]
```

## Migration Notes

1. **Initial Setup**: Run Better-Auth setup to auto-create `user` and `session` tables
2. **Custom Migration**: Manually run `user_profile` table creation SQL
3. **Seeding**: No seed data required for production
4. **Rollback**: DROP TABLE user_profile (custom table only)
