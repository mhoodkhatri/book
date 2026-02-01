# Quickstart: Better-Auth Authentication

**Feature**: 005-better-auth | **Date**: 2026-01-30

## Prerequisites

1. **Neon Postgres database** — Create a free-tier database at [neon.tech](https://neon.tech). Copy the connection string.
2. **Brevo account** — Sign up at [brevo.com](https://brevo.com). Get an API key from Settings → SMTP & API. Verify your sender email.
3. **Node.js 18+** — Required for the Better-Auth Express service.
4. **Python 3.10+** — Already required for the FastAPI backend.

## Environment Variables

Create/update `.env` files:

### `auth-service/.env`
```env
DATABASE_URL=postgresql://user:pass@ep-xxx.us-east-2.aws.neon.tech/neondb?sslmode=require
BETTER_AUTH_SECRET=<random-32-char-string>
BETTER_AUTH_URL=http://localhost:3005
BREVO_API_KEY=xkeysib-xxxx
BREVO_SENDER_EMAIL=your-verified@email.com
BREVO_SENDER_NAME=Robotics Textbook
FRONTEND_URL=http://localhost:3000
```

### `backend/.env` (add to existing)
```env
# Existing keys...
DATABASE_URL=postgresql://user:pass@ep-xxx.us-east-2.aws.neon.tech/neondb?sslmode=require
```

## Setup Steps

### 1. Initialize the Auth Service

```bash
# From repo root
mkdir auth-service && cd auth-service
npm init -y
npm install better-auth express cors dotenv pg
npm install -D typescript @types/express @types/cors @better-auth/cli tsx
```

### 2. Configure Better-Auth

Create `auth-service/src/auth.ts`:
```ts
import { betterAuth } from "better-auth";
import { Pool } from "pg";

export const auth = betterAuth({
  database: new Pool({
    connectionString: process.env.DATABASE_URL,
  }),
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: true,
    minPasswordLength: 8,
    maxPasswordLength: 128,
  },
  emailVerification: {
    sendOnSignUp: true,
    autoSignInAfterVerification: true,
    sendVerificationEmail: async ({ user, url }) => {
      await fetch("https://api.brevo.com/v3/smtp/email", {
        method: "POST",
        headers: {
          "api-key": process.env.BREVO_API_KEY!,
          "Content-Type": "application/json",
        },
        body: JSON.stringify({
          sender: {
            name: process.env.BREVO_SENDER_NAME,
            email: process.env.BREVO_SENDER_EMAIL,
          },
          to: [{ email: user.email }],
          subject: "Verify your email - Robotics Textbook",
          htmlContent: `<p>Click <a href="${url}">here</a> to verify your email.</p>`,
        }),
      });
    },
  },
  session: {
    expiresIn: 30 * 60, // 30 minutes
    updateAge: 5 * 60,  // refresh every 5 minutes
  },
  user: {
    additionalFields: {
      softwareBackground: { type: "string", required: false },
      hardwareBackground: { type: "string", required: false },
      softwareOther: { type: "string", required: false },
      hardwareOther: { type: "string", required: false },
      backgroundCompleted: { type: "boolean", required: false, defaultValue: false },
      failedLoginAttempts: { type: "number", required: false, defaultValue: 0 },
      lockoutUntil: { type: "string", required: false },
    },
  },
});
```

### 3. Create Express Server

Create `auth-service/src/index.ts`:
```ts
import express from "express";
import cors from "cors";
import { toNodeHandler } from "better-auth/node";
import { auth } from "./auth";
import "dotenv/config";

const app = express();

app.use(cors({
  origin: process.env.FRONTEND_URL || "http://localhost:3000",
  credentials: true,
}));

// Better-Auth handler (BEFORE express.json())
app.all("/api/auth/*", toNodeHandler(auth));

// Custom routes (AFTER Better-Auth handler)
app.use(express.json());

// Custom endpoints go here...

app.listen(3005, () => {
  console.log("Auth service running on http://localhost:3005");
});
```

### 4. Generate & Run Database Migrations

```bash
cd auth-service
npx @better-auth/cli generate  # Generates SQL schema
npx @better-auth/cli migrate   # Applies to Neon DB
```

### 5. Set Up React Client

Install in the Docusaurus project:
```bash
# From repo root
npm install better-auth
```

Create `src/lib/auth-client.ts`:
```ts
import { createAuthClient } from "better-auth/react";

export const authClient = createAuthClient({
  baseURL: "http://localhost:3005",
});
```

### 6. Add Auth Dependency to FastAPI

```bash
cd backend
pip install asyncpg  # For Neon Postgres session validation
```

### 7. Start Development

```bash
# Terminal 1: Auth service
cd auth-service && npx tsx src/index.ts

# Terminal 2: FastAPI backend
cd backend && uvicorn src.main:app --reload --port 8000

# Terminal 3: Docusaurus frontend
npm start
```

## Verification Checklist

- [ ] Neon database created and connection string works
- [ ] Brevo API key valid and sender email verified
- [ ] Auth service starts on port 3005 without errors
- [ ] Database tables created (user, session, account, verification)
- [ ] `POST /api/auth/sign-up/email` creates a user and sends email
- [ ] Verification email received and link works
- [ ] `POST /api/auth/sign-in/email` returns session cookie
- [ ] `GET /api/auth/get-session` returns user data with valid cookie
- [ ] FastAPI can query the session table to validate cookies
- [ ] CORS allows requests from `http://localhost:3000` with credentials

## Architecture Overview

```
┌─────────────────────────┐     ┌──────────────────────┐
│  Docusaurus Frontend    │     │  Better-Auth Service  │
│  (React, port 3000)     │────▶│  (Express, port 3005) │
│                         │     │                       │
│  - Auth pages           │     │  - /api/auth/* routes │
│  - AuthContext provider  │     │  - Email via Brevo    │
│  - Gated feature checks │     │  - Session management │
└────────┬────────────────┘     └───────────┬───────────┘
         │                                  │
         │  (chat, translate APIs)          │  (reads/writes)
         │                                  │
         ▼                                  ▼
┌─────────────────────────┐     ┌──────────────────────┐
│  FastAPI Backend         │     │  Neon Postgres        │
│  (Python, port 8000)    │────▶│  (Shared Database)    │
│                         │     │                       │
│  - RAG chatbot          │     │  - user table         │
│  - Translation          │     │  - session table      │
│  - Session validation   │     │  - account table      │
│    (reads session table)│     │  - verification table │
└─────────────────────────┘     │  - auth_audit_log     │
                                └──────────────────────┘
```
