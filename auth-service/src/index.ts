import "dotenv/config";
import express from "express";
import cors from "cors";
import { toNodeHandler } from "better-auth/node";
import { auth, pool } from "./auth.js";
import { backgroundHandler } from "./routes/background.js";
import { deleteAccountHandler } from "./routes/account.js";

// Validate required env vars at startup
const requiredEnvVars = ["DATABASE_URL", "BETTER_AUTH_SECRET"];
for (const envVar of requiredEnvVars) {
  if (!process.env[envVar]) {
    console.error(`FATAL: ${envVar} environment variable is required`);
    process.exit(1);
  }
}

const app = express();
const PORT = parseInt(process.env.PORT || "3005", 10);

// CORS configuration — restrict to exact frontend URL
app.use(
  cors({
    origin: process.env.FRONTEND_URL || "http://localhost:3000",
    credentials: true,
    methods: ["GET", "POST", "PUT", "DELETE", "OPTIONS"],
    allowedHeaders: ["Content-Type", "Authorization"],
  })
);

// Custom routes — registered BEFORE Better-Auth catch-all so they aren't swallowed
// These need express.json() applied inline since Better-Auth must not see it globally
app.post("/api/auth/custom/update-background", express.json(), backgroundHandler);
app.post("/api/auth/custom/delete-account", express.json(), deleteAccountHandler);

// Better-Auth handler — MUST be BEFORE express.json()
// Better-Auth parses its own request bodies
const authHandler = toNodeHandler(auth);
app.all("/api/auth/*", (req, res) => {
  // Debug: log incoming request details
  const reqChunks: Buffer[] = [];
  req.on("data", (chunk: Buffer) => reqChunks.push(chunk));
  req.on("end", () => {
    const body = Buffer.concat(reqChunks).toString("utf8");
    lastAuthRequest = `${req.method} ${req.url} content-type=${req.headers["content-type"]} body-length=${body.length} body=${body.slice(0, 300)}`;
    console.log(`[auth-req]`, lastAuthRequest);
  });
  // Capture written chunks
  const chunks: Buffer[] = [];
  const origWrite = res.write.bind(res);
  (res as any).write = function (chunk: any, ...rest: any[]) {
    if (chunk) chunks.push(Buffer.isBuffer(chunk) ? chunk : Buffer.from(String(chunk)));
    return (origWrite as any)(chunk, ...rest);
  };
  // Intercept res.end to log what Better-Auth sends
  const origEnd = res.end.bind(res);
  (res as any).end = function (...args: any[]) {
    if (args[0]) chunks.push(Buffer.isBuffer(args[0]) ? args[0] : Buffer.from(String(args[0])));
    if (res.statusCode >= 400) {
      const bodyStr = Buffer.concat(chunks).toString("utf8").slice(0, 1000) || "(empty)";
      lastAuthError = `${req.method} ${req.url} -> ${res.statusCode}: ${bodyStr}`;
      console.error("[auth-response]", lastAuthError);
      authErrors.push(`${new Date().toISOString()} ${lastAuthError}`);
      if (authErrors.length > 10) authErrors.shift();
    }
    return origEnd(...args);
  };
  Promise.resolve(authHandler(req, res)).catch((err: unknown) => {
    const errDetail = err instanceof Error ? `${err.message}\n${err.stack}` : String(err);
    console.error("[auth-error]", req.method, req.url, errDetail);
    lastAuthError = `THROWN: ${req.method} ${req.url} -> ${errDetail}`;
    authErrors.push(`${new Date().toISOString()} ${lastAuthError}`);
    if (authErrors.length > 10) authErrors.shift();
    if (!res.headersSent) {
      res.status(500).json({ error: "Internal auth error", detail: errDetail });
    }
  });
});

// Health check
app.get("/health", (_req, res) => {
  res.json({ status: "healthy", service: "auth-service" });
});

// Store last auth errors for debugging (keep up to 5)
let lastAuthError: string = "none";
const authErrors: string[] = [];
let lastAuthRequest: string = "none";

// Capture unhandled rejections
process.on("unhandledRejection", (reason) => {
  const msg = `[unhandledRejection] ${String(reason)}`;
  console.error(msg);
  authErrors.push(`${new Date().toISOString()} ${msg}`);
  if (authErrors.length > 10) authErrors.shift();
});

// Debug: expose last auth error
app.get("/debug/last-error", (_req, res) => {
  res.json({ lastAuthError, lastAuthRequest, recentErrors: authErrors });
});
app.get("/debug/env", (_req, res) => {
  res.json({
    DATABASE_URL: process.env.DATABASE_URL ? "set" : "missing",
    BETTER_AUTH_SECRET: process.env.BETTER_AUTH_SECRET ? "set" : "missing",
    BETTER_AUTH_URL: process.env.BETTER_AUTH_URL || "missing",
    BREVO_API_KEY: process.env.BREVO_API_KEY ? "set" : "missing",
    BREVO_SENDER_EMAIL: process.env.BREVO_SENDER_EMAIL || "missing",
    FRONTEND_URL: process.env.FRONTEND_URL || "missing",
    NODE_ENV: process.env.NODE_ENV || "unset",
    NODE_VERSION: process.version,
  });
});
// Debug: test sign-up via Better-Auth internal API
app.get("/debug/test-signup", async (_req, res) => {
  try {
    const result = await auth.api.signUpEmail({
      body: {
        email: "internal-test@example.com",
        password: "TestPass123!",
        name: "Internal Test",
      },
    });
    res.json({ ok: true, result: JSON.stringify(result).slice(0, 500) });
  } catch (err: any) {
    res.json({
      ok: false,
      error: err?.message || String(err),
      stack: err?.stack?.slice(0, 1000),
      code: err?.code,
      status: err?.status,
      body: err?.body ? JSON.stringify(err.body) : undefined,
    });
  }
});
// Debug: test DB connectivity
app.get("/debug/db-test", async (_req, res) => {
  try {
    const result = await pool.query("SELECT COUNT(*) as count FROM \"user\"");
    res.json({ ok: true, userCount: result.rows[0].count });
  } catch (err) {
    res.json({ ok: false, error: String(err) });
  }
});

app.listen(PORT, () => {
  console.log(`Auth service running on http://localhost:${PORT}`);
});

// Graceful shutdown
process.on("SIGTERM", async () => {
  console.log("Shutting down auth service...");
  await pool.end();
  process.exit(0);
});
