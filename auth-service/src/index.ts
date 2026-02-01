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
app.all("/api/auth/*", toNodeHandler(auth));

// Health check
app.get("/health", (_req, res) => {
  res.json({ status: "healthy", service: "auth-service" });
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
