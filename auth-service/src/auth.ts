import { betterAuth } from "better-auth";
import { createAuthMiddleware, APIError } from "better-auth/api";
import { Pool } from "pg";
import { sendVerificationEmail, sendResetPasswordEmail } from "./services/brevo.js";
import { logAuthEvent } from "./lib/audit.js";
import { checkLockout, recordFailedLogin } from "./middleware/lockout.js";

const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
});

export { pool };

export const auth = betterAuth({
  database: pool,
  baseURL: process.env.BETTER_AUTH_URL || "http://localhost:3005",
  secret: process.env.BETTER_AUTH_SECRET,
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: true,
    minPasswordLength: 8,
    maxPasswordLength: 128,
    sendResetPassword: async ({ user, url }) => {
      await sendResetPasswordEmail(user, url);
    },
  },
  emailVerification: {
    sendOnSignUp: true,
    autoSignInAfterVerification: true,
    sendVerificationEmail: async ({ user, url }) => {
      await sendVerificationEmail(user, url);
    },
  },
  session: {
    expiresIn: 30 * 60,
    updateAge: 5 * 60,
  },
  user: {
    additionalFields: {
      softwareBackground: {
        type: "string",
        required: false,
        input: true,
      },
      hardwareBackground: {
        type: "string",
        required: false,
        input: true,
      },
      softwareOther: {
        type: "string",
        required: false,
        input: true,
      },
      hardwareOther: {
        type: "string",
        required: false,
        input: true,
      },
      backgroundCompleted: {
        type: "boolean",
        required: false,
        defaultValue: false,
        input: true,
      },
      failedLoginAttempts: {
        type: "number",
        required: false,
        defaultValue: 0,
        input: false,
      },
      lockoutUntil: {
        type: "string",
        required: false,
        input: false,
      },
    },
  },
  hooks: {
    before: createAuthMiddleware(async (ctx) => {
      // Email normalization for sign-in and sign-up
      if (
        ctx.path === "/sign-in/email" ||
        ctx.path === "/sign-up/email"
      ) {
        const body = ctx.body as Record<string, unknown> | undefined;
        if (body?.email && typeof body.email === "string") {
          body.email = body.email.toLowerCase().trim();
        }
      }

      // Account lockout check before sign-in
      if (ctx.path === "/sign-in/email") {
        const body = ctx.body as Record<string, unknown> | undefined;
        const email = body?.email;
        if (email && typeof email === "string") {
          const lockoutResult = await checkLockout(pool, email);
          if (lockoutResult.locked) {
            const remainingMin = Math.ceil((lockoutResult.remainingMs || 0) / 60000);
            throw new APIError("FORBIDDEN", {
              message: `Account temporarily locked. Try again in ${remainingMin} minute(s).`,
            });
          }
        }
      }
    }),
    after: createAuthMiddleware(async (ctx) => {
      const ip = ctx.headers?.get?.("x-forwarded-for") || null;
      const ua = ctx.headers?.get?.("user-agent") || null;

      if (ctx.path === "/sign-up/email") {
        const userId = (ctx.context as any)?.newSession?.user?.id;
        if (userId) {
          await logAuthEvent(pool, {
            userId,
            eventType: "signup",
            ipAddress: ip,
            userAgent: ua,
            success: true,
          });
        }
      }

      if (ctx.path === "/sign-in/email") {
        const userId = (ctx.context as any)?.newSession?.user?.id;
        if (userId) {
          // Successful login — reset lockout counter
          await pool.query(
            `UPDATE "user" SET "failedLoginAttempts" = 0, "lockoutUntil" = NULL WHERE id = $1`,
            [userId]
          );
          await logAuthEvent(pool, {
            userId,
            eventType: "login",
            ipAddress: ip,
            userAgent: ua,
            success: true,
          });
        } else {
          // Failed login — record attempt for lockout
          const body = ctx.body as Record<string, unknown> | undefined;
          const email = body?.email;
          if (email && typeof email === "string") {
            await recordFailedLogin(pool, email, ip, ua);
          }
        }
      }

      if (ctx.path === "/sign-out") {
        await logAuthEvent(pool, {
          userId: null,
          eventType: "logout",
          ipAddress: ip,
          userAgent: ua,
          success: true,
        });
      }
    }),
  },
  trustedOrigins: [process.env.FRONTEND_URL || "http://localhost:3000"],
});
