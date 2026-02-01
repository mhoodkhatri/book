/**
 * Brevo (Sendinblue) email service for verification and password reset emails.
 * Uses Brevo REST API with retry logic (3 attempts, exponential backoff).
 */

const BREVO_API_URL = "https://api.brevo.com/v3/smtp/email";
const MAX_RETRIES = 3;

function escapeHtml(str: string): string {
  return str
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;")
    .replace(/"/g, "&quot;")
    .replace(/'/g, "&#039;");
}

interface BrevoEmailPayload {
  sender: { name: string; email: string };
  to: { email: string; name?: string }[];
  subject: string;
  htmlContent: string;
}

async function sendEmail(
  to: string,
  subject: string,
  htmlContent: string
): Promise<void> {
  const apiKey = process.env.BREVO_API_KEY;
  if (!apiKey) {
    throw new Error("BREVO_API_KEY environment variable is not set");
  }

  const payload: BrevoEmailPayload = {
    sender: {
      name: process.env.BREVO_SENDER_NAME || "Robotics Textbook",
      email: process.env.BREVO_SENDER_EMAIL || "",
    },
    to: [{ email: to }],
    subject,
    htmlContent,
  };

  let lastError: Error | null = null;

  for (let attempt = 1; attempt <= MAX_RETRIES; attempt++) {
    try {
      const response = await fetch(BREVO_API_URL, {
        method: "POST",
        headers: {
          "api-key": apiKey,
          "Content-Type": "application/json",
        },
        body: JSON.stringify(payload),
      });

      if (response.ok) {
        return;
      }

      const errorBody = await response.text();
      lastError = new Error(
        `Brevo API error (HTTP ${response.status}): ${errorBody}`
      );

      // Don't retry on client errors (4xx) except 429 (rate limit)
      if (response.status >= 400 && response.status < 500 && response.status !== 429) {
        throw lastError;
      }
    } catch (err) {
      if (err instanceof Error && err.message.startsWith("Brevo API error")) {
        lastError = err;
        // Re-throw non-retryable errors
        if (!err.message.includes("429")) {
          throw err;
        }
      } else {
        lastError = err instanceof Error ? err : new Error(String(err));
      }
    }

    // Exponential backoff: 1s, 2s, 4s
    if (attempt < MAX_RETRIES) {
      await new Promise((resolve) =>
        setTimeout(resolve, Math.pow(2, attempt - 1) * 1000)
      );
    }
  }

  throw lastError || new Error("Failed to send email after retries");
}

export async function sendVerificationEmail(
  user: { email?: string; name?: string },
  url: string
): Promise<void> {
  if (!user.email) {
    throw new Error("Cannot send verification email: user has no email");
  }
  const subject = "Verify your email - Robotics Textbook";
  const htmlContent = `
    <div style="font-family: system-ui, sans-serif; max-width: 600px; margin: 0 auto; padding: 20px;">
      <h2>Welcome to Physical AI & Humanoid Robotics Textbook!</h2>
      <p>Hi ${escapeHtml(user.name || "there")},</p>
      <p>Please verify your email address by clicking the button below:</p>
      <div style="text-align: center; margin: 30px 0;">
        <a href="${url}" style="background-color: #2e8555; color: white; padding: 12px 24px; text-decoration: none; border-radius: 6px; font-weight: bold;">
          Verify Email Address
        </a>
      </div>
      <p style="color: #666; font-size: 14px;">
        If you didn't create an account, you can safely ignore this email.
      </p>
      <p style="color: #666; font-size: 14px;">
        This link expires in 24 hours.
      </p>
    </div>
  `;

  await sendEmail(user.email, subject, htmlContent);
}

export async function sendResetPasswordEmail(
  user: { email?: string; name?: string },
  url: string
): Promise<void> {
  if (!user.email) {
    throw new Error("Cannot send reset email: user has no email");
  }
  const subject = "Reset your password - Robotics Textbook";
  const htmlContent = `
    <div style="font-family: system-ui, sans-serif; max-width: 600px; margin: 0 auto; padding: 20px;">
      <h2>Password Reset Request</h2>
      <p>Hi ${escapeHtml(user.name || "there")},</p>
      <p>You requested a password reset. Click the button below to set a new password:</p>
      <div style="text-align: center; margin: 30px 0;">
        <a href="${url}" style="background-color: #2e8555; color: white; padding: 12px 24px; text-decoration: none; border-radius: 6px; font-weight: bold;">
          Reset Password
        </a>
      </div>
      <p style="color: #666; font-size: 14px;">
        If you didn't request this, you can safely ignore this email. Your password won't change.
      </p>
      <p style="color: #666; font-size: 14px;">
        This link expires in 1 hour.
      </p>
    </div>
  `;

  await sendEmail(user.email, subject, htmlContent);
}
