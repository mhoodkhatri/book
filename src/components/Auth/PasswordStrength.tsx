import React from "react";

interface Props {
  password: string;
}

interface StrengthResult {
  score: number;
  label: string;
  color: string;
}

function evaluateStrength(password: string): StrengthResult {
  let score = 0;

  if (password.length >= 8) score++;
  if (password.length >= 12) score++;
  if (/[a-z]/.test(password)) score++;
  if (/[A-Z]/.test(password)) score++;
  if (/[0-9]/.test(password)) score++;
  if (/[^a-zA-Z0-9]/.test(password)) score++;

  if (score <= 2) return { score, label: "Weak", color: "#e74c3c" };
  if (score <= 3) return { score, label: "Fair", color: "#f39c12" };
  if (score <= 4) return { score, label: "Strong", color: "#27ae60" };
  return { score, label: "Very Strong", color: "#2e8555" };
}

export default function PasswordStrength({ password }: Props): React.JSX.Element | null {
  if (!password) return null;

  const { score, label, color } = evaluateStrength(password);
  const percent = Math.min((score / 6) * 100, 100);

  return (
    <div className="auth-password-strength" aria-live="polite">
      <div className="auth-password-strength__bar">
        <div
          className="auth-password-strength__fill"
          style={{ width: `${percent}%`, backgroundColor: color }}
        />
      </div>
      <span className="auth-password-strength__label" style={{ color }}>
        {label}
      </span>
    </div>
  );
}
