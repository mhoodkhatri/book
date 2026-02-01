# Feature Specification: Better-Auth Authentication with Email Verification & Gated Access

**Feature Branch**: `005-better-auth`
**Created**: 2026-01-30
**Status**: Draft
**Input**: User description: "Implement Better-Auth authentication with email verification, user background capture, and gated access for RAG, Translation, and Personalization features"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - New User Signup with Email Verification (Priority: P1)

A new visitor arrives at the textbook site and wants to use the RAG chatbot or Translation features. They click "Sign Up," fill in their email and password, and receive a verification email. They click the verification link, complete their profile by providing their software and hardware background, and gain full access to all features.

**Why this priority**: Without signup and verification, no other auth-dependent feature works. This is the foundation of the entire authentication system and directly satisfies the hackathon bonus requirements.

**Independent Test**: Can be fully tested by creating a new account, receiving a verification email, clicking the link, and confirming the account is marked as verified. Delivers the core auth value independently.

**Acceptance Scenarios**:

1. **Given** a visitor is on the signup page, **When** they enter a valid email and a password meeting strength requirements, **Then** the system creates an unverified account and sends a verification email to the provided address.
2. **Given** a user has received a verification email, **When** they click the verification link within 24 hours, **Then** their account is marked as verified and they are redirected to the background capture form.
3. **Given** a user clicks an expired verification link (older than 24 hours), **When** the page loads, **Then** the system displays "This link has expired" with a "Resend verification" button.
4. **Given** a visitor enters an email that is already registered, **When** they submit the signup form, **Then** the system displays "An account with this email already exists. Sign in instead?" with a link to the login page.
5. **Given** a visitor enters a weak password, **When** they submit the signup form, **Then** the system displays inline validation errors with password strength guidance.

---

### User Story 2 - User Sign-In and Session Management (Priority: P1)

A returning user visits the site and signs in with their email and password. Their session persists across page navigations within the Docusaurus site. They can optionally choose "Remember me" to extend session duration. They can log out, which invalidates their session.

**Why this priority**: Sign-in is equally critical as signup — users must be able to return and access gated features. Session management ensures a seamless experience across the static site.

**Independent Test**: Can be tested by signing in with valid credentials, navigating between pages to confirm session persistence, and logging out to confirm session invalidation.

**Acceptance Scenarios**:

1. **Given** a verified user is on the login page, **When** they enter correct email and password, **Then** the system authenticates them and redirects to the page they were previously on (or home).
2. **Given** a user enters incorrect credentials, **When** they submit the login form, **Then** the system displays "Invalid email or password" (generic message, no field-specific hints).
3. **Given** a user has failed login 5 times consecutively, **When** they attempt a 6th login, **Then** the system temporarily locks the account for 15 minutes and displays a lockout message.
4. **Given** a logged-in user checks "Remember me," **When** they close and reopen the browser, **Then** their session persists for up to 30 days.
5. **Given** a logged-in user without "Remember me," **When** they are idle for 30 minutes, **Then** the session expires and they are prompted to log in again.
6. **Given** a logged-in user clicks "Log out," **When** the action completes, **Then** the session is invalidated and they are redirected to the home page.
7. **Given** a user is logged in on multiple tabs, **When** they log out in one tab, **Then** all other tabs detect the logout and update their auth state.

---

### User Story 3 - User Background Capture at Signup (Priority: P2)

After verifying their email, the user is presented with a background capture form that asks about their software experience (e.g., Python, ROS 2, C++) and hardware setup (e.g., Jetson Orin, desktop workstation). This data is stored in their profile and used by the Personalization feature.

**Why this priority**: Required by the constitution for the Personalization bonus. Must happen during signup flow but does not block basic auth functionality.

**Independent Test**: Can be tested by completing signup, being redirected to the background form, filling it out, and confirming the data persists in the user's profile.

**Acceptance Scenarios**:

1. **Given** a user has just verified their email, **When** they are redirected after verification, **Then** they see the background capture form.
2. **Given** a user is on the background form, **When** they select their software skills and hardware setup, **Then** the data is saved to their profile.
3. **Given** a user wants to skip the background form, **When** they click "Skip for now," **Then** they are taken to the home page and can complete their profile later from account settings.
4. **Given** a user has already filled in their background, **When** they visit account settings, **Then** they can update their software and hardware background.

---

### User Story 4 - Gated Access to RAG, Translation, and Personalization (Priority: P2)

Unauthenticated users can read the textbook freely but cannot access the RAG chatbot, Translation button, or Personalization button. When they attempt to use any gated feature, they are prompted to sign in or sign up. After authenticating, they are returned to where they left off.

**Why this priority**: This is the access policy decision — RAG, Translation, and Personalization all require login. Gating these features protects API costs and ties usage to verified accounts.

**Independent Test**: Can be tested by visiting a chapter page without logging in, attempting to use the RAG chatbot, being prompted to log in, authenticating, and then being returned to the same chapter with the chatbot now accessible.

**Acceptance Scenarios**:

1. **Given** an unauthenticated user is on a chapter page, **When** they try to open the RAG chatbot, **Then** a login prompt appears instead of the chatbot interface.
2. **Given** an unauthenticated user is on a chapter page, **When** they click the "Urdu" translation button, **Then** a login prompt appears instead of triggering translation.
3. **Given** an unauthenticated user is on a chapter page, **When** they click the "Personalize" button, **Then** a login prompt appears instead of triggering personalization.
4. **Given** a user logs in after being prompted by a gated feature, **When** authentication succeeds, **Then** they are redirected back to the same chapter page with the feature now accessible.
5. **Given** an unauthenticated user, **When** they browse chapter content (reading text, viewing diagrams), **Then** the content loads normally without any auth requirement.

---

### User Story 5 - Password Reset Flow (Priority: P3)

A user who has forgotten their password can request a reset. They receive an email with a reset link, click it, set a new password, and all existing sessions are invalidated.

**Why this priority**: Important for usability but not a core hackathon deliverable. Users who forget passwords would be locked out entirely without this.

**Independent Test**: Can be tested by requesting a password reset, receiving the email, clicking the link, setting a new password, and confirming old sessions are invalidated.

**Acceptance Scenarios**:

1. **Given** a user is on the login page, **When** they click "Forgot password?" and enter their email, **Then** the system sends a password reset email (regardless of whether the email exists — no account enumeration).
2. **Given** a user has a reset email, **When** they click the reset link within 1 hour, **Then** they see a form to set a new password.
3. **Given** a user clicks an expired reset link (older than 1 hour), **When** the page loads, **Then** the system displays "This link has expired" with an option to request a new one.
4. **Given** a user sets a new password via reset, **When** the password is saved, **Then** all existing sessions for that account are invalidated.
5. **Given** a reset token has been used once, **When** someone attempts to use it again, **Then** the system rejects it as invalid.

---

### User Story 6 - Account Deletion (Priority: P3)

A user can delete their account from account settings. They must confirm by re-entering their password. Deletion anonymizes their data and invalidates all sessions.

**Why this priority**: Privacy compliance requirement from the constitution. Lower priority since it's rarely exercised but legally and ethically necessary.

**Independent Test**: Can be tested by navigating to account settings, initiating deletion, confirming with password, and verifying the account is no longer accessible.

**Acceptance Scenarios**:

1. **Given** a logged-in user is in account settings, **When** they click "Delete Account," **Then** a confirmation dialog appears requiring password re-entry.
2. **Given** a user confirms deletion with correct password, **When** the action completes, **Then** the account is soft-deleted, data is anonymized, all sessions are invalidated, and the user is redirected to the home page.
3. **Given** an account has been deleted, **When** the same email is used to sign up again after 24 hours, **Then** a new account is created successfully.

---

### Edge Cases

- What happens when a user signs up with mixed-case email (e.g., `User@Email.COM`)? System normalizes to lowercase before storage and comparison.
- What happens when the email delivery service is down? System queues the email for retry (3 attempts with exponential backoff) and shows "Verification email sent. If you don't receive it, click Resend."
- What happens when a user clicks "Resend verification" rapidly? A 60-second cooldown is enforced with a visible countdown timer.
- What happens when a user's session expires while they are composing a RAG query? The query input is preserved in local state, a login prompt appears, and after re-authentication the query is still present.
- What happens when the auth service is unavailable? Textbook content remains fully readable. Gated features show "Service temporarily unavailable" instead of a login prompt.
- What happens when a user tries to sign in with an unverified account? System displays "Please verify your email first" with a "Resend verification" option.
- What happens when multiple tabs are open and the user logs in on one? All tabs detect the login via storage event and update their auth state.

## Requirements *(mandatory)*

### Functional Requirements

#### Authentication Core
- **FR-001**: System MUST provide email/password signup with inline field validation (email format, password strength).
- **FR-002**: System MUST send a verification email to every new user upon signup, deliverable to any email address (not restricted to pre-registered recipients).
- **FR-003**: System MUST verify user accounts only after they click the verification link. Unverified accounts cannot access gated features.
- **FR-004**: System MUST provide email/password sign-in with "Remember me" option.
- **FR-005**: System MUST display generic error messages on failed login ("Invalid email or password") to prevent account enumeration.
- **FR-006**: System MUST lock accounts temporarily (15 minutes) after 5 consecutive failed login attempts.
- **FR-007**: System MUST normalize email addresses to lowercase before storage and comparison.

#### Session Management
- **FR-008**: System MUST maintain user sessions across page navigations within the Docusaurus site.
- **FR-009**: System MUST expire idle sessions after 30 minutes (without "Remember me") or 30 days (with "Remember me").
- **FR-010**: System MUST rotate session identifiers upon successful login to prevent session fixation.
- **FR-011**: System MUST synchronize auth state across browser tabs (login/logout in one tab reflects in all tabs).
- **FR-012**: System MUST invalidate all sessions when a user changes their password or deletes their account.

#### Email Service
- **FR-013**: System MUST use Brevo as the transactional email service, configured with a verified personal sender email, capable of sending to any recipient.
- **FR-014**: System MUST retry failed email deliveries up to 3 times with exponential backoff.
- **FR-015**: System MUST enforce a 60-second cooldown on verification email resend requests with a visible countdown.
- **FR-016**: Verification links MUST expire after 24 hours. Password reset links MUST expire after 1 hour.
- **FR-017**: All email tokens MUST be single-use — once clicked, the token is invalidated.

#### User Profile & Background
- **FR-018**: System MUST present a background capture form after email verification, collecting software experience and hardware setup via multi-select checkboxes from predefined lists. Software options: Python, ROS 2, C++, JavaScript, MATLAB, Bash/Shell, and "Other" (free-text). Hardware options: Jetson Orin, Desktop Workstation, Laptop, Raspberry Pi, Cloud/VM, and "Other" (free-text).
- **FR-019**: System MUST allow users to skip background capture and complete it later from account settings.
- **FR-020**: System MUST allow users to update their background information from account settings (`/profile/settings`) at any time.

#### Gated Access
- **FR-021**: System MUST gate RAG chatbot access behind authentication — unauthenticated users see a login prompt.
- **FR-022**: System MUST gate Translation feature access behind authentication — unauthenticated users see a login prompt.
- **FR-023**: System MUST gate Personalization feature access behind authentication — unauthenticated users see a login prompt.
- **FR-024**: System MUST redirect users back to their original page after successful authentication from a gated feature prompt.
- **FR-025**: Textbook content (reading chapters, viewing diagrams) MUST remain accessible without authentication.

#### Password Management
- **FR-026**: System MUST provide a "Forgot password?" flow that sends a reset email without confirming whether the email exists (no account enumeration).
- **FR-027**: System MUST allow users to set a new password via a time-limited, single-use reset link.
- **FR-028**: Password reset MUST invalidate all existing sessions for the account.

#### Account Management
- **FR-029**: System MUST allow users to delete their account with password re-confirmation.
- **FR-030**: Account deletion MUST soft-delete and anonymize user data, invalidate all sessions.
- **FR-031**: System MUST allow re-registration with the same email after a 24-hour cooling period following deletion.

#### Security
- **FR-032**: System MUST use HTTP-only, Secure, SameSite=None cookies for session tokens (required for cross-origin auth between Docusaurus site and Better-Auth service on separate origins).
- **FR-033**: System MUST never commit API keys, secrets, or database credentials — all secrets via environment variables.
- **FR-034**: System MUST implement CORS on the Better-Auth service (separate origin/port) to only accept requests from the Docusaurus site domain. Cookies must use `SameSite=None; Secure` for cross-origin auth.
- **FR-035**: System MUST log authentication events (signup, login, logout, password reset, account deletion) with timestamps.

#### UI/UX
- **FR-036**: System MUST provide a combined sign-in/sign-up page at `/auth` (via `src/pages/`) with tab toggle between the two forms. Account settings and background capture live at `/profile/settings` and `/profile/background` respectively.
- **FR-037**: System MUST show a password visibility toggle (show/hide) on password fields.
- **FR-038**: System MUST show a real-time password strength meter during signup.
- **FR-039**: System MUST use proper `autocomplete` attributes on form fields for browser/password manager support.
- **FR-040**: System MUST disable the submit button while a request is in-flight to prevent double submission.
- **FR-041**: System MUST show toast notifications for key actions (signed in, signed out, email sent, account deleted).
- **FR-042**: System MUST show loading/skeleton states during auth checks to prevent flash of incorrect auth state.
- **FR-043**: System MUST gracefully degrade when the auth service is unavailable — textbook content remains readable, gated features show "Service temporarily unavailable."

### Key Entities

- **User**: Represents a registered user. Key attributes: email (unique, normalized), password (hashed), verification status, account status (active/deleted), creation date.
- **User Profile/Background**: Represents the user's software experience and hardware setup, stored as arrays of selected checkbox values plus optional free-text "Other" entries. Software options: Python, ROS 2, C++, JavaScript, MATLAB, Bash/Shell. Hardware options: Jetson Orin, Desktop Workstation, Laptop, Raspberry Pi, Cloud/VM. Linked one-to-one with User.
- **Session**: Represents an active authenticated session. Key attributes: session token, user reference, creation time, expiry time, remember-me flag.
- **Verification Token**: Single-use token for email verification. Attributes: token value, user reference, creation time, expiry (24 hours), used status.
- **Password Reset Token**: Single-use token for password reset. Attributes: token value, user reference, creation time, expiry (1 hour), used status.
- **Auth Audit Log**: Record of authentication events. Attributes: event type, user reference, timestamp, IP address, success/failure.

## Clarifications

### Session 2026-01-30

- Q: How should Better-Auth integrate with the existing FastAPI backend? → A: Better-Auth runs as a standalone Node.js/Express API; FastAPI validates sessions by querying the shared Neon Postgres database directly (decoupled runtimes, no proxy layer).
- Q: Where should auth UI pages live within the Docusaurus site? → A: Dedicated Docusaurus pages via `src/pages/` (e.g., `/auth`, `/profile/settings`) — full-page auth UI with proper routes.
- Q: Which transactional email provider should be used? → A: Brevo (formerly Sendinblue) — 300 emails/day free tier, personal sender email verification, no custom domain required.
- Q: What is the deployment topology for the Better-Auth service relative to the Docusaurus site? → A: Separate origin (different port or subdomain), CORS configured for the Docusaurus domain. Two independent processes with clear separation.
- Q: What input format should the background capture form use? → A: Multi-select checkboxes from predefined lists (software: Python, ROS 2, C++, JavaScript, etc.; hardware: Jetson Orin, Desktop, Laptop, Raspberry Pi, etc.) with optional "Other" free-text field.

## Assumptions

- The transactional email provider is Brevo (formerly Sendinblue), configured with personal sender email verification (no custom domain). Free tier provides 300 emails/day, sufficient for hackathon scope.
- Better-Auth runs as a standalone Node.js/Express API service, separate from the FastAPI backend. The two runtimes are decoupled; FastAPI validates sessions by querying the shared Neon Postgres database directly (no proxy layer).
- The Docusaurus frontend is a static site — auth state will be managed via React context and persisted via cookies/localStorage. Auth UI lives in dedicated Docusaurus pages under `src/pages/` (e.g., `/auth`, `/profile/settings`, `/profile/background`).
- Rate limiting and account lockout are per-account, not per-IP, for simplicity.
- OAuth providers (Google, GitHub) are out of scope for the initial implementation but the architecture should not prevent future addition.
- The "Personalize" button implementation is out of scope for this feature — only the background capture that feeds it is included.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can complete the full signup flow (form submission to verified account with background captured) in under 3 minutes, excluding email delivery wait time.
- **SC-002**: Users can sign in and reach their target page in under 30 seconds.
- **SC-003**: 100% of gated features (RAG, Translation, Personalization) are inaccessible to unauthenticated users and accessible to authenticated users.
- **SC-004**: Verification emails are delivered to any valid email address (not restricted to pre-registered recipients).
- **SC-005**: Session state persists correctly across all page navigations within the Docusaurus site without requiring re-login.
- **SC-006**: Auth state synchronizes across multiple browser tabs within 2 seconds of a login or logout event.
- **SC-007**: Failed login attempts trigger account lockout after exactly 5 consecutive failures.
- **SC-008**: Expired or used tokens are rejected 100% of the time — no token reuse is possible.
- **SC-009**: Textbook content remains fully accessible when the auth service is unavailable.
- **SC-010**: No secrets, API keys, or credentials are present in the codebase — all externalized to environment variables.
