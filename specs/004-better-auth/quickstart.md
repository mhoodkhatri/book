# Quickstart: Better-Auth Development Setup

**Feature**: 004-better-auth | **Date**: 2026-01-26

## Prerequisites

- Node.js 20+ (for frontend and Better-Auth server)
- Python 3.10+ (for FastAPI backend)
- PostgreSQL client tools (optional, for debugging)
- Neon account (free tier) at https://neon.tech

## Environment Setup

### 1. Clone and Install Dependencies

```bash
# Navigate to project root
cd book

# Install frontend dependencies
npm install

# Install backend dependencies
cd backend
python -m venv .venv
.\.venv\Scripts\activate  # Windows
# source .venv/bin/activate  # Linux/Mac
pip install -r requirements.txt
cd ..
```

### 2. Create Neon Database

1. Go to https://console.neon.tech
2. Create a new project (free tier)
3. Copy the connection string from the dashboard
4. Note: It should look like: `postgresql://user:pass@ep-xxx.neon.tech/neondb?sslmode=require`

### 3. Configure Environment Variables

Create/update `.env` files:

**Backend `.env`** (in `backend/` directory):
```env
# Existing variables
GOOGLE_API_KEY=your-google-api-key
GROQ_API_KEY=your-groq-api-key
QDRANT_URL=your-qdrant-url
QDRANT_API_KEY=your-qdrant-api-key

# New auth variables
DATABASE_URL=postgresql://user:pass@ep-xxx.neon.tech/neondb?sslmode=require
BETTER_AUTH_SECRET=your-secret-key-at-least-32-characters-long
JWT_ALGORITHM=HS256
```

**Root `.env`** (for frontend, if needed):
```env
CHAT_API_URL=http://localhost:8000
```

### 4. Initialize Database Schema

Run the following SQL in Neon Console (SQL Editor) or via psql:

```sql
-- Better-Auth tables (will be auto-created, but can pre-create)
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

-- Custom user profile table
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

### 5. Install Auth Dependencies

**Frontend (npm)**:
```bash
npm install better-auth
```

**Backend (pip)**:
```bash
cd backend
pip install PyJWT psycopg2-binary
# Update requirements.txt
echo "PyJWT>=2.8.0" >> requirements.txt
echo "psycopg2-binary>=2.9.9" >> requirements.txt
cd ..
```

## Running the Development Server

### Option A: All services together (recommended)
```bash
npm run dev
```

This starts:
- Frontend: http://localhost:3000
- Backend API: http://localhost:8000

### Option B: Services separately
```bash
# Terminal 1: Backend
cd backend && .\.venv\Scripts\python -m uvicorn src.main:app --reload --port 8000

# Terminal 2: Frontend
npm run start
```

## Testing Authentication Flow

### 1. Create Test User (via API)

```bash
# Sign up
curl -X POST http://localhost:8000/api/auth/signup \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "password123",
    "name": "Test User",
    "backgroundType": "beginner_robotics"
  }'

# Response: { "user": {...}, "token": "eyJ...", "expiresAt": "..." }
```

### 2. Sign In

```bash
curl -X POST http://localhost:8000/api/auth/signin \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "password123"
  }'
```

### 3. Access Protected Endpoint

```bash
# Get the token from sign in response, then:
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer YOUR_TOKEN_HERE" \
  -d '{
    "messages": [{"role": "user", "content": "What is ROS 2?"}],
    "chapterId": "module-1-ros2/nodes-topics",
    "chapterTitle": "Nodes and Topics"
  }'
```

### 4. Test Unauthenticated Access (should fail)

```bash
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{
    "messages": [{"role": "user", "content": "What is ROS 2?"}],
    "chapterId": "module-1-ros2/nodes-topics",
    "chapterTitle": "Nodes and Topics"
  }'

# Response: 401 { "error": "unauthorized", "message": "Please sign in first" }
```

## Troubleshooting

### Database Connection Issues

```bash
# Test Neon connection
psql "postgresql://user:pass@ep-xxx.neon.tech/neondb?sslmode=require"

# Check if tables exist
\dt
```

### JWT Token Issues

```python
# Debug JWT in Python
import jwt
token = "your-token-here"
secret = "your-secret-here"
try:
    payload = jwt.decode(token, secret, algorithms=["HS256"])
    print(payload)
except jwt.InvalidTokenError as e:
    print(f"Invalid token: {e}")
```

### CORS Issues

If you see CORS errors in browser console:
1. Check that backend CORS allows your frontend origin
2. Verify `allow_credentials=True` in FastAPI middleware
3. Check that `Authorization` header is in `allow_headers`

### Session Not Persisting

1. Check browser localStorage: `localStorage.getItem('auth_token')`
2. Verify token is being included in API requests (Network tab)
3. Check token expiration (`expiresAt` in auth response)

## File Structure After Setup

```
book/
├── backend/
│   ├── src/
│   │   ├── api/
│   │   │   ├── auth.py          # New: Auth endpoints
│   │   │   ├── chat.py          # Modified: Add auth dependency
│   │   │   └── translate.py     # Modified: Add auth dependency
│   │   ├── middleware/
│   │   │   └── auth.py          # New: Auth middleware
│   │   ├── models/
│   │   │   └── auth.py          # New: Auth models
│   │   ├── services/
│   │   │   ├── auth.py          # New: Auth service
│   │   │   └── database.py      # New: DB connection
│   │   └── main.py              # Modified: Add auth router
│   ├── .env                     # Add DATABASE_URL, BETTER_AUTH_SECRET
│   └── requirements.txt         # Add PyJWT, psycopg2-binary
├── src/
│   ├── components/
│   │   └── Auth/                # New: Auth UI components
│   ├── context/
│   │   └── AuthContext.tsx      # New: Auth state
│   └── theme/
│       └── DocItem/Layout/      # Modified: Add AuthProvider
├── package.json                 # Add better-auth
└── .env                         # Add CHAT_API_URL if needed
```

## Next Steps

After completing setup:
1. Run `/sp.tasks` to generate implementation tasks
2. Implement backend auth endpoints
3. Implement frontend auth components
4. Add auth checks to protected features
5. Test full authentication flow
