"""Authentication middleware for FastAPI.

Validates Better-Auth session cookies by querying the shared Neon Postgres
session table directly. No HTTP call to the auth service needed.
"""

import os
import logging
from typing import Optional

import asyncpg
from fastapi import Depends, HTTPException, Request

logger = logging.getLogger(__name__)

_pool: Optional[asyncpg.Pool] = None


async def get_db_pool() -> asyncpg.Pool:
    """Get or create the asyncpg connection pool."""
    global _pool
    if _pool is None:
        database_url = os.getenv("DATABASE_URL")
        if not database_url:
            raise RuntimeError("DATABASE_URL environment variable is required for auth")
        _pool = await asyncpg.create_pool(database_url, min_size=1, max_size=5)
    return _pool


async def close_db_pool() -> None:
    """Close the asyncpg connection pool."""
    global _pool
    if _pool:
        await _pool.close()
        _pool = None


async def get_current_user(request: Request) -> dict:
    """FastAPI dependency that validates the Better-Auth session cookie.

    Reads the `better-auth.session_token` cookie, queries the session table
    in the shared Neon Postgres database, and returns user data.

    Raises HTTPException(401) if unauthenticated.
    """
    # Better-Auth cookie name — value is "TOKEN.SIGNATURE", DB stores only TOKEN
    raw_token = request.cookies.get("better-auth.session_token")

    if not raw_token:
        raise HTTPException(status_code=401, detail="Authentication required")

    # Extract the token part (before the dot), discard the HMAC signature
    session_token = raw_token.split(".")[0] if "." in raw_token else raw_token

    try:
        pool = await get_db_pool()
        row = await pool.fetchrow(
            """
            SELECT s.id as session_id, s.token, s."expiresAt",
                   u.id as user_id, u.name, u.email, u."emailVerified"
            FROM session s
            JOIN "user" u ON s."userId" = u.id
            WHERE s.token = $1 AND s."expiresAt" > NOW()
            """,
            session_token,
        )

        if not row:
            raise HTTPException(status_code=401, detail="Authentication required")

        return {
            "id": row["user_id"],
            "name": row["name"],
            "email": row["email"],
            "emailVerified": row["emailVerified"],
            "sessionId": row["session_id"],
        }

    except HTTPException:
        raise
    except Exception as e:
        logger.error("Session validation error: %s", str(e))
        raise HTTPException(status_code=401, detail="Authentication required")
