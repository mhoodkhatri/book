import type { Request, Response } from "express";
import { auth, pool } from "../auth.js";
import { fromNodeHeaders } from "better-auth/node";

const VALID_SOFTWARE = ["Python", "ROS 2", "C++", "JavaScript", "MATLAB", "Bash/Shell"];
const VALID_HARDWARE = ["Jetson Orin", "Desktop Workstation", "Laptop", "Raspberry Pi", "Cloud/VM"];

export async function backgroundHandler(req: Request, res: Response): Promise<void> {
  try {
    // Validate session
    const session = await auth.api.getSession({
      headers: fromNodeHeaders(req.headers),
    });

    if (!session?.user) {
      res.status(401).json({ error: "Authentication required" });
      return;
    }

    const { softwareBackground, hardwareBackground, softwareOther, hardwareOther } = req.body;

    // Validate software background — must be array if provided
    if (softwareBackground !== undefined) {
      if (!Array.isArray(softwareBackground)) {
        res.status(400).json({ error: "softwareBackground must be an array" });
        return;
      }
      const invalid = softwareBackground.filter((s: string) => !VALID_SOFTWARE.includes(s));
      if (invalid.length > 0) {
        res.status(400).json({ error: `Invalid software options: ${invalid.join(", ")}` });
        return;
      }
    }

    // Validate hardware background — must be array if provided
    if (hardwareBackground !== undefined) {
      if (!Array.isArray(hardwareBackground)) {
        res.status(400).json({ error: "hardwareBackground must be an array" });
        return;
      }
      const invalid = hardwareBackground.filter((h: string) => !VALID_HARDWARE.includes(h));
      if (invalid.length > 0) {
        res.status(400).json({ error: `Invalid hardware options: ${invalid.join(", ")}` });
        return;
      }
    }

    // Validate "Other" fields length
    if (softwareOther && typeof softwareOther === "string" && softwareOther.length > 255) {
      res.status(400).json({ error: "softwareOther must be 255 characters or less" });
      return;
    }
    if (hardwareOther && typeof hardwareOther === "string" && hardwareOther.length > 255) {
      res.status(400).json({ error: "hardwareOther must be 255 characters or less" });
      return;
    }

    // Update user background in database
    const result = await pool.query(
      `UPDATE "user"
       SET "softwareBackground" = $1,
           "hardwareBackground" = $2,
           "softwareOther" = $3,
           "hardwareOther" = $4,
           "backgroundCompleted" = true,
           "updatedAt" = NOW()
       WHERE id = $5
       RETURNING id, name, email, "softwareBackground", "hardwareBackground", "softwareOther", "hardwareOther", "backgroundCompleted"`,
      [
        JSON.stringify(softwareBackground || []),
        JSON.stringify(hardwareBackground || []),
        softwareOther || null,
        hardwareOther || null,
        session.user.id,
      ]
    );

    if (result.rows.length === 0) {
      res.status(404).json({ error: "User not found" });
      return;
    }

    const user = result.rows[0];
    res.json({
      id: user.id,
      name: user.name,
      email: user.email,
      softwareBackground: JSON.parse(user.softwareBackground || "[]"),
      hardwareBackground: JSON.parse(user.hardwareBackground || "[]"),
      softwareOther: user.softwareOther,
      hardwareOther: user.hardwareOther,
      backgroundCompleted: user.backgroundCompleted,
    });
  } catch (error) {
    console.error("Background update error:", error);
    res.status(500).json({ error: "Internal server error" });
  }
}
