import { defineConfig } from "vitest/config";
import path from "path";

export default defineConfig({
  test: {
    environment: "jsdom",
    globals: true,
    setupFiles: ["./src/__tests__/setup.ts"],
    include: ["src/**/*.test.{ts,tsx}"],
  },
  resolve: {
    alias: {
      "@site": path.resolve(__dirname, "."),
      "@docusaurus/useBaseUrl": path.resolve(
        __dirname,
        "src/__tests__/__mocks__/useBaseUrl.ts"
      ),
      "@docusaurus/BrowserOnly": path.resolve(
        __dirname,
        "src/__tests__/__mocks__/BrowserOnly.ts"
      ),
      "@theme/Layout": path.resolve(
        __dirname,
        "src/__tests__/__mocks__/Layout.ts"
      ),
    },
  },
});
