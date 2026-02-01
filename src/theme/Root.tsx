import React from "react";
import { AuthContextProvider } from "@site/src/contexts/AuthContext";
import { ToastProvider } from "@site/src/components/Auth/Toast";

interface Props {
  children: React.ReactNode;
}

/**
 * Docusaurus Root wrapper.
 * Provides auth state and toast notifications to all pages.
 */
export default function Root({ children }: Props): React.JSX.Element {
  return (
    <ToastProvider>
      <AuthContextProvider>{children}</AuthContextProvider>
    </ToastProvider>
  );
}
