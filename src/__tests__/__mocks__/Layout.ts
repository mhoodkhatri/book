import React from "react";

export default function Layout({
  children,
}: {
  children: React.ReactNode;
}): React.JSX.Element {
  return React.createElement("div", { "data-testid": "layout" }, children);
}
