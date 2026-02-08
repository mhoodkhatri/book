import React from "react";

export default function BrowserOnly({
  children,
}: {
  children: () => React.JSX.Element;
}): React.JSX.Element {
  return children();
}
