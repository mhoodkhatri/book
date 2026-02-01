import React from "react";
import Content from "@theme-original/Navbar/Content";
import type ContentType from "@theme/Navbar/Content";
import type { WrapperProps } from "@docusaurus/types";
import BrowserOnly from "@docusaurus/BrowserOnly";
import AuthNavbar from "@site/src/components/Auth/AuthNavbar";

type Props = WrapperProps<typeof ContentType>;

/**
 * Swizzled Navbar/Content to inject AuthNavbar on the right side.
 */
export default function ContentWrapper(props: Props): React.JSX.Element {
  return (
    <div style={{ display: "flex", alignItems: "center", width: "100%" }}>
      <div style={{ flex: 1 }}>
        <Content {...props} />
      </div>
      <BrowserOnly>{() => <AuthNavbar />}</BrowserOnly>
    </div>
  );
}
