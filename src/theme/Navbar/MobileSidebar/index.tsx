import React from "react";
import MobileSidebar from "@theme-original/Navbar/MobileSidebar";
import type MobileSidebarType from "@theme/Navbar/MobileSidebar";
import type { WrapperProps } from "@docusaurus/types";

type Props = WrapperProps<typeof MobileSidebarType>;

export default function MobileSidebarWrapper(props: Props): React.JSX.Element {
  return <MobileSidebar {...props} />;
}
