import React from "react";
import PrimaryMenu from "@theme-original/Navbar/MobileSidebar/PrimaryMenu";
import type PrimaryMenuType from "@theme/Navbar/MobileSidebar/PrimaryMenu";
import type { WrapperProps } from "@docusaurus/types";
import SearchBar from "@theme/SearchBar";

type Props = WrapperProps<typeof PrimaryMenuType>;

/**
 * Wraps the mobile sidebar PrimaryMenu to include the SearchBar
 * at the top. The search bar is hidden from the top navbar on
 * mobile via CSS, so this is its only visible instance.
 */
export default function PrimaryMenuWrapper(props: Props): React.JSX.Element {
  return (
    <>
      <div className="mobile-sidebar-search">
        <SearchBar />
      </div>
      <PrimaryMenu {...props} />
    </>
  );
}
