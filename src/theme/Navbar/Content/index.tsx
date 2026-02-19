import React, { useEffect } from "react";
import Content from "@theme-original/Navbar/Content";
import type ContentType from "@theme/Navbar/Content";
import type { WrapperProps } from "@docusaurus/types";
import { useLocation } from "@docusaurus/router";
import useBaseUrl from "@docusaurus/useBaseUrl";
import AuthNavbar from "@site/src/components/Auth/AuthNavbar";

type Props = WrapperProps<typeof ContentType>;

/**
 * Transparent-on-homepage scroll watcher.
 * Sets [data-navbar-transparent] on <html> when at the top of the homepage.
 */
function NavbarTransparencyController(): null {
  const location = useLocation();
  const homeBase = useBaseUrl("/");

  useEffect(() => {
    const isHome =
      location.pathname === homeBase ||
      location.pathname === homeBase.replace(/\/$/, "");

    if (!isHome) {
      document.documentElement.removeAttribute("data-navbar-transparent");
      return;
    }

    const onScroll = () => {
      if (window.scrollY < 60) {
        document.documentElement.setAttribute("data-navbar-transparent", "");
      } else {
        document.documentElement.removeAttribute("data-navbar-transparent");
      }
    };

    onScroll(); // initial check
    window.addEventListener("scroll", onScroll, { passive: true });
    return () => {
      window.removeEventListener("scroll", onScroll);
      document.documentElement.removeAttribute("data-navbar-transparent");
    };
  }, [location.pathname, homeBase]);

  return null;
}

/**
 * Swizzled Navbar/Content to inject AuthNavbar + homepage transparency.
 *
 * Uses a React Fragment (no extra wrapper divs) so we don't break
 * the mobile sidebar's internal DOM structure on doc pages.
 * AuthNavbar is hidden inside the sidebar via CSS.
 *
 * No BrowserOnly wrapper — both child components only access
 * browser APIs inside useEffect, which is SSR-safe.
 */
export default function ContentWrapper(props: Props): React.JSX.Element {
  return (
    <>
      <NavbarTransparencyController />
      <Content {...props} />
      <AuthNavbar />
    </>
  );
}
