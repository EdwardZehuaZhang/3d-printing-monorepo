"use client";

import Link from "next/link";
import { usePathname } from "next/navigation";
import { useState } from "react";

const navLinks = [
  { href: "/", label: "Home" },
  { href: "/product", label: "How It Works" },
  { href: "/pricing", label: "Pricing" },
  { href: "/docs", label: "Docs" },
  { href: "/calibrate", label: "Calibrate" },
];

export default function Navigation() {
  const pathname = usePathname();
  const [mobileOpen, setMobileOpen] = useState(false);

  return (
    <nav className="sticky top-0 z-50 border-b-2 border-border bg-surface/95 backdrop-blur-sm">
      <div className="mx-auto flex max-w-6xl items-center justify-between px-6 py-4">
        <Link href="/" className="flex items-center gap-2 no-underline">
          <span className="text-2xl font-bold tracking-tight text-text-primary">
            <span className="text-primary">//</span> SenseKit
          </span>
        </Link>

        {/* Desktop nav */}
        <div className="hidden items-center gap-1 md:flex">
          {navLinks.map((link) => (
            <Link
              key={link.href}
              href={link.href}
              className={`rounded-lg px-3 py-2 text-xs font-semibold uppercase tracking-widest no-underline transition-all ${
                pathname === link.href
                  ? "bg-primary text-white"
                  : "text-text-secondary hover:bg-surface-inset hover:text-text-primary"
              }`}
            >
              {link.label}
            </Link>
          ))}
          <Link
            href="/pricing"
            className="ml-3 rounded-lg border-2 border-primary bg-primary px-4 py-2 text-xs font-semibold uppercase tracking-widest text-white no-underline transition-all hover:-translate-y-0.5 hover:bg-primary-dark hover:shadow-lg"
          >
            Get Kit
          </Link>
        </div>

        {/* Mobile hamburger */}
        <button
          onClick={() => setMobileOpen(!mobileOpen)}
          className="flex flex-col gap-1.5 bg-transparent p-2 md:hidden"
          aria-label="Toggle menu"
        >
          <span
            className={`block h-0.5 w-5 bg-text-primary transition-all ${mobileOpen ? "translate-y-2 rotate-45" : ""}`}
          />
          <span
            className={`block h-0.5 w-5 bg-text-primary transition-all ${mobileOpen ? "opacity-0" : ""}`}
          />
          <span
            className={`block h-0.5 w-5 bg-text-primary transition-all ${mobileOpen ? "-translate-y-2 -rotate-45" : ""}`}
          />
        </button>
      </div>

      {/* Mobile menu */}
      {mobileOpen && (
        <div className="border-t border-border bg-surface px-6 py-4 md:hidden">
          {navLinks.map((link) => (
            <Link
              key={link.href}
              href={link.href}
              onClick={() => setMobileOpen(false)}
              className={`block rounded-lg px-3 py-3 text-sm font-semibold uppercase tracking-widest no-underline ${
                pathname === link.href
                  ? "bg-primary/10 text-primary"
                  : "text-text-secondary"
              }`}
            >
              {link.label}
            </Link>
          ))}
        </div>
      )}
    </nav>
  );
}
