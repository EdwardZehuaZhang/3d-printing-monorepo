import Link from "next/link";

export default function Footer() {
  return (
    <footer className="border-t-2 border-border bg-surface-raised">
      <div className="mx-auto max-w-6xl px-6 py-12">
        <div className="grid gap-8 md:grid-cols-4">
          <div>
            <h3 className="mb-3 text-lg font-bold">
              <span className="text-primary">//</span> SenseKit
            </h3>
            <p className="text-sm leading-relaxed text-text-secondary">
              Turn any 3D print into an interactive, touch-sensitive surface.
              Made for makers, by makers.
            </p>
          </div>

          <div>
            <h4 className="mb-3 text-xs font-semibold uppercase tracking-widest text-text-tertiary">
              Product
            </h4>
            <div className="flex flex-col gap-2">
              <Link href="/product" className="text-sm text-text-secondary no-underline hover:text-primary">
                How It Works
              </Link>
              <Link href="/pricing" className="text-sm text-text-secondary no-underline hover:text-primary">
                Pricing
              </Link>
              <Link href="/calibrate" className="text-sm text-text-secondary no-underline hover:text-primary">
                Calibration Tool
              </Link>
            </div>
          </div>

          <div>
            <h4 className="mb-3 text-xs font-semibold uppercase tracking-widest text-text-tertiary">
              Resources
            </h4>
            <div className="flex flex-col gap-2">
              <Link href="/docs" className="text-sm text-text-secondary no-underline hover:text-primary">
                Getting Started
              </Link>
              <a href="https://github.com" className="text-sm text-text-secondary no-underline hover:text-primary">
                GitHub
              </a>
              <a href="#" className="text-sm text-text-secondary no-underline hover:text-primary">
                Community
              </a>
            </div>
          </div>

          <div>
            <h4 className="mb-3 text-xs font-semibold uppercase tracking-widest text-text-tertiary">
              Research
            </h4>
            <p className="text-sm leading-relaxed text-text-secondary">
              Based on &ldquo;A Computational Design Pipeline to Fabricate
              Sensing Network Physicalizations&rdquo; — IEEE TVCG 2024.
            </p>
          </div>
        </div>

        <div className="mt-8 flex flex-col items-center justify-between gap-4 border-t border-border pt-8 md:flex-row">
          <p className="text-xs text-text-tertiary">
            &copy; {new Date().getFullYear()} SenseKit. All rights reserved.
          </p>
          <p className="text-xs text-text-tertiary">
            Made with conductive filament & love
          </p>
        </div>
      </div>
    </footer>
  );
}
