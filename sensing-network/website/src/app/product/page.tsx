"use client";

import Link from "next/link";
import { motion } from "framer-motion";

const fadeUp = {
  hidden: { opacity: 0, y: 24 },
  visible: (i: number) => ({
    opacity: 1,
    y: 0,
    transition: { delay: i * 0.1, duration: 0.5, ease: "easeOut" as const },
  }),
};

const specs = {
  filament: [
    { label: "Material", value: "PLA + Conductive Carbon" },
    { label: "Diameter", value: "1.75mm ± 0.05mm" },
    { label: "Print Temp", value: "200–220°C" },
    { label: "Bed Temp", value: "50–60°C" },
    { label: "Weight", value: "500g spool" },
    { label: "Resistance", value: "~1kΩ/cm (varies with geometry)" },
  ],
  board: [
    { label: "MCU", value: "Arduino Uno R4 WiFi" },
    { label: "Sensing", value: "Capacitive touch (up to 20 nodes)" },
    { label: "Connectivity", value: "USB Serial + WiFi" },
    { label: "Power", value: "USB-C, 5V" },
    { label: "Dimensions", value: "68.6 × 53.4 mm" },
    { label: "Software", value: "Arduino IDE compatible" },
  ],
  plugin: [
    { label: "Platform", value: "Rhino 7 / 8 + Grasshopper" },
    { label: "OS", value: "Windows & macOS" },
    { label: "Input", value: "Any mesh or NURBS surface" },
    { label: "Output", value: "Multi-material STL / 3MF" },
    { label: "Nodes", value: "1–100+ sensing zones" },
    { label: "License", value: "Perpetual, single-seat" },
  ],
};

export default function ProductPage() {
  return (
    <div>
      {/* Hero */}
      <section className="border-b-2 border-border">
        <div className="mx-auto max-w-6xl px-6 py-16 md:py-24">
          <motion.div initial="hidden" animate="visible">
            <motion.p
              variants={fadeUp}
              custom={0}
              className="mb-2 text-xs font-semibold uppercase tracking-widest text-primary"
            >
              How It Works
            </motion.p>
            <motion.h1
              variants={fadeUp}
              custom={1}
              className="mb-4 text-4xl font-bold tracking-tight md:text-5xl"
            >
              Three components.{" "}
              <span className="text-primary">One workflow.</span>
            </motion.h1>
            <motion.p
              variants={fadeUp}
              custom={2}
              className="max-w-2xl text-lg leading-relaxed text-text-secondary"
            >
              SenseKit combines conductive 3D printing filament, a computational
              design plugin, and sensing hardware into a seamless pipeline for
              creating touch-sensitive objects.
            </motion.p>
          </motion.div>
        </div>
      </section>

      {/* Component 1: Filament */}
      <section className="border-b-2 border-border">
        <div className="mx-auto max-w-6xl px-6 py-16">
          <div className="grid items-start gap-12 md:grid-cols-2">
            <motion.div
              initial="hidden"
              whileInView="visible"
              viewport={{ once: true, margin: "-100px" }}
            >
              <motion.div
                variants={fadeUp}
                custom={0}
                className="mb-4 inline-block rounded-full bg-primary px-3 py-1 text-[11px] font-semibold uppercase tracking-widest text-white"
              >
                Component 01
              </motion.div>
              <motion.h2
                variants={fadeUp}
                custom={1}
                className="mb-4 text-3xl font-bold"
              >
                Conductive Filament
              </motion.h2>
              <motion.p
                variants={fadeUp}
                custom={2}
                className="mb-6 text-base leading-relaxed text-text-secondary"
              >
                Our PLA-based filament is infused with conductive carbon
                particles. It prints on any standard FDM printer — no special
                hardware needed. The filament creates resistive traces within
                your print that act as electrodes for capacitive touch sensing.
              </motion.p>
              <motion.ul
                variants={fadeUp}
                custom={3}
                className="flex flex-col gap-3"
              >
                {[
                  "Works with any FDM printer (Prusa, Bambu, Ender, etc.)",
                  "Compatible with standard PLA print profiles",
                  "Multi-material printing for selective conductivity",
                  "Food-safe PLA base material",
                ].map((item) => (
                  <li
                    key={item}
                    className="flex items-start gap-3 text-sm text-text-secondary"
                  >
                    <span className="mt-0.5 text-primary">▸</span>
                    {item}
                  </li>
                ))}
              </motion.ul>
            </motion.div>

            {/* Spec table */}
            <motion.div
              initial="hidden"
              whileInView="visible"
              viewport={{ once: true, margin: "-100px" }}
              variants={fadeUp}
              custom={2}
              className="rounded-2xl border-2 border-border bg-surface-raised p-6"
            >
              <h4 className="mb-4 text-xs font-semibold uppercase tracking-widest text-text-tertiary">
                Technical Specs
              </h4>
              <div className="flex flex-col divide-y divide-border">
                {specs.filament.map((s) => (
                  <div
                    key={s.label}
                    className="flex items-center justify-between py-3"
                  >
                    <span className="text-sm text-text-secondary">
                      {s.label}
                    </span>
                    <span className="text-sm font-semibold">{s.value}</span>
                  </div>
                ))}
              </div>
            </motion.div>
          </div>
        </div>
      </section>

      {/* Component 2: Plugin */}
      <section className="border-b-2 border-border bg-surface-raised">
        <div className="mx-auto max-w-6xl px-6 py-16">
          <div className="grid items-start gap-12 md:grid-cols-2">
            <motion.div
              initial="hidden"
              whileInView="visible"
              viewport={{ once: true, margin: "-100px" }}
              variants={fadeUp}
              custom={2}
              className="order-2 rounded-2xl border-2 border-border bg-surface p-6 md:order-1"
            >
              <h4 className="mb-4 text-xs font-semibold uppercase tracking-widest text-text-tertiary">
                Technical Specs
              </h4>
              <div className="flex flex-col divide-y divide-border">
                {specs.plugin.map((s) => (
                  <div
                    key={s.label}
                    className="flex items-center justify-between py-3"
                  >
                    <span className="text-sm text-text-secondary">
                      {s.label}
                    </span>
                    <span className="text-sm font-semibold">{s.value}</span>
                  </div>
                ))}
              </div>
            </motion.div>

            <motion.div
              initial="hidden"
              whileInView="visible"
              viewport={{ once: true, margin: "-100px" }}
              className="order-1 md:order-2"
            >
              <motion.div
                variants={fadeUp}
                custom={0}
                className="mb-4 inline-block rounded-full bg-secondary px-3 py-1 text-[11px] font-semibold uppercase tracking-widest text-white"
              >
                Component 02
              </motion.div>
              <motion.h2
                variants={fadeUp}
                custom={1}
                className="mb-4 text-3xl font-bold"
              >
                Rhino / Grasshopper Plugin
              </motion.h2>
              <motion.p
                variants={fadeUp}
                custom={2}
                className="mb-6 text-base leading-relaxed text-text-secondary"
              >
                The SenseKit plugin integrates directly into your Rhino +
                Grasshopper workflow. Select any surface, define touch zones, and
                the algorithm automatically generates optimized electrode
                networks that follow your geometry.
              </motion.p>
              <motion.ul
                variants={fadeUp}
                custom={3}
                className="flex flex-col gap-3"
              >
                {[
                  "Drag-and-drop Grasshopper components",
                  "Automatic electrode path optimization",
                  "Real-time preview of sensing regions",
                  "One-click export to multi-material STL",
                ].map((item) => (
                  <li
                    key={item}
                    className="flex items-start gap-3 text-sm text-text-secondary"
                  >
                    <span className="mt-0.5 text-secondary">▸</span>
                    {item}
                  </li>
                ))}
              </motion.ul>
            </motion.div>
          </div>
        </div>
      </section>

      {/* Component 3: Board */}
      <section className="border-b-2 border-border">
        <div className="mx-auto max-w-6xl px-6 py-16">
          <div className="grid items-start gap-12 md:grid-cols-2">
            <motion.div
              initial="hidden"
              whileInView="visible"
              viewport={{ once: true, margin: "-100px" }}
            >
              <motion.div
                variants={fadeUp}
                custom={0}
                className="mb-4 inline-block rounded-full bg-primary px-3 py-1 text-[11px] font-semibold uppercase tracking-widest text-white"
              >
                Component 03
              </motion.div>
              <motion.h2
                variants={fadeUp}
                custom={1}
                className="mb-4 text-3xl font-bold"
              >
                SenseBoard
              </motion.h2>
              <motion.p
                variants={fadeUp}
                custom={2}
                className="mb-6 text-base leading-relaxed text-text-secondary"
              >
                The SenseBoard is an Arduino Uno R4 WiFi pre-loaded with our
                capacitive sensing firmware. Connect your 3D print with simple
                clip-on connectors, run the web-based calibration tool, and start
                reading touch data immediately.
              </motion.p>
              <motion.ul
                variants={fadeUp}
                custom={3}
                className="flex flex-col gap-3"
              >
                {[
                  "Up to 20 independent sensing nodes",
                  "WiFi + USB connectivity",
                  "Web-based calibration interface",
                  "Arduino IDE for custom firmware",
                ].map((item) => (
                  <li
                    key={item}
                    className="flex items-start gap-3 text-sm text-text-secondary"
                  >
                    <span className="mt-0.5 text-primary">▸</span>
                    {item}
                  </li>
                ))}
              </motion.ul>
            </motion.div>

            <motion.div
              initial="hidden"
              whileInView="visible"
              viewport={{ once: true, margin: "-100px" }}
              variants={fadeUp}
              custom={2}
              className="rounded-2xl border-2 border-border bg-surface-raised p-6"
            >
              <h4 className="mb-4 text-xs font-semibold uppercase tracking-widest text-text-tertiary">
                Technical Specs
              </h4>
              <div className="flex flex-col divide-y divide-border">
                {specs.board.map((s) => (
                  <div
                    key={s.label}
                    className="flex items-center justify-between py-3"
                  >
                    <span className="text-sm text-text-secondary">
                      {s.label}
                    </span>
                    <span className="text-sm font-semibold">{s.value}</span>
                  </div>
                ))}
              </div>
            </motion.div>
          </div>
        </div>
      </section>

      {/* Workflow Diagram */}
      <section className="border-b-2 border-border bg-surface-raised">
        <div className="mx-auto max-w-6xl px-6 py-16">
          <motion.div
            initial="hidden"
            whileInView="visible"
            viewport={{ once: true, margin: "-100px" }}
          >
            <motion.p
              variants={fadeUp}
              custom={0}
              className="mb-2 text-xs font-semibold uppercase tracking-widest text-primary"
            >
              Pipeline
            </motion.p>
            <motion.h2
              variants={fadeUp}
              custom={1}
              className="mb-12 text-3xl font-bold"
            >
              The full workflow
            </motion.h2>
          </motion.div>

          <motion.div
            initial="hidden"
            whileInView="visible"
            viewport={{ once: true, margin: "-50px" }}
            variants={fadeUp}
            custom={2}
            className="rounded-2xl border-2 border-border bg-surface p-8"
          >
            <div className="flex flex-col items-center gap-4 md:flex-row md:gap-0">
              {[
                { label: "3D Model", sub: "Rhino / Any CAD" },
                { label: "Sensing Design", sub: "Grasshopper Plugin" },
                { label: "Slicing", sub: "Multi-material" },
                { label: "Printing", sub: "FDM Printer" },
                { label: "Calibration", sub: "Web Tool" },
                { label: "Interaction", sub: "Touch Sensing" },
              ].map((step, i, arr) => (
                <div
                  key={step.label}
                  className="flex items-center gap-0 md:flex-1"
                >
                  <div className="flex flex-col items-center text-center">
                    <div className="mb-2 flex h-10 w-10 items-center justify-center rounded-full bg-primary text-sm font-bold text-white">
                      {i + 1}
                    </div>
                    <div className="text-sm font-semibold">{step.label}</div>
                    <div className="text-xs text-text-tertiary">{step.sub}</div>
                  </div>
                  {i < arr.length - 1 && (
                    <div className="mx-2 hidden h-px flex-1 border-t-2 border-dashed border-border md:block" />
                  )}
                </div>
              ))}
            </div>
          </motion.div>
        </div>
      </section>

      {/* CTA */}
      <section className="bg-text-primary">
        <div className="mx-auto max-w-6xl px-6 py-16 text-center">
          <h2 className="mb-4 text-3xl font-bold text-white">
            Ready to start building?
          </h2>
          <p className="mb-8 text-text-tertiary">
            Pick the kit that fits your project and start making interactive
            prints.
          </p>
          <Link
            href="/pricing"
            className="inline-flex items-center rounded-xl border-2 border-primary bg-primary px-8 py-4 text-sm font-bold uppercase tracking-widest text-white no-underline transition-all hover:-translate-y-0.5 hover:bg-primary-dark hover:shadow-lg hover:shadow-primary/30"
          >
            View Pricing →
          </Link>
        </div>
      </section>
    </div>
  );
}
