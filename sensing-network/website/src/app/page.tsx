"use client";

import Link from "next/link";
import dynamic from "next/dynamic";
import { motion } from "framer-motion";

const ModelViewer = dynamic(() => import("@/components/ModelViewer"), {
  ssr: false,
  loading: () => (
    <div className="flex h-full w-full items-center justify-center rounded-2xl border-2 border-dashed border-border bg-surface-inset">
      <div className="text-center">
        <div className="mb-2 text-4xl text-text-tertiary">◎</div>
        <p className="text-xs text-text-tertiary">Loading 3D model...</p>
      </div>
    </div>
  ),
});

const fadeUp = {
  hidden: { opacity: 0, y: 24 },
  visible: (i: number) => ({
    opacity: 1,
    y: 0,
    transition: { delay: i * 0.1, duration: 0.5, ease: "easeOut" as const },
  }),
};

const kitItems = [
  {
    step: "01",
    title: "Conductive Filament",
    description:
      "PLA-based filament infused with conductive particles. Print as usual — your slicer, your printer, your settings.",
    color: "primary" as const,
  },
  {
    step: "02",
    title: "Rhino Plugin",
    description:
      "Design sensing networks directly in Rhino/Grasshopper. Our algorithm generates optimized electrode paths on any surface.",
    color: "secondary" as const,
  },
  {
    step: "03",
    title: "SenseBoard",
    description:
      "Arduino-compatible board with capacitive sensing built in. Plug in your print, calibrate, and start detecting touch.",
    color: "primary" as const,
  },
];

const useCases = [
  {
    title: "Interactive Prototypes",
    description: "Add touch input to physical prototypes without wiring.",
    icon: "⊞",
  },
  {
    title: "Smart Home Objects",
    description: "Print lamp shades, switches, and controllers that respond to touch.",
    icon: "◈",
  },
  {
    title: "Educational Models",
    description: "Build anatomy models, maps, or science kits with touch-responsive zones.",
    icon: "△",
  },
  {
    title: "Art Installations",
    description: "Create sculptures and surfaces that react to human contact.",
    icon: "◎",
  },
];

export default function HomePage() {
  return (
    <div>
      {/* Hero */}
      <section className="relative border-b-2 border-border">
        {/* Background grid pattern */}
        <div
          className="absolute inset-0 opacity-[0.03]"
          style={{
            backgroundImage:
              "linear-gradient(var(--color-text-primary) 1px, transparent 1px), linear-gradient(90deg, var(--color-text-primary) 1px, transparent 1px)",
            backgroundSize: "32px 32px",
          }}
        />

        <div className="relative mx-auto grid max-w-6xl items-center gap-8 px-6 pt-10 pb-24 md:grid-cols-2 md:pt-16 md:pb-32">
          {/* Left: Text content */}
          <motion.div
            initial="hidden"
            animate="visible"
          >
            <motion.div
              variants={fadeUp}
              custom={0}
              className="mb-4 inline-block rounded-full border-2 border-primary/30 bg-primary/5 px-4 py-1.5 text-xs font-semibold uppercase tracking-widest text-primary"
            >
              For Makers & Creators
            </motion.div>

            <motion.h1
              variants={fadeUp}
              custom={1}
              className="mb-6 text-4xl font-bold leading-tight tracking-tight md:text-6xl"
            >
              Make any 3D print{" "}
              <span className="text-primary">touch-sensitive</span>
            </motion.h1>

            <motion.p
              variants={fadeUp}
              custom={2}
              className="mb-8 max-w-xl text-lg leading-relaxed text-text-secondary"
            >
              SenseKit is a complete kit — conductive filament, a Rhino plugin,
              and a sensing board — that turns your 3D prints into interactive
              surfaces. No soldering. No wiring. Just print and sense.
            </motion.p>

            <motion.div
              variants={fadeUp}
              custom={3}
              className="flex flex-wrap gap-4"
            >
              <Link
                href="/pricing"
                className="inline-flex items-center gap-2 rounded-xl border-2 border-primary bg-primary px-6 py-3 text-sm font-bold uppercase tracking-widest text-white no-underline transition-all hover:-translate-y-0.5 hover:bg-primary-dark hover:shadow-lg hover:shadow-primary/20"
              >
                Get the Kit →
              </Link>
              <Link
                href="/product"
                className="inline-flex items-center gap-2 rounded-xl border-2 border-border px-6 py-3 text-sm font-bold uppercase tracking-widest text-text-primary no-underline transition-all hover:-translate-y-0.5 hover:border-border-strong hover:bg-surface-inset"
              >
                See How It Works
              </Link>
            </motion.div>
          </motion.div>

          {/* Right: 3D Model Viewer */}
          <motion.div
            initial={{ opacity: 0, scale: 0.95 }}
            animate={{ opacity: 1, scale: 1 }}
            transition={{ delay: 0.3, duration: 0.6, ease: "easeOut" as const }}
            className="relative hidden h-[480px] overflow-visible md:block"
          >
            <ModelViewer />
          </motion.div>
        </div>
      </section>

      {/* What's in the Kit */}
      <section className="border-b-2 border-border bg-surface-raised">
        <div className="mx-auto max-w-6xl px-6 py-20">
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
              The Kit
            </motion.p>
            <motion.h2
              variants={fadeUp}
              custom={1}
              className="mb-12 text-3xl font-bold tracking-tight"
            >
              Everything you need in one box
            </motion.h2>
          </motion.div>

          <div className="grid gap-6 md:grid-cols-3">
            {kitItems.map((item, i) => (
              <motion.div
                key={item.step}
                initial="hidden"
                whileInView="visible"
                viewport={{ once: true, margin: "-50px" }}
                variants={fadeUp}
                custom={i}
                className={`group rounded-2xl border-2 border-border bg-surface p-8 transition-all hover:-translate-y-1 hover:border-border-strong hover:shadow-lg ${
                  item.color === "primary"
                    ? "border-t-[3px] border-t-primary"
                    : "border-t-[3px] border-t-secondary"
                }`}
              >
                <div
                  className={`mb-4 inline-block rounded-full px-3 py-1 text-[11px] font-semibold uppercase tracking-widest text-white ${
                    item.color === "primary" ? "bg-primary" : "bg-secondary"
                  }`}
                >
                  {item.step}
                </div>
                <h3 className="mb-3 text-xl font-bold">{item.title}</h3>
                <p className="text-sm leading-relaxed text-text-secondary">
                  {item.description}
                </p>
              </motion.div>
            ))}
          </div>
        </div>
      </section>

      {/* How it Works - Steps */}
      <section className="border-b-2 border-border">
        <div className="mx-auto max-w-6xl px-6 py-20">
          <motion.div
            initial="hidden"
            whileInView="visible"
            viewport={{ once: true, margin: "-100px" }}
          >
            <motion.p
              variants={fadeUp}
              custom={0}
              className="mb-2 text-xs font-semibold uppercase tracking-widest text-secondary"
            >
              Workflow
            </motion.p>
            <motion.h2
              variants={fadeUp}
              custom={1}
              className="mb-12 text-3xl font-bold tracking-tight"
            >
              From model to interactive surface
            </motion.h2>
          </motion.div>

          <div className="grid gap-0 md:grid-cols-4">
            {[
              {
                step: "1",
                title: "Design",
                desc: "Open your 3D model in Rhino. Use the SenseKit plugin to define touch zones and generate electrode paths.",
              },
              {
                step: "2",
                title: "Print",
                desc: "Export for your slicer. Print with the conductive filament where electrodes are needed, regular filament elsewhere.",
              },
              {
                step: "3",
                title: "Calibrate",
                desc: "Connect your print to the SenseBoard. Use our web calibration tool to map each touch zone.",
              },
              {
                step: "4",
                title: "Sense",
                desc: "Your 3D print now detects touch. Read sensor data via serial, WiFi, or integrate with your own code.",
              },
            ].map((step, i) => (
              <motion.div
                key={step.step}
                initial="hidden"
                whileInView="visible"
                viewport={{ once: true, margin: "-50px" }}
                variants={fadeUp}
                custom={i}
                className="relative border-l-2 border-dashed border-border py-2 pl-8 md:border-l-0 md:border-t-2 md:pl-0 md:pt-8 md:pr-6"
              >
                <div className="absolute -left-3 top-0 flex h-6 w-6 items-center justify-center rounded-full bg-primary text-xs font-bold text-white md:-top-3 md:left-0">
                  {step.step}
                </div>
                <h3 className="mb-2 text-lg font-bold">{step.title}</h3>
                <p className="text-sm leading-relaxed text-text-secondary">
                  {step.desc}
                </p>
              </motion.div>
            ))}
          </div>
        </div>
      </section>

      {/* Use Cases */}
      <section className="border-b-2 border-border bg-surface-raised">
        <div className="mx-auto max-w-6xl px-6 py-20">
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
              Use Cases
            </motion.p>
            <motion.h2
              variants={fadeUp}
              custom={1}
              className="mb-12 text-3xl font-bold tracking-tight"
            >
              What will you build?
            </motion.h2>
          </motion.div>

          <div className="grid gap-6 sm:grid-cols-2 lg:grid-cols-4">
            {useCases.map((uc, i) => (
              <motion.div
                key={uc.title}
                initial="hidden"
                whileInView="visible"
                viewport={{ once: true, margin: "-50px" }}
                variants={fadeUp}
                custom={i}
                className="rounded-2xl border-2 border-border bg-surface p-6 transition-all hover:-translate-y-1 hover:border-primary/30 hover:shadow-md"
              >
                <div className="mb-4 flex h-12 w-12 items-center justify-center rounded-xl bg-primary/10 text-2xl text-primary">
                  {uc.icon}
                </div>
                <h3 className="mb-2 text-base font-bold">{uc.title}</h3>
                <p className="text-sm leading-relaxed text-text-secondary">
                  {uc.description}
                </p>
              </motion.div>
            ))}
          </div>
        </div>
      </section>

      {/* CTA */}
      <section className="bg-text-primary">
        <div className="mx-auto max-w-6xl px-6 py-20 text-center">
          <motion.div
            initial="hidden"
            whileInView="visible"
            viewport={{ once: true }}
          >
            <motion.h2
              variants={fadeUp}
              custom={0}
              className="mb-4 text-3xl font-bold text-white md:text-4xl"
            >
              Ready to make your prints{" "}
              <span className="text-primary">interactive</span>?
            </motion.h2>
            <motion.p
              variants={fadeUp}
              custom={1}
              className="mb-8 text-base text-text-tertiary"
            >
              Get SenseKit and start building touch-sensitive 3D prints today.
            </motion.p>
            <motion.div variants={fadeUp} custom={2}>
              <Link
                href="/pricing"
                className="inline-flex items-center gap-2 rounded-xl border-2 border-primary bg-primary px-8 py-4 text-sm font-bold uppercase tracking-widest text-white no-underline transition-all hover:-translate-y-0.5 hover:bg-primary-dark hover:shadow-lg hover:shadow-primary/30"
              >
                View Pricing →
              </Link>
            </motion.div>
          </motion.div>
        </div>
      </section>
    </div>
  );
}
