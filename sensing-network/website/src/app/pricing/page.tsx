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

const tiers = [
  {
    name: "Starter",
    price: 49,
    description: "Just the filament — for makers who already have hardware.",
    color: "border-t-text-tertiary",
    badge: null,
    features: [
      { text: "500g conductive filament spool", included: true },
      { text: "1.75mm PLA + Carbon", included: true },
      { text: "Printing guide PDF", included: true },
      { text: "Community Discord access", included: true },
      { text: "SenseBoard hardware", included: false },
      { text: "Rhino/Grasshopper plugin", included: false },
      { text: "Calibration tool access", included: false },
      { text: "Priority support", included: false },
    ],
    cta: "Get Filament",
    ctaStyle: "border-2 border-border text-text-primary hover:bg-surface-inset",
  },
  {
    name: "Pro",
    price: 99,
    description:
      "Filament + SenseBoard — everything to start sensing, no plugin required.",
    color: "border-t-primary",
    badge: "Most Popular",
    features: [
      { text: "500g conductive filament spool", included: true },
      { text: "1.75mm PLA + Carbon", included: true },
      { text: "Printing guide PDF", included: true },
      { text: "Community Discord access", included: true },
      { text: "SenseBoard (Arduino R4 WiFi)", included: true },
      { text: "Web calibration tool", included: true },
      { text: "Rhino/Grasshopper plugin", included: false },
      { text: "Priority support", included: false },
    ],
    cta: "Get Pro Kit",
    ctaStyle:
      "border-2 border-primary bg-primary text-white hover:bg-primary-dark hover:shadow-lg hover:shadow-primary/20",
  },
  {
    name: "Full Kit",
    price: 149,
    description:
      "The complete SenseKit — design, print, calibrate, and sense.",
    color: "border-t-secondary",
    badge: "Complete",
    features: [
      { text: "500g conductive filament spool", included: true },
      { text: "1.75mm PLA + Carbon", included: true },
      { text: "Printing guide PDF", included: true },
      { text: "Community Discord access", included: true },
      { text: "SenseBoard (Arduino R4 WiFi)", included: true },
      { text: "Web calibration tool", included: true },
      { text: "Rhino/Grasshopper plugin license", included: true },
      { text: "Priority email support", included: true },
    ],
    cta: "Get Full Kit",
    ctaStyle:
      "border-2 border-secondary bg-secondary text-white hover:bg-secondary-dark hover:shadow-lg hover:shadow-secondary/20",
  },
];

const faqs = [
  {
    q: "What printers are compatible?",
    a: "Any FDM printer that supports 1.75mm PLA filament — Prusa, Bambu Lab, Creality, Elegoo, etc. No modifications needed.",
  },
  {
    q: "Do I need a multi-material setup?",
    a: "Not required, but recommended. Multi-material lets you mix conductive and regular PLA for selective sensing zones. Single-material prints work too — the entire surface becomes conductive.",
  },
  {
    q: "Is the Rhino plugin required?",
    a: "No. The Starter and Pro kits work without it. You can design electrode patterns manually or use any CAD tool. The plugin automates the process and optimizes layouts.",
  },
  {
    q: "How many touch zones can I create?",
    a: "The SenseBoard supports up to 20 independent sensing nodes. With the plugin, you can design complex multi-zone layouts with automatic routing.",
  },
  {
    q: "Can I use my own Arduino?",
    a: "Yes. The SenseBoard is an Arduino Uno R4 WiFi with our firmware pre-loaded. You can flash the firmware onto your own R4 WiFi — it's open source.",
  },
  {
    q: "Do you ship internationally?",
    a: "Yes, we ship worldwide. Shipping is calculated at checkout based on your location.",
  },
];

export default function PricingPage() {
  return (
    <div>
      {/* Header */}
      <section className="border-b-2 border-border">
        <div className="mx-auto max-w-6xl px-6 py-16 text-center md:py-24">
          <motion.div initial="hidden" animate="visible">
            <motion.p
              variants={fadeUp}
              custom={0}
              className="mb-2 text-xs font-semibold uppercase tracking-widest text-primary"
            >
              Pricing
            </motion.p>
            <motion.h1
              variants={fadeUp}
              custom={1}
              className="mb-4 text-4xl font-bold tracking-tight md:text-5xl"
            >
              Pick your <span className="text-primary">kit</span>
            </motion.h1>
            <motion.p
              variants={fadeUp}
              custom={2}
              className="mx-auto max-w-lg text-base text-text-secondary"
            >
              Start with just the filament, or get the complete package. Every
              kit includes free shipping within the US.
            </motion.p>
          </motion.div>
        </div>
      </section>

      {/* Pricing Cards */}
      <section className="border-b-2 border-border bg-surface-raised">
        <div className="mx-auto max-w-6xl px-6 py-16">
          <div className="grid gap-6 md:grid-cols-3">
            {tiers.map((tier, i) => (
              <motion.div
                key={tier.name}
                initial="hidden"
                whileInView="visible"
                viewport={{ once: true, margin: "-50px" }}
                variants={fadeUp}
                custom={i}
                className={`relative flex flex-col rounded-2xl border-2 border-border border-t-[3px] bg-surface p-8 transition-all hover:-translate-y-1 hover:shadow-lg ${tier.color}`}
              >
                {tier.badge && (
                  <div className="absolute -top-3 right-6 rounded-full bg-primary px-3 py-1 text-[11px] font-semibold uppercase tracking-widest text-white">
                    {tier.badge}
                  </div>
                )}

                <h3 className="mb-1 text-xl font-bold">{tier.name}</h3>
                <p className="mb-6 text-sm text-text-secondary">
                  {tier.description}
                </p>

                <div className="mb-6">
                  <span className="text-4xl font-bold">${tier.price}</span>
                  <span className="text-sm text-text-tertiary"> / kit</span>
                </div>

                <div className="mb-8 flex flex-1 flex-col gap-3">
                  {tier.features.map((f) => (
                    <div
                      key={f.text}
                      className={`flex items-start gap-3 text-sm ${
                        f.included ? "text-text-primary" : "text-text-tertiary line-through"
                      }`}
                    >
                      <span
                        className={`mt-0.5 ${f.included ? "text-success" : "text-text-tertiary"}`}
                      >
                        {f.included ? "✓" : "—"}
                      </span>
                      {f.text}
                    </div>
                  ))}
                </div>

                <button
                  className={`w-full rounded-xl px-6 py-3 text-sm font-bold uppercase tracking-widest transition-all hover:-translate-y-0.5 ${tier.ctaStyle}`}
                >
                  {tier.cta}
                </button>
              </motion.div>
            ))}
          </div>
        </div>
      </section>

      {/* Comparison note */}
      <section className="border-b-2 border-border">
        <div className="mx-auto max-w-6xl px-6 py-12">
          <div className="rounded-2xl border-2 border-dashed border-primary/30 bg-primary/5 p-8 text-center">
            <p className="mb-2 text-sm font-semibold text-primary">
              Not sure which to pick?
            </p>
            <p className="text-sm text-text-secondary">
              The <strong>Pro Kit</strong> is perfect for most makers. It
              includes everything you need to print and sense. Add the Rhino
              plugin later if you want automated electrode design.
            </p>
          </div>
        </div>
      </section>

      {/* FAQ */}
      <section className="border-b-2 border-border bg-surface-raised">
        <div className="mx-auto max-w-3xl px-6 py-16">
          <motion.div
            initial="hidden"
            whileInView="visible"
            viewport={{ once: true }}
          >
            <motion.h2
              variants={fadeUp}
              custom={0}
              className="mb-8 text-center text-3xl font-bold"
            >
              Frequently asked questions
            </motion.h2>
          </motion.div>

          <div className="flex flex-col gap-4">
            {faqs.map((faq, i) => (
              <motion.details
                key={faq.q}
                initial="hidden"
                whileInView="visible"
                viewport={{ once: true, margin: "-30px" }}
                variants={fadeUp}
                custom={i}
                className="group rounded-xl border-2 border-border bg-surface"
              >
                <summary className="cursor-pointer px-6 py-4 text-sm font-semibold list-none">
                  <span className="flex items-center justify-between">
                    {faq.q}
                    <span className="ml-4 text-text-tertiary transition-transform group-open:rotate-45">
                      +
                    </span>
                  </span>
                </summary>
                <div className="border-t border-border px-6 py-4 text-sm leading-relaxed text-text-secondary">
                  {faq.a}
                </div>
              </motion.details>
            ))}
          </div>
        </div>
      </section>
    </div>
  );
}
