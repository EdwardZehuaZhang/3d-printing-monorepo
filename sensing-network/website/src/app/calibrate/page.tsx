"use client";

import { motion } from "framer-motion";

const fadeUp = {
  hidden: { opacity: 0, y: 24 },
  visible: (i: number) => ({
    opacity: 1,
    y: 0,
    transition: { delay: i * 0.1, duration: 0.5, ease: "easeOut" as const },
  }),
};

export default function CalibratePage() {
  return (
    <div>
      {/* Header */}
      <section className="border-b-2 border-border">
        <div className="mx-auto max-w-6xl px-6 py-12">
          <motion.div initial="hidden" animate="visible">
            <motion.div
              variants={fadeUp}
              custom={0}
              className="mb-3 flex items-center gap-3"
            >
              <span className="text-xs font-semibold uppercase tracking-widest text-primary">
                Calibration Tool
              </span>
              <div className="flex items-center gap-2 rounded-full border border-border bg-surface-inset px-3 py-1">
                <span className="text-xs text-text-secondary">Status</span>
                <div className="h-2 w-2 rounded-full bg-text-tertiary" id="statusDot" />
              </div>
            </motion.div>
            <motion.h1
              variants={fadeUp}
              custom={1}
              className="mb-2 text-3xl font-bold tracking-tight"
            >
              <span className="text-primary">//</span> Calibration Tool
            </motion.h1>
            <motion.p
              variants={fadeUp}
              custom={2}
              className="max-w-xl text-sm text-text-secondary"
            >
              Connect your SenseBoard via USB to get started. The tool will
              collect capacitive data from each node and generate a calibration
              profile.
            </motion.p>
          </motion.div>
        </div>
      </section>

      {/* Main Tool Area */}
      <section className="bg-surface-raised">
        <div className="mx-auto max-w-6xl px-6 py-8">
          <div className="grid gap-6 lg:grid-cols-[1.4fr_0.6fr]">
            {/* Step 1: Data Collection */}
            <motion.div
              initial="hidden"
              animate="visible"
              variants={fadeUp}
              custom={0}
              className="rounded-2xl border-2 border-border border-t-[3px] border-t-primary bg-surface p-6"
            >
              <div className="mb-3 flex items-center gap-3">
                <span className="rounded-full bg-primary px-3 py-0.5 text-[11px] font-semibold uppercase tracking-widest text-white">
                  Step 01
                </span>
                <span className="text-lg font-semibold">Data Collection</span>
              </div>

              <div className="mb-4 h-px bg-border" />

              <div className="mb-4 flex flex-wrap items-center gap-3">
                <label className="text-xs font-medium uppercase tracking-wider text-text-secondary">
                  Nodes
                </label>
                <input
                  type="number"
                  defaultValue={20}
                  min={1}
                  max={1000}
                  className="w-20 rounded-lg border-2 border-border bg-surface-inset px-3 py-2 text-sm font-semibold focus:border-primary focus:outline-none focus:ring-2 focus:ring-primary/20"
                />
                <button className="rounded-lg border-2 border-primary bg-primary px-4 py-2 text-xs font-semibold uppercase tracking-widest text-white transition-all hover:-translate-y-0.5 hover:bg-primary-dark hover:shadow-lg hover:shadow-primary/20">
                  Start Capture
                </button>
              </div>

              <p className="mb-3 text-sm text-text-secondary">
                Connect the SenseBoard and click Start Capture to begin
                collecting data from each node.
              </p>

              {/* Progress bar placeholder */}
              <div className="mb-4 flex items-center gap-3">
                <span className="text-[11px] font-medium uppercase tracking-wider text-text-tertiary">
                  Progress
                </span>
                <div className="flex flex-1 gap-1">
                  {Array.from({ length: 20 }).map((_, i) => (
                    <div
                      key={i}
                      className="h-1.5 flex-1 rounded-full border border-border bg-surface-inset"
                    />
                  ))}
                </div>
              </div>

              {/* Chart placeholder */}
              <div className="flex min-h-[300px] items-center justify-center rounded-xl border border-border bg-surface-inset p-8">
                <div className="text-center">
                  <div className="mb-3 text-4xl text-text-tertiary">◎</div>
                  <p className="text-sm text-text-tertiary">
                    Sensor data will appear here during capture
                  </p>
                  <p className="mt-1 text-xs text-text-tertiary">
                    Connect SenseBoard via USB to begin
                  </p>
                </div>
              </div>
            </motion.div>

            {/* Step 2: Node Detection */}
            <motion.div
              initial="hidden"
              animate="visible"
              variants={fadeUp}
              custom={1}
              className="rounded-2xl border-2 border-border border-t-[3px] border-t-secondary bg-surface p-6 opacity-40"
            >
              <div className="mb-3 flex items-center gap-3">
                <span className="rounded-full bg-secondary px-3 py-0.5 text-[11px] font-semibold uppercase tracking-widest text-white">
                  Step 02
                </span>
                <span className="text-lg font-semibold">Node Detection</span>
              </div>

              <div className="mb-4 h-px bg-border" />

              <button
                disabled
                className="mb-4 rounded-lg border-2 border-border bg-surface-inset px-4 py-2 text-xs font-semibold uppercase tracking-widest text-text-tertiary opacity-60"
              >
                Start Detection
              </button>

              <div className="flex flex-1 flex-col gap-3">
                <span className="text-[11px] font-medium uppercase tracking-widest text-text-tertiary">
                  Detected Node
                </span>
                <div className="flex flex-1 items-center justify-center rounded-xl border-2 border-dashed border-border bg-surface-inset p-12 text-6xl font-bold text-text-tertiary">
                  —
                </div>
              </div>
            </motion.div>
          </div>

          {/* Info note */}
          <motion.div
            initial="hidden"
            animate="visible"
            variants={fadeUp}
            custom={2}
            className="mt-6 rounded-xl border-2 border-dashed border-secondary/30 bg-secondary/5 p-6"
          >
            <p className="mb-1 text-sm font-semibold text-secondary">
              How does calibration work?
            </p>
            <p className="text-sm leading-relaxed text-text-secondary">
              During data collection, touch each node in sequence for 5 seconds.
              The tool records capacitive sensor values and calculates average
              readings per node. These averages become reference values for
              real-time detection. During detection, incoming sensor values are
              compared against references using a buffered consensus algorithm
              that requires 80% agreement over the last 10 readings.
            </p>
          </motion.div>
        </div>
      </section>
    </div>
  );
}
