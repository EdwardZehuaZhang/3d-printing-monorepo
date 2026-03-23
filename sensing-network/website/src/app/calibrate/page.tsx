"use client";

import { useState, useRef, useCallback, useEffect } from "react";
import { motion, AnimatePresence } from "framer-motion";
import { useWebSerial, ConnectionStatus } from "./useWebSerial";

const fadeUp = {
  hidden: { opacity: 0, y: 24 },
  visible: (i: number) => ({
    opacity: 1,
    y: 0,
    transition: { delay: i * 0.1, duration: 0.5, ease: "easeOut" as const },
  }),
};

const STATUS_CONFIG: Record<
  ConnectionStatus,
  { color: string; label: string }
> = {
  disconnected: { color: "bg-text-tertiary", label: "Disconnected" },
  connecting: { color: "bg-warning animate-pulse", label: "Connecting..." },
  connected: { color: "bg-success", label: "Connected" },
  error: { color: "bg-danger", label: "Error" },
};

// Arduino calibration phases — matched to sketch output
type CalPhase =
  | "idle"           // before calibration starts
  | "phase0-notouch" // Phase 0: don't touch (4s)
  | "phase0-touch"   // Phase 0: touch any node (4s)
  | "phase0-enter"   // Phase 0: waiting for ENTER
  | "phase1"         // Phase 1: baseline measurement
  | "phase2"         // Phase 2: per-node calibration
  | "phase3"         // Phase 3: calibration results
  | "live";          // Live detection loop

const NODE_COLORS = [
  "#1245A8", "#2563eb", "#16a34a", "#d97706",
  "#dc2626", "#7c3aed", "#0891b2", "#db2777",
];

export default function CalibratePage() {
  const [baudRate, setBaudRate] = useState(115200);

  // Flash state
  const [isFlashing, setIsFlashing] = useState(false);
  const [flashStatus, setFlashStatus] = useState<string | null>(null);

  // Calibration phase tracking (parsed from Arduino output)
  const [phase, setPhase] = useState<CalPhase>("idle");
  const phaseRef = useRef<CalPhase>("idle");

  // Phase 2: which node the Arduino is currently calibrating (1-based)
  const [calNode, setCalNode] = useState(0);
  const [calNodeStatus, setCalNodeStatus] = useState<
    "waiting" | "touching" | "collecting" | "releasing" | "done"
  >("waiting");
  const [calNodesCompleted, setCalNodesCompleted] = useState(0);
  const numNodesRef = useRef(4); // default, updated from Arduino output

  // Live detection: parsed from Arduino output
  const [detectedNode, setDetectedNode] = useState<number | null>(null);

  // Live sensor values for chart (sum values)
  const [liveValues, setLiveValues] = useState<{ time: number; value: number; node?: number }[]>([]);
  const MAX_LIVE = 200;
  const startTimeRef = useRef(0);

  // Serial monitor log
  const [serialLog, setSerialLog] = useState<string[]>([]);
  const MAX_LOG = 300;
  const serialLogRef = useRef<HTMLDivElement>(null);

  // Auto-scroll serial monitor
  useEffect(() => {
    if (serialLogRef.current) {
      serialLogRef.current.scrollTop = serialLogRef.current.scrollHeight;
    }
  }, [serialLog]);

  const handleSerialData = useCallback((raw: string) => {
    // Log to serial monitor
    setSerialLog((prev) => {
      const next = [...prev, raw];
      return next.length > MAX_LOG ? next.slice(-MAX_LOG) : next;
    });

    // ── Phase detection from Arduino output ──────────────────────
    if (raw.includes("[PHASE 0]")) {
      phaseRef.current = "phase0-notouch";
      setPhase("phase0-notouch");
      startTimeRef.current = performance.now();
      setLiveValues([]);
      setCalNodesCompleted(0);
      setDetectedNode(null);
      return;
    }

    if (raw.includes("TOUCH ANY NODE FIRMLY")) {
      phaseRef.current = "phase0-touch";
      setPhase("phase0-touch");
      return;
    }

    if (raw.includes("Press ENTER to continue")) {
      phaseRef.current = "phase0-enter";
      setPhase("phase0-enter");
      return;
    }

    if (raw.includes("[PHASE 1]")) {
      phaseRef.current = "phase1";
      setPhase("phase1");
      return;
    }

    if (raw.includes("[PHASE 2]")) {
      phaseRef.current = "phase2";
      setPhase("phase2");
      setCalNode(0);
      setCalNodeStatus("waiting");
      return;
    }

    if (raw.includes("[PHASE 3]")) {
      phaseRef.current = "phase3";
      setPhase("phase3");
      return;
    }

    if (raw.includes("LIVE DETECTION")) {
      phaseRef.current = "live";
      setPhase("live");
      setDetectedNode(null);
      return;
    }

    // ── Phase 2: per-node calibration tracking ───────────────────
    const touchHoldMatch = raw.match(/TOUCH AND HOLD\s+Node\s+(\d+)/);
    if (touchHoldMatch) {
      const n = parseInt(touchHoldMatch[1]);
      setCalNode(n);
      setCalNodeStatus("waiting");
      // Update numNodes if we see a higher node number
      if (n > numNodesRef.current) numNodesRef.current = n;
      return;
    }

    if (raw.includes("Touch detected!")) {
      setCalNodeStatus("collecting");
      return;
    }

    if (raw.includes("collecting Node")) {
      setCalNodeStatus("collecting");
    }

    if (raw.includes("RELEASE NOW")) {
      setCalNodeStatus("releasing");
      return;
    }

    if (raw.includes("Released.")) {
      setCalNodeStatus("done");
      setCalNodesCompleted((prev) => prev + 1);
      return;
    }

    // ── Live detection: parse pressed/released/switch ────────────
    const pressedMatch = raw.match(/>>> Node (\d+) PRESSED <<</);
    if (pressedMatch) {
      setDetectedNode(parseInt(pressedMatch[1]));
      return;
    }

    if (raw.includes("[RELEASE]")) {
      setDetectedNode(null);
      return;
    }

    const switchMatch = raw.match(/\[SWITCH → Node (\d+)\]/);
    if (switchMatch) {
      setDetectedNode(parseInt(switchMatch[1]));
      return;
    }

    // ── Parse sum= values for live chart ─────────────────────────
    const sumMatch = raw.match(/sum=\s*(-?\d+)/);
    if (sumMatch) {
      const value = parseInt(sumMatch[1]);
      const time = performance.now() - startTimeRef.current;

      // Parse current node from [NX] in debug output
      let node: number | undefined;
      const nodeMatch = raw.match(/\[N(\d+)\]/);
      if (nodeMatch) node = parseInt(nodeMatch[1]);

      setLiveValues((prev) => {
        const next = [...prev, { time, value, node }];
        return next.length > MAX_LIVE ? next.slice(-MAX_LIVE) : next;
      });
    }
  }, []);

  const serial = useWebSerial({
    baudRate,
    onData: handleSerialData,
  });

  // Send ENTER to Arduino (for Phase 0 continue prompt)
  // readSerialLine() requires a non-empty trimmed string before \n
  const sendEnter = useCallback(async () => {
    await serial.write("1\n");
  }, [serial]);

  // ── Chart rendering ──────────────────────────────────────────────
  const sparklinePath = (() => {
    if (liveValues.length < 2) return "";
    const vals = liveValues.map((v) => v.value);
    const min = Math.min(...vals);
    const max = Math.max(...vals);
    const range = max - min || 1;
    const w = 100;
    const h = 100;
    return liveValues
      .map((v, i) => {
        const x = (i / (liveValues.length - 1)) * w;
        const y = h - ((v.value - min) / range) * h;
        return `${i === 0 ? "M" : "L"} ${x.toFixed(1)} ${y.toFixed(1)}`;
      })
      .join(" ");
  })();

  // Build per-node colored chart paths (for live detection mode)
  const nodeChartPaths = (() => {
    if (phaseRef.current !== "live" || liveValues.length < 2) return null;

    const vals = liveValues.map((v) => v.value);
    const min = Math.min(...vals);
    const max = Math.max(...vals);
    const range = max - min || 1;

    // Group consecutive points by node
    const segments: { node: number | undefined; points: { x: number; y: number }[] }[] = [];
    let currentSeg: typeof segments[0] | null = null;

    liveValues.forEach((v, i) => {
      const x = (i / (liveValues.length - 1)) * 100;
      const y = 100 - ((v.value - min) / range) * 100;
      const pt = { x, y };

      if (!currentSeg || currentSeg.node !== v.node) {
        // Carry over last point for continuity
        if (currentSeg && currentSeg.points.length > 0) {
          currentSeg = { node: v.node, points: [currentSeg.points[currentSeg.points.length - 1], pt] };
        } else {
          currentSeg = { node: v.node, points: [pt] };
        }
        segments.push(currentSeg);
      } else {
        currentSeg.points.push(pt);
      }
    });

    return segments
      .filter((s) => s.points.length >= 2)
      .map((s, i) => ({
        key: i,
        color: s.node ? NODE_COLORS[((s.node - 1) % NODE_COLORS.length + NODE_COLORS.length) % NODE_COLORS.length] : "var(--color-text-tertiary)",
        d: s.points
          .map((p, j) => `${j === 0 ? "M" : "L"} ${p.x.toFixed(1)} ${p.y.toFixed(1)}`)
          .join(" "),
      }));
  })();

  const latestSum = liveValues.length > 0 ? liveValues[liveValues.length - 1].value : null;
  const numNodes = numNodesRef.current;

  // Phase-specific status message
  const phaseMessage = (() => {
    switch (phase) {
      case "idle":
        return serial.status === "connected"
          ? "Arduino is starting up... waiting for calibration to begin"
          : null;
      case "phase0-notouch":
        return "Phase 0: DON'T TOUCH the sensor — measuring baseline signal";
      case "phase0-touch":
        return "Phase 0: TOUCH ANY NODE firmly and hold for 4 seconds";
      case "phase0-enter":
        return "Phase 0 complete — press Continue to proceed";
      case "phase1":
        return "Phase 1: Measuring baseline — HANDS OFF completely";
      case "phase2":
        if (calNodeStatus === "waiting")
          return `Phase 2: TOUCH AND HOLD Node ${calNode}`;
        if (calNodeStatus === "collecting")
          return `Phase 2: Keep touching Node ${calNode} — collecting data (7s)`;
        if (calNodeStatus === "releasing")
          return `Phase 2: RELEASE Node ${calNode} now`;
        if (calNodeStatus === "done")
          return `Phase 2: Node ${calNode} complete`;
        return "Phase 2: Per-node calibration";
      case "phase3":
        return "Phase 3: Calibration results — review quality below";
      case "live":
        return "Live detection active — touch nodes to test";
      default:
        return null;
    }
  })();

  const isCalibrating = ["phase0-notouch", "phase0-touch", "phase0-enter", "phase1", "phase2", "phase3"].includes(phase);

  // ── Full progress steps ────────────────────────────────────────────
  // Each step: { label, status: "done" | "active" | "pending" }
  const progressSteps = (() => {
    const steps: { label: string; status: "done" | "active" | "pending" }[] = [];

    // Phase 0: Signal Test
    const p0Done = ["phase1", "phase2", "phase3", "live"].includes(phase);
    const p0Active = ["phase0-notouch", "phase0-touch", "phase0-enter"].includes(phase);
    steps.push({ label: "Signal Test", status: p0Done ? "done" : p0Active ? "active" : "pending" });

    // Phase 1: Baseline
    const p1Done = ["phase2", "phase3", "live"].includes(phase);
    const p1Active = phase === "phase1";
    steps.push({ label: "Baseline", status: p1Done ? "done" : p1Active ? "active" : "pending" });

    // Phase 2: one step per node
    for (let i = 1; i <= numNodes; i++) {
      let status: "done" | "active" | "pending" = "pending";
      if (phase === "phase2") {
        if (i < calNode || (i === calNode && calNodeStatus === "done")) {
          status = "done";
        } else if (i === calNode && calNodeStatus !== "done") {
          status = "active";
        }
      } else if (["phase3", "live"].includes(phase)) {
        status = "done";
      }
      steps.push({ label: `N${i}`, status });
    }

    // Phase 3 / Live
    const resultsDone = phase === "live";
    const resultsActive = phase === "phase3";
    steps.push({ label: "Live", status: resultsDone ? "done" : resultsActive ? "active" : "pending" });

    return steps;
  })();

  return (
    <div>
      {/* Header */}
      <section className="border-b-2 border-border">
        <div className="mx-auto max-w-6xl px-6 py-12">
          <motion.div initial="hidden" animate="visible" className="flex items-start justify-between gap-6">
            <div>
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
                className="max-w-2xl text-sm text-text-secondary"
              >
                Connect your SenseBoard via USB. The Arduino runs the full
                calibration sequence — this page guides you through each step
                and visualizes the results.
              </motion.p>
            </div>

            {/* Connect / disconnect */}
            <motion.div variants={fadeUp} custom={1} className="flex shrink-0 flex-col items-end gap-2 pt-6">
              <div className="flex items-center gap-2">
                {serial.status !== "connected" && !isFlashing && (
                  <select
                    value={baudRate}
                    onChange={(e) => setBaudRate(parseInt(e.target.value))}
                    className="appearance-none rounded-lg border-2 border-border bg-surface-inset bg-[url('data:image/svg+xml;charset=utf-8,%3Csvg%20xmlns%3D%22http%3A%2F%2Fwww.w3.org%2F2000%2Fsvg%22%20width%3D%2212%22%20height%3D%2212%22%20viewBox%3D%220%200%2012%2012%22%3E%3Cpath%20fill%3D%22%236b7280%22%20d%3D%22M3%205l3%203%203-3%22%2F%3E%3C%2Fsvg%3E')] bg-[length:12px] bg-[position:right_8px_center] bg-no-repeat py-2 pl-3 pr-8 text-xs font-semibold focus:border-primary focus:outline-none"
                  >
                    <option value={9600}>9600</option>
                    <option value={115200}>115200</option>
                  </select>
                )}
                {serial.status === "connected" ? (
                  <button
                    onClick={serial.disconnect}
                    className="rounded-lg border-2 border-danger bg-danger/10 px-4 py-2 text-xs font-semibold uppercase tracking-widest text-danger transition-all hover:-translate-y-0.5 hover:bg-danger/20"
                  >
                    Disconnect
                  </button>
                ) : (
                  <button
                    onClick={async () => {
                      // 1. Request port NOW (in user gesture context)
                      const port = await serial.requestPort();
                      if (!port) return; // user cancelled picker

                      setIsFlashing(true);
                      setFlashStatus("Compiling & uploading sketch...");
                      try {
                        const res = await fetch("/api/flash", { method: "POST" });
                        const data = await res.json();
                        if (!res.ok) {
                          setFlashStatus(`Flash failed: ${data.error}`);
                          setIsFlashing(false);
                          return;
                        }
                        setFlashStatus("Flashed! Waiting for board to restart...");
                        await new Promise((r) => setTimeout(r, 2500));
                        setFlashStatus("Connecting to serial...");
                        setPhase("idle");
                        phaseRef.current = "idle";
                        setSerialLog([]);
                        setLiveValues([]);
                        setDetectedNode(null);
                        // 2. Open the already-selected port (no gesture needed)
                        await serial.openPort();
                        setFlashStatus(null);
                      } catch (err) {
                        setFlashStatus(
                          `Error: ${err instanceof Error ? err.message : "Unknown error"}`
                        );
                      } finally {
                        setIsFlashing(false);
                      }
                    }}
                    disabled={!serial.isSupported || serial.status === "connecting" || isFlashing}
                    className="rounded-lg border-2 border-primary bg-primary px-4 py-2 text-xs font-semibold uppercase tracking-widest text-white transition-all hover:-translate-y-0.5 hover:bg-primary-dark hover:shadow-lg hover:shadow-primary/20 disabled:opacity-50 disabled:hover:translate-y-0"
                  >
                    {isFlashing
                      ? "Flashing..."
                      : serial.status === "connecting"
                        ? "Connecting..."
                        : "Flash & Connect"}
                  </button>
                )}
              </div>

              {isFlashing && flashStatus && (
                <span className="text-xs font-medium text-warning animate-pulse">
                  {flashStatus}
                </span>
              )}

              {!isFlashing && flashStatus && (
                <span className="text-xs text-danger">{flashStatus}</span>
              )}

              {!serial.isSupported && (
                <span className="text-xs text-danger">
                  Web Serial not supported. Use Chrome or Edge.
                </span>
              )}

              {serial.error && (
                <span className="text-xs text-danger">{serial.error}</span>
              )}
            </motion.div>
          </motion.div>
        </div>
      </section>

      {/* Main Tool Area */}
      <section className="bg-surface-raised">
        <div className="mx-auto max-w-6xl px-6 py-8">
          <div className="grid gap-6 lg:grid-cols-[1.4fr_0.6fr]">
            {/* Left: Calibration Progress & Chart */}
            <motion.div
              initial="hidden"
              animate="visible"
              variants={fadeUp}
              custom={0}
              className="rounded-2xl border-2 border-border border-t-[3px] border-t-primary bg-surface p-6"
            >
              <div className="mb-3 flex items-center gap-3">
                <span className={`rounded-full px-3 py-0.5 text-[11px] font-semibold uppercase tracking-widest text-white ${
                  phase === "live" ? "bg-success" : isCalibrating ? "bg-primary animate-pulse" : "bg-text-tertiary"
                }`}>
                  {phase === "live" ? "Live" : isCalibrating ? "Calibrating" : "Waiting"}
                </span>
                <span className="text-lg font-semibold">
                  {phase === "live" ? "Live Detection" : "Calibration"}
                </span>
              </div>

              <div className="mb-4 h-px bg-border" />

              {/* Phase status message */}
              <AnimatePresence mode="wait">
                {phaseMessage && (
                  <motion.div
                    key={phase + calNode + calNodeStatus}
                    initial={{ opacity: 0, y: -8 }}
                    animate={{ opacity: 1, y: 0 }}
                    exit={{ opacity: 0, y: 8 }}
                    className="mb-4"
                  >
                    <p className={`text-sm font-semibold ${
                      phase === "phase0-touch" || (phase === "phase2" && calNodeStatus === "waiting")
                        ? "text-warning"
                        : phase === "phase0-notouch" || phase === "phase1" || (phase === "phase2" && calNodeStatus === "releasing")
                          ? "text-danger"
                          : phase === "live" || phase === "phase3"
                            ? "text-success"
                            : "text-primary"
                    }`}>
                      {phaseMessage}
                    </p>

                    {/* Continue button for Phase 0 */}
                    {phase === "phase0-enter" && (
                      <button
                        onClick={sendEnter}
                        className="mt-2 rounded-lg border-2 border-primary bg-primary px-4 py-2 text-xs font-semibold uppercase tracking-widest text-white transition-all hover:-translate-y-0.5 hover:bg-primary-dark hover:shadow-lg hover:shadow-primary/20"
                      >
                        Continue
                      </button>
                    )}
                  </motion.div>
                )}
              </AnimatePresence>

              {/* Progress bar — full calibration pipeline */}
              {serial.status === "connected" && phase !== "idle" && (
                <div className="mb-4">
                  <div className="mb-1.5 flex gap-0.5">
                    {progressSteps.map((step, i) => (
                      <div
                        key={i}
                        className={`h-1.5 flex-1 rounded-full transition-colors duration-300 ${
                          step.status === "done"
                            ? "bg-primary"
                            : step.status === "active"
                              ? "bg-primary animate-pulse"
                              : "bg-border"
                        }`}
                      />
                    ))}
                  </div>
                  <div className="flex gap-0.5">
                    {progressSteps.map((step, i) => (
                      <span
                        key={i}
                        className={`flex-1 text-center text-[9px] font-medium uppercase tracking-wider ${
                          step.status === "done"
                            ? "text-primary"
                            : step.status === "active"
                              ? "text-primary"
                              : "text-text-tertiary"
                        }`}
                      >
                        {step.label}
                      </span>
                    ))}
                  </div>
                </div>
              )}

              {/* Sensor chart */}
              <div className="flex min-h-[300px] items-center justify-center rounded-xl border border-border bg-surface-inset p-4">
                {phase === "live" && nodeChartPaths && nodeChartPaths.length > 0 ? (
                  <div className="h-full w-full">
                    <div className="mb-2 flex items-center justify-between">
                      <span className="text-[10px] font-medium uppercase tracking-widest text-text-tertiary">
                        Live Sensor Data (sum)
                      </span>
                      {latestSum !== null && (
                        <span className="font-mono text-sm font-bold text-primary">
                          {latestSum}
                        </span>
                      )}
                    </div>
                    <svg
                      viewBox="0 0 100 100"
                      preserveAspectRatio="none"
                      className="h-[260px] w-full"
                    >
                      {nodeChartPaths.map(({ key, color, d }) => (
                        <path
                          key={key}
                          d={d}
                          fill="none"
                          stroke={color}
                          strokeWidth="0.4"
                          vectorEffect="non-scaling-stroke"
                        />
                      ))}
                    </svg>
                  </div>
                ) : liveValues.length > 1 ? (
                  <div className="h-full w-full">
                    <div className="mb-2 flex items-center justify-between">
                      <span className="text-[10px] font-medium uppercase tracking-widest text-text-tertiary">
                        Live Sensor Data (sum)
                      </span>
                      {latestSum !== null && (
                        <span className="font-mono text-sm font-bold text-primary">
                          {latestSum}
                        </span>
                      )}
                    </div>
                    <svg
                      viewBox="0 0 100 100"
                      preserveAspectRatio="none"
                      className="h-[260px] w-full"
                    >
                      <path
                        d={sparklinePath}
                        fill="none"
                        stroke="var(--color-primary)"
                        strokeWidth="0.5"
                        vectorEffect="non-scaling-stroke"
                      />
                    </svg>
                  </div>
                ) : (
                  <div className="text-center">
                    <div className="mb-3 text-4xl text-text-tertiary">
                      {serial.status === "connected" ? "~" : "\u25CE"}
                    </div>
                    <p className="text-sm text-text-tertiary">
                      {serial.status === "connected"
                        ? "Waiting for sensor data..."
                        : "Sensor data will appear here after connecting"}
                    </p>
                  </div>
                )}
              </div>
            </motion.div>

            {/* Right: Node Detection */}
            <motion.div
              initial="hidden"
              animate="visible"
              variants={fadeUp}
              custom={1}
              className={`rounded-2xl border-2 border-border border-t-[3px] border-t-secondary bg-surface p-6 transition-opacity duration-500 ${
                phase === "live" ? "opacity-100" : "opacity-40"
              }`}
            >
              <div className="mb-3 flex items-center gap-3">
                <span className={`rounded-full px-3 py-0.5 text-[11px] font-semibold uppercase tracking-widest text-white ${
                  phase === "live" ? "bg-secondary" : "bg-text-tertiary"
                }`}>
                  {phase === "live" ? "Active" : "Waiting"}
                </span>
                <span className="text-lg font-semibold">Node Detection</span>
              </div>

              <div className="mb-4 h-px bg-border" />

              {phase !== "live" && (
                <p className="mb-4 text-xs text-text-tertiary">
                  Detection activates automatically after calibration completes.
                </p>
              )}

              <div className="flex flex-1 flex-col gap-3">
                <span className="text-[11px] font-medium uppercase tracking-widest text-text-tertiary">
                  Detected Node
                </span>
                <div className="flex flex-1 items-center justify-center rounded-xl border-2 border-dashed border-border bg-surface-inset p-12">
                  <AnimatePresence mode="wait">
                    <motion.span
                      key={detectedNode ?? "none"}
                      initial={{ opacity: 0, scale: 0.8 }}
                      animate={{ opacity: 1, scale: 1 }}
                      exit={{ opacity: 0, scale: 0.8 }}
                      transition={{ duration: 0.2 }}
                      className={`text-6xl font-bold ${
                        detectedNode !== null
                          ? "text-secondary"
                          : "text-text-tertiary"
                      }`}
                    >
                      {detectedNode !== null ? detectedNode : "\u2014"}
                    </motion.span>
                  </AnimatePresence>
                </div>
              </div>
            </motion.div>
          </div>

          {/* Serial Monitor */}
          {serial.status === "connected" && (
            <motion.div
              initial="hidden"
              animate="visible"
              variants={fadeUp}
              custom={2}
              className="mt-6 rounded-2xl border-2 border-border border-t-[3px] border-t-text-tertiary bg-surface p-4"
            >
              <div className="mb-2 flex items-center justify-between">
                <span className="text-[11px] font-medium uppercase tracking-widest text-text-tertiary">
                  Serial Monitor ({baudRate} baud)
                </span>
                <button
                  onClick={() => setSerialLog([])}
                  className="text-[10px] font-medium uppercase tracking-widest text-text-tertiary hover:text-text-secondary"
                >
                  Clear
                </button>
              </div>
              <div
                ref={serialLogRef}
                className="max-h-[200px] overflow-y-auto rounded-lg bg-[#1a1a2e] p-3 font-mono text-xs leading-relaxed text-green-400"
              >
                {serialLog.length === 0 ? (
                  <span className="text-text-tertiary">
                    Waiting for data from Arduino...
                  </span>
                ) : (
                  serialLog.map((line, i) => (
                    <div key={i} className="whitespace-pre-wrap break-all">
                      {line}
                    </div>
                  ))
                )}
              </div>
            </motion.div>
          )}

          {/* Info note */}
          <motion.div
            initial="hidden"
            animate="visible"
            variants={fadeUp}
            custom={3}
            className="mt-6 rounded-xl border-2 border-dashed border-secondary/30 bg-secondary/5 p-6"
          >
            <p className="mb-1 text-sm font-semibold text-secondary">
              How does calibration work?
            </p>
            <p className="text-sm leading-relaxed text-text-secondary">
              The Arduino runs a 3-phase calibration sequence automatically.
              Phase 0 tests signal strength (don&apos;t touch, then touch any node).
              Phase 1 measures the untouched baseline.
              Phase 2 calibrates each node individually — touch and hold when
              prompted, then release. The Arduino collects bidirectional (fwd/rev)
              data for 7 seconds per node to build a 2D signature. After
              calibration, live detection uses nearest-centroid scoring with
              debouncing for reliable node identification.
            </p>
            <div className="mt-4 h-px bg-secondary/20" />
            <p className="mt-3 text-xs leading-relaxed text-text-tertiary">
              <strong className="text-text-secondary">No Arduino IDE needed:</strong>{" "}
              Clicking &ldquo;Flash &amp; Connect&rdquo; compiles and uploads the
              sensor sketch automatically, then opens a serial connection — all
              from the browser. Supported in Chrome and Edge (version 89+).
            </p>
          </motion.div>
        </div>
      </section>
    </div>
  );
}
