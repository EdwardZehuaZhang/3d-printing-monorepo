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

// Calibration config: per-node average capacitive values
type CalibrationConfig = Record<string, number>;

export default function CalibratePage() {
  const [numNodes, setNumNodes] = useState(4);
  const [baudRate, setBaudRate] = useState(115200);

  // Flash state
  const [isFlashing, setIsFlashing] = useState(false);
  const [flashStatus, setFlashStatus] = useState<string | null>(null);

  // Collection state
  const [isCollecting, setIsCollecting] = useState(false);
  const [currentNode, setCurrentNode] = useState(-1);
  const [collectingData, setCollectingData] = useState<
    { node: number; time: number; value: number }[]
  >([]);
  const startTimeRef = useRef(0);
  const intervalRef = useRef<ReturnType<typeof setInterval> | null>(null);
  const [collectionComplete, setCollectionComplete] = useState(false);

  // Detection state
  const [isDetecting, setIsDetecting] = useState(false);
  const [detectedNode, setDetectedNode] = useState<number | null>(null);
  const [config, setConfig] = useState<CalibrationConfig | null>(null);
  const capBufferRef = useRef<number[]>([]);
  const nodeBufferRef = useRef<number[]>([]);
  const prevNodeRef = useRef(-1);

  // Live sensor value for chart area
  const [liveValues, setLiveValues] = useState<number[]>([]);
  const MAX_LIVE = 100;

  // Serial monitor log
  const [serialLog, setSerialLog] = useState<string[]>([]);
  const MAX_LOG = 200;

  // Current collecting node ref (for use inside serial callback)
  const currentNodeRef = useRef(-1);
  const modeRef = useRef<"idle" | "collection" | "detection">("idle");
  const collectingDataRef = useRef<
    { node: number; time: number; value: number }[]
  >([]);

  const handleSerialData = useCallback(
    (raw: string) => {
      // Log all serial output to the serial monitor
      setSerialLog((prev) => {
        const next = [...prev, raw];
        return next.length > MAX_LOG ? next.slice(-MAX_LOG) : next;
      });

      // Try to parse as a plain number (wifi_connection sketch)
      // or extract the last number from debug lines (four_node_sensor sketch)
      let value = NaN;
      const plain = parseFloat(raw);
      if (!isNaN(plain) && raw.trim() === String(plain)) {
        // Plain numeric line (e.g. "1234")
        value = plain;
      } else {
        // Try to extract a numeric value — look for known patterns
        // e.g. "sum=1234" or last number on the line
        const sumMatch = raw.match(/sum[=:]\s*(-?\d+)/);
        if (sumMatch) {
          value = parseFloat(sumMatch[1]);
        } else {
          // Fall back to last number on the line
          const nums = raw.match(/-?\d+\.?\d*/g);
          if (nums && nums.length > 0) {
            value = parseFloat(nums[nums.length - 1]);
          }
        }
      }
      if (isNaN(value)) return;

      // Update live chart
      setLiveValues((prev) => {
        const next = [...prev, value];
        return next.length > MAX_LIVE ? next.slice(-MAX_LIVE) : next;
      });

      if (modeRef.current === "collection") {
        const time = performance.now() - startTimeRef.current;
        collectingDataRef.current.push({
          node: currentNodeRef.current,
          time,
          value,
        });
      } else if (modeRef.current === "detection" && config) {
        capBufferRef.current.push(value);
        if (capBufferRef.current.length > 10) {
          capBufferRef.current.shift();
          const avg =
            capBufferRef.current.reduce((s, v) => s + v, 0) /
            capBufferRef.current.length;

          // Find closest node
          let bestNode = -1;
          let minDiff = Infinity;
          for (const [nodeStr, refVal] of Object.entries(config)) {
            const diff = Math.abs(refVal - avg);
            if (diff < minDiff) {
              minDiff = diff;
              bestNode = parseInt(nodeStr);
            }
          }

          nodeBufferRef.current.push(bestNode);
          if (nodeBufferRef.current.length > 10) {
            nodeBufferRef.current.shift();

            // Consensus: 80% agreement
            const counts: Record<number, number> = {};
            for (const n of nodeBufferRef.current) {
              counts[n] = (counts[n] || 0) + 1;
            }
            const sorted = Object.entries(counts).sort(
              ([, a], [, b]) => b - a
            );
            const [topNode, topCount] = sorted[0];

            if (topCount > 8) {
              setDetectedNode(parseInt(topNode));
              prevNodeRef.current = parseInt(topNode);
              capBufferRef.current.length = 0;
            } else {
              setDetectedNode(prevNodeRef.current === -1 ? null : prevNodeRef.current);
            }
          }
        }
      }
    },
    [config]
  );

  const serial = useWebSerial({
    baudRate,
    onData: handleSerialData,
  });

  // Process collected data into config
  const processData = useCallback(
    (data: { node: number; time: number; value: number }[]) => {
      const cutThreshold = 1500; // ms
      const byNode: Record<number, { times: number[]; values: number[] }> = {};

      for (const d of data) {
        if (!byNode[d.node]) byNode[d.node] = { times: [], values: [] };
        byNode[d.node].times.push(d.time);
        byNode[d.node].values.push(d.value);
      }

      const cfg: CalibrationConfig = {};
      for (const nodeStr in byNode) {
        const node = parseInt(nodeStr);
        const { times, values } = byNode[node];
        const lastPrevTime =
          node > 0 && byNode[node - 1]
            ? byNode[node - 1].times[byNode[node - 1].times.length - 1]
            : 0;

        let sum = 0;
        let count = 0;
        for (let i = 0; i < times.length; i++) {
          if (times[i] >= lastPrevTime + cutThreshold) {
            sum += values[i];
            count++;
          }
        }
        cfg[nodeStr] = count > 0 ? sum / count : 0;
      }

      setConfig(cfg);
      return cfg;
    },
    []
  );

  // Start data collection
  const startCollection = useCallback(async () => {
    if (serial.status !== "connected") return;

    setIsCollecting(true);
    setCollectionComplete(false);
    setConfig(null);
    setLiveValues([]);
    collectingDataRef.current = [];
    currentNodeRef.current = -1;
    setCurrentNode(-1);
    startTimeRef.current = performance.now();
    modeRef.current = "collection";

    // Send start signal to Arduino
    await serial.write("1");

    let nodeCounter = -1;
    intervalRef.current = setInterval(() => {
      nodeCounter++;
      currentNodeRef.current = nodeCounter;
      setCurrentNode(nodeCounter);

      if (nodeCounter >= numNodes) {
        // Collection done
        clearInterval(intervalRef.current!);
        intervalRef.current = null;
        modeRef.current = "idle";
        setIsCollecting(false);
        setCollectionComplete(true);
        setCollectingData([...collectingDataRef.current]);

        const cfg = processData(collectingDataRef.current);
        // Download config as JSON
        const blob = new Blob([JSON.stringify(cfg, null, 2)], {
          type: "application/json",
        });
        const url = URL.createObjectURL(blob);
        const a = document.createElement("a");
        a.href = url;
        a.download = "calibration-config.json";
        a.click();
        URL.revokeObjectURL(url);
      }
    }, 5000);
  }, [serial, numNodes, processData]);

  // Start detection
  const startDetection = useCallback(async () => {
    if (serial.status !== "connected" || !config) return;

    capBufferRef.current = [];
    nodeBufferRef.current = [];
    prevNodeRef.current = -1;
    setDetectedNode(null);
    setIsDetecting(true);
    modeRef.current = "detection";

    await serial.write("1");
  }, [serial, config]);

  const stopDetection = useCallback(() => {
    setIsDetecting(false);
    modeRef.current = "idle";
  }, []);

  // Cleanup interval on unmount
  useEffect(() => {
    return () => {
      if (intervalRef.current) clearInterval(intervalRef.current);
    };
  }, []);

  const statusInfo = STATUS_CONFIG[serial.status];
  const step2Active = collectionComplete && config !== null;

  // Sparkline for live values
  const sparklinePath = (() => {
    if (liveValues.length < 2) return "";
    const min = Math.min(...liveValues);
    const max = Math.max(...liveValues);
    const range = max - min || 1;
    const w = 100;
    const h = 100;
    return liveValues
      .map((v, i) => {
        const x = (i / (liveValues.length - 1)) * w;
        const y = h - ((v - min) / range) * h;
        return `${i === 0 ? "M" : "L"} ${x.toFixed(1)} ${y.toFixed(1)}`;
      })
      .join(" ");
  })();

  // Node colors for the collected data chart
  const NODE_COLORS = [
    "#1245A8", "#2563eb", "#16a34a", "#d97706",
    "#dc2626", "#7c3aed", "#0891b2", "#db2777",
    "#65a30d", "#ea580c", "#4f46e5", "#0d9488",
  ];

  // Build per-node SVG paths from collected data
  const collectedChartData = (() => {
    if (collectingData.length < 2) return null;

    const allValues = collectingData.map((d) => d.value);
    const minVal = Math.min(...allValues);
    const maxVal = Math.max(...allValues);
    const rangeVal = maxVal - minVal || 1;
    const maxTime = Math.max(...collectingData.map((d) => d.time));
    const minTime = Math.min(...collectingData.map((d) => d.time));
    const rangeTime = maxTime - minTime || 1;

    // Group by node
    const byNode: Record<number, { time: number; value: number }[]> = {};
    for (const d of collectingData) {
      if (!byNode[d.node]) byNode[d.node] = [];
      byNode[d.node].push(d);
    }

    const paths: { node: number; color: string; d: string }[] = [];
    for (const nodeStr in byNode) {
      const node = parseInt(nodeStr);
      const points = byNode[node];
      if (points.length < 2) continue;

      const pathD = points
        .map((p, i) => {
          const x = ((p.time - minTime) / rangeTime) * 100;
          const y = 100 - ((p.value - minVal) / rangeVal) * 100;
          return `${i === 0 ? "M" : "L"} ${x.toFixed(1)} ${y.toFixed(1)}`;
        })
        .join(" ");

      paths.push({
        node,
        color: NODE_COLORS[((node % NODE_COLORS.length) + NODE_COLORS.length) % NODE_COLORS.length],
        d: pathD,
      });
    }

    return paths;
  })();

  return (
    <div>
      {/* Header */}
      <section className="border-b-2 border-border">
        <div className="mx-auto max-w-6xl px-6 py-12">
          <motion.div initial="hidden" animate="visible" className="flex items-start justify-between gap-6">
            {/* Left: title & description */}
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
                Connect your SenseBoard via USB to get started. The tool will
                collect capacitive data from each node and generate a calibration
                profile.
              </motion.p>
            </div>

            {/* Right: connect / disconnect button */}
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
                        // Arduino resets after flash — give it time to boot
                        await new Promise((r) => setTimeout(r, 2500));
                        setFlashStatus("Connecting to serial...");
                        await serial.connect();
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
                  value={numNodes}
                  onChange={(e) =>
                    setNumNodes(
                      Math.max(1, Math.min(1000, parseInt(e.target.value) || 1))
                    )
                  }
                  min={1}
                  max={1000}
                  disabled={isCollecting}
                  className="w-20 rounded-lg border-2 border-border bg-surface-inset px-3 py-2 text-sm font-semibold focus:border-primary focus:outline-none focus:ring-2 focus:ring-primary/20 disabled:opacity-50"
                />
                <button
                  onClick={startCollection}
                  disabled={serial.status !== "connected" || isCollecting}
                  className="rounded-lg border-2 border-primary bg-primary px-4 py-2 text-xs font-semibold uppercase tracking-widest text-white transition-all hover:-translate-y-0.5 hover:bg-primary-dark hover:shadow-lg hover:shadow-primary/20 disabled:opacity-50 disabled:hover:translate-y-0"
                >
                  {isCollecting ? "Capturing..." : "Start Capture"}
                </button>
              </div>

              {/* Status message */}
              <AnimatePresence mode="wait">
                {isCollecting && currentNode >= 0 && (
                  <motion.p
                    key={`collecting-${currentNode}`}
                    initial={{ opacity: 0, y: -8 }}
                    animate={{ opacity: 1, y: 0 }}
                    exit={{ opacity: 0, y: 8 }}
                    className="mb-3 text-sm font-semibold text-primary"
                  >
                    Touch Node {currentNode} now ({currentNode + 1} of{" "}
                    {numNodes})
                  </motion.p>
                )}
                {isCollecting && currentNode === -1 && (
                  <motion.p
                    key="initializing"
                    initial={{ opacity: 0 }}
                    animate={{ opacity: 1 }}
                    className="mb-3 text-sm text-text-secondary"
                  >
                    Initializing... release all nodes
                  </motion.p>
                )}
                {!isCollecting && collectionComplete && (
                  <motion.p
                    key="complete"
                    initial={{ opacity: 0 }}
                    animate={{ opacity: 1 }}
                    className="mb-3 text-sm font-semibold text-success"
                  >
                    Collection complete — config downloaded
                  </motion.p>
                )}
                {!isCollecting && !collectionComplete && serial.status === "connected" && (
                  <motion.p
                    key="waiting"
                    initial={{ opacity: 0 }}
                    animate={{ opacity: 1 }}
                    className="mb-3 text-sm text-text-secondary"
                  >
                    Ready — click Start Capture to begin
                  </motion.p>
                )}
              </AnimatePresence>

              {/* Progress bar */}
              <div className="mb-4 flex items-center gap-3">
                <span className="text-[11px] font-medium uppercase tracking-wider text-text-tertiary">
                  Progress
                </span>
                <div className="flex flex-1 gap-1">
                  {Array.from({ length: numNodes }).map((_, i) => (
                    <div
                      key={i}
                      className={`h-1.5 flex-1 rounded-full border transition-colors duration-300 ${
                        isCollecting && i < currentNode
                          ? "border-primary bg-primary"
                          : isCollecting && i === currentNode
                            ? "border-primary bg-primary/40 animate-pulse"
                            : collectionComplete
                              ? "border-success bg-success"
                              : "border-border bg-surface-inset"
                      }`}
                    />
                  ))}
                </div>
              </div>

              {/* Sensor chart */}
              <div className="flex min-h-[300px] items-center justify-center rounded-xl border border-border bg-surface-inset p-4">
                {collectionComplete && collectedChartData && collectedChartData.length > 0 ? (
                  /* Collected data chart — shown after collection completes */
                  <div className="h-full w-full">
                    <div className="mb-2 flex items-center justify-between">
                      <span className="text-[10px] font-medium uppercase tracking-widest text-text-tertiary">
                        Calibration Data
                      </span>
                      <div className="flex flex-wrap gap-3">
                        {collectedChartData.map(({ node, color }) => (
                          <span
                            key={node}
                            className="flex items-center gap-1 text-[10px] font-medium"
                          >
                            <span
                              className="inline-block h-2 w-2 rounded-full"
                              style={{ backgroundColor: color }}
                            />
                            {node === -1 ? "Baseline" : `Node ${node}`}
                          </span>
                        ))}
                      </div>
                    </div>
                    <svg
                      viewBox="0 0 100 100"
                      preserveAspectRatio="none"
                      className="h-[260px] w-full"
                    >
                      {collectedChartData.map(({ node, color, d }) => (
                        <path
                          key={node}
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
                  /* Live sparkline — shown during collection */
                  <div className="h-full w-full">
                    <div className="mb-2 flex items-center justify-between">
                      <span className="text-[10px] font-medium uppercase tracking-widest text-text-tertiary">
                        Live Sensor Data
                      </span>
                      <span className="font-mono text-sm font-bold text-primary">
                        {liveValues[liveValues.length - 1]?.toFixed(0)}
                      </span>
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
                  /* Empty state */
                  <div className="text-center">
                    <div className="mb-3 text-4xl text-text-tertiary">
                      {serial.status === "connected" ? "~" : "\u25CE"}
                    </div>
                    <p className="text-sm text-text-tertiary">
                      {serial.status === "connected"
                        ? "Waiting for sensor data..."
                        : "Sensor data will appear here during capture"}
                    </p>
                  </div>
                )}
              </div>
            </motion.div>

            {/* Step 2: Node Detection */}
            <motion.div
              initial="hidden"
              animate="visible"
              variants={fadeUp}
              custom={1}
              className={`rounded-2xl border-2 border-border border-t-[3px] border-t-secondary bg-surface p-6 transition-opacity duration-500 ${
                step2Active ? "opacity-100" : "opacity-40"
              }`}
            >
              <div className="mb-3 flex items-center gap-3">
                <span className="rounded-full bg-secondary px-3 py-0.5 text-[11px] font-semibold uppercase tracking-widest text-white">
                  Step 02
                </span>
                <span className="text-lg font-semibold">Node Detection</span>
              </div>

              <div className="mb-4 h-px bg-border" />

              {isDetecting ? (
                <button
                  onClick={stopDetection}
                  className="mb-4 rounded-lg border-2 border-danger bg-danger/10 px-4 py-2 text-xs font-semibold uppercase tracking-widest text-danger transition-all hover:-translate-y-0.5 hover:bg-danger/20"
                >
                  Stop Detection
                </button>
              ) : (
                <button
                  onClick={startDetection}
                  disabled={!step2Active || serial.status !== "connected"}
                  className="mb-4 rounded-lg border-2 border-secondary bg-secondary px-4 py-2 text-xs font-semibold uppercase tracking-widest text-white transition-all hover:-translate-y-0.5 hover:bg-secondary-dark hover:shadow-lg hover:shadow-secondary/20 disabled:opacity-50 disabled:hover:translate-y-0"
                >
                  Start Detection
                </button>
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
              <div className="max-h-[200px] overflow-y-auto rounded-lg bg-[#1a1a2e] p-3 font-mono text-xs leading-relaxed text-green-400">
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
              During data collection, touch each node in sequence for 5 seconds.
              The tool records capacitive sensor values and calculates average
              readings per node. These averages become reference values for
              real-time detection. During detection, incoming sensor values are
              compared against references using a buffered consensus algorithm
              that requires 80% agreement over the last 10 readings.
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
