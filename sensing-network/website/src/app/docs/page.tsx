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

const sections = [
  {
    id: "overview",
    title: "Overview",
    content: `SenseKit lets you create touch-sensitive 3D printed objects. The kit combines three components:

**Conductive Filament** — PLA infused with carbon particles that creates resistive traces inside your print.

**Rhino/Grasshopper Plugin** — A computational design tool that generates optimized electrode networks on any 3D surface.

**SenseBoard** — An Arduino Uno R4 WiFi pre-loaded with capacitive sensing firmware that reads touch input from your print.`,
  },
  {
    id: "requirements",
    title: "What You Need",
    content: null,
    list: [
      "An FDM 3D printer (any brand, 1.75mm filament)",
      "SenseKit conductive filament",
      "SenseBoard (or Arduino Uno R4 WiFi)",
      "USB-C cable",
      "A computer with a web browser (for calibration)",
      "Optional: Rhino 7/8 + Grasshopper (for the plugin)",
      "Optional: Multi-material printing capability",
    ],
  },
  {
    id: "printing",
    title: "Step 1: Printing",
    content: `### Slicer Settings

Use your normal PLA profile with these adjustments for the conductive filament:

| Setting | Value |
|---------|-------|
| Nozzle temp | 200–220°C |
| Bed temp | 50–60°C |
| Print speed | 40–60 mm/s |
| Layer height | 0.2mm |
| Retraction | Standard PLA settings |
| Fan | 100% after first layer |

### Single-Material Prints

If you're printing with conductive filament only, the entire surface becomes touch-sensitive. This works great for simple objects where you want to detect any touch.

### Multi-Material Prints

For selective touch zones, use multi-material printing:
1. Assign the conductive filament to electrode regions
2. Use regular PLA for non-sensing areas
3. Ensure electrode paths connect to exposed pads at the base for wiring

### Design Tips

- **Minimum trace width**: 2mm for reliable conductivity
- **Electrode spacing**: At least 3mm between separate sensing zones
- **Connection pads**: Leave flat, exposed areas at the print base for attaching clips
- **Orientation**: Print electrodes in the XY plane when possible for best conductivity`,
  },
  {
    id: "wiring",
    title: "Step 2: Wiring",
    content: `### Connecting Your Print to the SenseBoard

1. **Locate connection pads** — Find the exposed conductive areas at the base of your print
2. **Attach clip connectors** — Use the included alligator or pogo-pin clips to connect each electrode pad to the SenseBoard
3. **Connect to pins** — Each clip goes to one of the SenseBoard's analog input pins (A0–A19)
4. **USB connection** — Plug the SenseBoard into your computer via USB-C

### Pin Mapping

| SenseBoard Pin | Electrode |
|----------------|-----------|
| A0 | Node 1 |
| A1 | Node 2 |
| A2 | Node 3 |
| ... | ... |
| A19 | Node 20 |

### Troubleshooting Connections

- **No signal**: Check that the clip makes firm contact with the conductive surface
- **Noisy readings**: Shorten clip wires, or add a ground connection
- **Cross-talk**: Ensure electrode paths don't touch each other`,
  },
  {
    id: "calibration",
    title: "Step 3: Calibration",
    content: `### Using the Calibration Tool

The web-based calibration tool maps each touch zone on your print to a node ID.

1. **Open the tool** — Go to the [Calibrate](/calibrate) page
2. **Set node count** — Enter the number of sensing zones on your print
3. **Start capture** — Press each zone in sequence when prompted. Hold for 5 seconds per zone.
4. **Review data** — The tool shows a chart of capacitive values for each zone
5. **Save calibration** — The tool generates a config file mapping zones to node IDs

### How Calibration Works

During capture, the tool records the capacitive sensor value when you touch each zone. After collection:
- It calculates the average value for each zone (ignoring the first 1.5s of transition)
- These averages become the reference values for real-time detection
- During detection, incoming values are compared to references using a buffered consensus algorithm

### Tips for Good Calibration

- **Consistent pressure**: Touch each zone with the same finger and pressure
- **Wait for stability**: Don't move between zones too quickly
- **Environment**: Calibrate in the same environment you'll use the object
- **Re-calibrate**: If detection seems off, re-run calibration`,
  },
  {
    id: "detection",
    title: "Step 4: Detection",
    content: `### Real-Time Touch Detection

After calibration, the SenseBoard continuously detects which zone is being touched.

#### Via the Web Tool
Use the "Node Detection" panel on the [Calibrate](/calibrate) page to see live detection.

#### Via Serial
Read the serial output at 9600 baud:

\`\`\`
// Arduino Serial Monitor or any serial reader
// Output: raw capacitive value (integer)
1523
1489
1501
...
\`\`\`

#### Via WiFi
The SenseBoard creates a WiFi access point. Connect to it and read data via TCP:

\`\`\`
Host: 192.168.4.1
Port: 1021
\`\`\`

### Writing Custom Firmware

The SenseBoard firmware is open-source Arduino code. You can modify it to:
- Change the sensing algorithm
- Add LED feedback
- Send data over different protocols
- Integrate with home automation systems`,
  },
  {
    id: "plugin",
    title: "Rhino Plugin Guide",
    content: `### Installation

1. Download the plugin from your SenseKit account
2. Open Rhino → Tools → Package Manager
3. Install the \`.yak\` package file
4. Restart Rhino

### Grasshopper Components

The plugin adds these components to Grasshopper:

- **SenseRegion** — Define a touch zone on a surface
- **ElectrodeNetwork** — Generate electrode paths connecting all zones
- **SenseExport** — Export multi-material STL/3MF with correct assignments

### Basic Workflow

1. Import or model your 3D object in Rhino
2. Open Grasshopper and add a **SenseRegion** component
3. Reference your surface and draw touch zone boundaries
4. Connect to **ElectrodeNetwork** — it generates optimal wiring paths
5. Preview the result — electrodes appear as colored paths on your model
6. Use **SenseExport** to generate your print file with material assignments

### Advanced Features

- **Auto-routing**: The algorithm finds shortest paths while maintaining minimum spacing
- **Resistance estimation**: Preview the expected resistance of each trace
- **Zone merging**: Combine multiple surfaces into one sensing zone
- **Parametric control**: All parameters are Grasshopper-native for parametric design`,
  },
];

export default function DocsPage() {
  return (
    <div>
      {/* Header */}
      <section className="border-b-2 border-border">
        <div className="mx-auto max-w-6xl px-6 py-16 md:py-24">
          <motion.div initial="hidden" animate="visible">
            <motion.p
              variants={fadeUp}
              custom={0}
              className="mb-2 text-xs font-semibold uppercase tracking-widest text-primary"
            >
              Documentation
            </motion.p>
            <motion.h1
              variants={fadeUp}
              custom={1}
              className="mb-4 text-4xl font-bold tracking-tight md:text-5xl"
            >
              Getting started with{" "}
              <span className="text-primary">SenseKit</span>
            </motion.h1>
            <motion.p
              variants={fadeUp}
              custom={2}
              className="max-w-2xl text-lg text-text-secondary"
            >
              Everything you need to go from box to working touch sensor.
            </motion.p>
          </motion.div>
        </div>
      </section>

      {/* Two-column layout: sidebar + content */}
      <section className="border-b-2 border-border">
        <div className="mx-auto flex max-w-6xl gap-0">
          {/* Sidebar nav */}
          <aside className="sticky top-[73px] hidden h-[calc(100vh-73px)] w-64 shrink-0 overflow-y-auto border-r-2 border-border p-6 md:block">
            <h4 className="mb-4 text-xs font-semibold uppercase tracking-widest text-text-tertiary">
              On this page
            </h4>
            <nav className="flex flex-col gap-1">
              {sections.map((s) => (
                <a
                  key={s.id}
                  href={`#${s.id}`}
                  className="rounded-lg px-3 py-2 text-sm text-text-secondary no-underline transition-colors hover:bg-surface-inset hover:text-text-primary"
                >
                  {s.title}
                </a>
              ))}
            </nav>
          </aside>

          {/* Content */}
          <div className="min-w-0 flex-1 px-6 py-12 md:px-12">
            {sections.map((section, si) => (
              <motion.div
                key={section.id}
                id={section.id}
                initial="hidden"
                whileInView="visible"
                viewport={{ once: true, margin: "-50px" }}
                variants={fadeUp}
                custom={0}
                className={`${si > 0 ? "mt-16 border-t-2 border-border pt-16" : ""}`}
              >
                <h2 className="mb-6 text-2xl font-bold">{section.title}</h2>

                {section.content && (
                  <div className="prose-sm max-w-none">
                    {section.content.split("\n\n").map((block, bi) => {
                      // Handle code blocks
                      if (block.startsWith("```")) {
                        const lines = block.split("\n");
                        const code = lines.slice(1, -1).join("\n");
                        return (
                          <pre
                            key={bi}
                            className="my-4 overflow-x-auto rounded-xl border-2 border-border bg-surface-inset p-4 text-sm"
                          >
                            <code>{code}</code>
                          </pre>
                        );
                      }

                      // Handle tables
                      if (block.includes("|") && block.includes("---")) {
                        const rows = block
                          .split("\n")
                          .filter((r) => !r.includes("---"));
                        const headers = rows[0]
                          .split("|")
                          .filter(Boolean)
                          .map((h) => h.trim());
                        const data = rows.slice(1).map((r) =>
                          r
                            .split("|")
                            .filter(Boolean)
                            .map((c) => c.trim())
                        );
                        return (
                          <div
                            key={bi}
                            className="my-4 overflow-x-auto rounded-xl border-2 border-border"
                          >
                            <table className="w-full text-sm">
                              <thead>
                                <tr className="border-b-2 border-border bg-surface-inset">
                                  {headers.map((h) => (
                                    <th
                                      key={h}
                                      className="px-4 py-3 text-left font-semibold"
                                    >
                                      {h}
                                    </th>
                                  ))}
                                </tr>
                              </thead>
                              <tbody>
                                {data.map((row, ri) => (
                                  <tr
                                    key={ri}
                                    className="border-b border-border last:border-0"
                                  >
                                    {row.map((cell, ci) => (
                                      <td
                                        key={ci}
                                        className="px-4 py-3 text-text-secondary"
                                      >
                                        {cell}
                                      </td>
                                    ))}
                                  </tr>
                                ))}
                              </tbody>
                            </table>
                          </div>
                        );
                      }

                      // Handle headings
                      if (block.startsWith("### ")) {
                        return (
                          <h3 key={bi} className="mb-3 mt-8 text-lg font-bold">
                            {block.replace("### ", "")}
                          </h3>
                        );
                      }
                      if (block.startsWith("#### ")) {
                        return (
                          <h4
                            key={bi}
                            className="mb-2 mt-6 text-base font-bold"
                          >
                            {block.replace("#### ", "")}
                          </h4>
                        );
                      }

                      // Handle lists
                      if (block.startsWith("- ")) {
                        return (
                          <ul
                            key={bi}
                            className="my-3 flex flex-col gap-2 pl-4"
                          >
                            {block.split("\n").map((line, li) => (
                              <li
                                key={li}
                                className="text-sm leading-relaxed text-text-secondary"
                              >
                                <span className="mr-2 text-primary">▸</span>
                                {line.replace(/^- /, "").replace(/\*\*(.*?)\*\*/g, "$1")}
                              </li>
                            ))}
                          </ul>
                        );
                      }

                      // Handle numbered lists
                      if (/^\d+\. /.test(block)) {
                        return (
                          <ol
                            key={bi}
                            className="my-3 flex flex-col gap-2 pl-4"
                          >
                            {block.split("\n").map((line, li) => (
                              <li
                                key={li}
                                className="text-sm leading-relaxed text-text-secondary"
                              >
                                {line.replace(/^\d+\. /, "").replace(/\*\*(.*?)\*\*/g, "$1")}
                              </li>
                            ))}
                          </ol>
                        );
                      }

                      // Regular paragraph
                      return (
                        <p
                          key={bi}
                          className="my-3 text-sm leading-relaxed text-text-secondary"
                          dangerouslySetInnerHTML={{
                            __html: block
                              .replace(/\*\*(.*?)\*\*/g, "<strong class='text-text-primary font-semibold'>$1</strong>")
                              .replace(
                                /\[([^\]]+)\]\(([^)]+)\)/g,
                                '<a href="$2" class="text-primary hover:underline">$1</a>'
                              )
                              .replace(/`([^`]+)`/g, '<code class="rounded bg-surface-inset px-1.5 py-0.5 text-xs border border-border">$1</code>'),
                          }}
                        />
                      );
                    })}
                  </div>
                )}

                {section.list && (
                  <ul className="flex flex-col gap-3">
                    {section.list.map((item) => (
                      <li
                        key={item}
                        className="flex items-start gap-3 text-sm text-text-secondary"
                      >
                        <span className="mt-0.5 text-primary">▸</span>
                        {item}
                      </li>
                    ))}
                  </ul>
                )}
              </motion.div>
            ))}
          </div>
        </div>
      </section>

      {/* Help CTA */}
      <section className="bg-text-primary">
        <div className="mx-auto max-w-6xl px-6 py-12 text-center">
          <h2 className="mb-3 text-2xl font-bold text-white">Need help?</h2>
          <p className="mb-6 text-sm text-text-tertiary">
            Join the community Discord or reach out via email for support.
          </p>
          <div className="flex flex-wrap justify-center gap-4">
            <a
              href="#"
              className="rounded-xl border-2 border-primary bg-primary px-6 py-3 text-sm font-bold uppercase tracking-widest text-white no-underline transition-all hover:-translate-y-0.5 hover:bg-primary-dark"
            >
              Join Discord
            </a>
            <Link
              href="/calibrate"
              className="rounded-xl border-2 border-border px-6 py-3 text-sm font-bold uppercase tracking-widest text-white no-underline transition-all hover:-translate-y-0.5 hover:bg-white/10"
            >
              Open Calibration Tool
            </Link>
          </div>
        </div>
      </section>
    </div>
  );
}
