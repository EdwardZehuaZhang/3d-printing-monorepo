import { NextRequest, NextResponse } from "next/server";
import { execFile } from "child_process";
import { promisify } from "util";
import { readFile, writeFile } from "fs/promises";
import path from "path";

const execFileAsync = promisify(execFile);

const ARDUINO_CLI = "/opt/homebrew/bin/arduino-cli";
const FQBN = "arduino:renesas_uno:unor4wifi";
const SKETCH_PATH = path.resolve(
  process.cwd(),
  "../arduino/four_node_sensor/four_node_sensor.ino"
);

async function detectPort(): Promise<string | null> {
  const { stdout } = await execFileAsync(ARDUINO_CLI, [
    "board",
    "list",
    "--format",
    "json",
  ]);
  const boards = JSON.parse(stdout);

  // arduino-cli v1 returns an array of detected_ports
  const ports = boards.detected_ports ?? boards;
  for (const entry of ports) {
    const port = entry.port;
    const matchingBoards = entry.matching_boards ?? [];
    if (
      matchingBoards.some((b: { fqbn?: string }) => b.fqbn === FQBN) ||
      port?.label?.includes("usbmodem")
    ) {
      return port.address;
    }
  }
  return null;
}

/**
 * Generate evenly-spaced cumulative resistance values for N nodes.
 * For 4 nodes the original was {3.0, 13.0, 19.0, 27.0} — roughly
 * linear spacing across ~27kΩ total. We scale total resistance with
 * node count and space nodes evenly along the trace.
 */
function generateNodeResKohm(n: number): string {
  const perNodeK = 7.0; // ~7kΩ per node segment (matches original average)
  const values = Array.from({ length: n }, (_, i) =>
    ((i + 1) * perNodeK).toFixed(1)
  );
  return `{${values.join(", ")}}`;
}

/**
 * Patch NUM_NODES and nodeResKohm in the sketch source to match the
 * requested node count. Returns the original source for restoration.
 */
async function patchSketch(numNodes: number): Promise<string> {
  const original = await readFile(SKETCH_PATH, "utf-8");

  let patched = original.replace(
    /^#define NUM_NODES \d+$/m,
    `#define NUM_NODES ${numNodes}`
  );

  patched = patched.replace(
    /^const float nodeResKohm\[NUM_NODES\]\s*=\s*\{[^}]*\};$/m,
    `const float nodeResKohm[NUM_NODES] = ${generateNodeResKohm(numNodes)};`
  );

  await writeFile(SKETCH_PATH, patched, "utf-8");
  return original;
}

export async function POST(request: NextRequest) {
  let originalSketch: string | null = null;

  try {
    // Parse numNodes from request body (default 4)
    let numNodes = 4;
    try {
      const body = await request.json();
      if (body.numNodes && Number.isInteger(body.numNodes) && body.numNodes >= 1 && body.numNodes <= 24) {
        numNodes = body.numNodes;
      }
    } catch {
      // no body or invalid JSON — use default
    }

    // 1. Detect the board
    const port = await detectPort();
    if (!port) {
      return NextResponse.json(
        { error: "No Arduino Uno R4 WiFi found. Is it plugged in?" },
        { status: 404 }
      );
    }

    // 2. Patch sketch with requested node count
    originalSketch = await patchSketch(numNodes);

    // 3. Compile
    const compileResult = await execFileAsync(
      ARDUINO_CLI,
      ["compile", "--fqbn", FQBN, SKETCH_PATH],
      { timeout: 120_000 }
    );

    // 4. Upload
    const uploadResult = await execFileAsync(
      ARDUINO_CLI,
      ["upload", "--fqbn", FQBN, "--port", port, SKETCH_PATH],
      { timeout: 120_000 }
    );

    return NextResponse.json({
      success: true,
      port,
      numNodes,
      compile: compileResult.stdout + compileResult.stderr,
      upload: uploadResult.stdout + uploadResult.stderr,
    });
  } catch (err: unknown) {
    const message =
      err instanceof Error ? err.message : "Flash failed";
    const stderr =
      err && typeof err === "object" && "stderr" in err
        ? (err as { stderr: string }).stderr
        : "";
    return NextResponse.json(
      { error: message, details: stderr },
      { status: 500 }
    );
  } finally {
    // Always restore the original sketch
    if (originalSketch !== null) {
      await writeFile(SKETCH_PATH, originalSketch, "utf-8").catch(() => {});
    }
  }
}
