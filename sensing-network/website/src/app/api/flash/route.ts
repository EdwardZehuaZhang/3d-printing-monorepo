import { NextResponse } from "next/server";
import { execFile } from "child_process";
import { promisify } from "util";
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

export async function POST() {
  try {
    // 1. Detect the board
    const port = await detectPort();
    if (!port) {
      return NextResponse.json(
        { error: "No Arduino Uno R4 WiFi found. Is it plugged in?" },
        { status: 404 }
      );
    }

    // 2. Compile
    const compileResult = await execFileAsync(
      ARDUINO_CLI,
      ["compile", "--fqbn", FQBN, SKETCH_PATH],
      { timeout: 120_000 }
    );

    // 3. Upload
    const uploadResult = await execFileAsync(
      ARDUINO_CLI,
      ["upload", "--fqbn", FQBN, "--port", port, SKETCH_PATH],
      { timeout: 120_000 }
    );

    return NextResponse.json({
      success: true,
      port,
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
  }
}
