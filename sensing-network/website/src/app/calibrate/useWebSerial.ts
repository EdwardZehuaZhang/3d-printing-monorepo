"use client";

import { useState, useRef, useCallback, useEffect } from "react";

export type ConnectionStatus = "disconnected" | "connecting" | "connected" | "error";

interface UseWebSerialOptions {
  baudRate?: number;
  onData?: (value: string) => void;
}

export function useWebSerial({ baudRate = 9600, onData }: UseWebSerialOptions = {}) {
  const [status, setStatus] = useState<ConnectionStatus>("disconnected");
  const [error, setError] = useState<string | null>(null);
  const [isSupported, setIsSupported] = useState(false);

  useEffect(() => {
    setIsSupported("serial" in navigator);
  }, []);

  const portRef = useRef<SerialPort | null>(null);
  const readerRef = useRef<ReadableStreamDefaultReader<Uint8Array> | null>(null);
  const onDataRef = useRef(onData);
  const readingRef = useRef(false);

  // Keep callback ref up to date without re-triggering effects
  useEffect(() => {
    onDataRef.current = onData;
  }, [onData]);

  const readLoop = useCallback(async (port: SerialPort) => {
    const decoder = new TextDecoder();
    let buffer = "";
    readingRef.current = true;

    console.log("[WebSerial] Read loop started");

    while (port.readable && readingRef.current) {
      const reader = port.readable.getReader();
      readerRef.current = reader;

      try {
        while (readingRef.current) {
          const { value, done } = await reader.read();
          if (done) {
            console.log("[WebSerial] Reader done");
            break;
          }

          const chunk = decoder.decode(value, { stream: true });
          buffer += chunk;

          // Split on newlines — Arduino sends "\r\n" delimited values
          const lines = buffer.split(/\r?\n/);
          // Keep the last incomplete chunk in the buffer
          buffer = lines.pop() || "";

          for (const line of lines) {
            const trimmed = line.trim();
            if (trimmed) {
              onDataRef.current?.(trimmed);
            }
          }
        }
      } catch (err) {
        if (readingRef.current) {
          console.error("[WebSerial] Read error:", err);
          setError(err instanceof Error ? err.message : "Read error");
          setStatus("error");
        }
      } finally {
        reader.releaseLock();
        readerRef.current = null;
      }
    }

    console.log("[WebSerial] Read loop ended");
  }, []);

  const connect = useCallback(async () => {
    if (!isSupported) {
      setError("Web Serial API is not supported in this browser. Use Chrome or Edge.");
      setStatus("error");
      return;
    }

    try {
      setStatus("connecting");
      setError(null);

      console.log("[WebSerial] Opening port picker...");

      // No filter — UNO R4 WiFi uses Renesas chip, not classic Arduino VID
      const port = await navigator.serial.requestPort();

      console.log("[WebSerial] Port selected, opening at", baudRate, "baud...");
      const info = port.getInfo();
      console.log("[WebSerial] Port info:", info);

      await port.open({
        baudRate,
        dataBits: 8,
        parity: "none",
        stopBits: 1,
        flowControl: "none",
      });

      console.log("[WebSerial] Port opened successfully");

      portRef.current = port;
      setStatus("connected");

      // Start reading
      readLoop(port);
    } catch (err) {
      if (err instanceof DOMException && err.name === "NotFoundError") {
        // User cancelled the picker — not an error
        console.log("[WebSerial] User cancelled port picker");
        setStatus("disconnected");
      } else {
        console.error("[WebSerial] Connect error:", err);
        setError(err instanceof Error ? err.message : "Connection failed");
        setStatus("error");
      }
    }
  }, [isSupported, baudRate, readLoop]);

  const disconnect = useCallback(async () => {
    console.log("[WebSerial] Disconnecting...");
    readingRef.current = false;

    if (readerRef.current) {
      try {
        await readerRef.current.cancel();
      } catch {}
      readerRef.current = null;
    }

    if (portRef.current) {
      try {
        await portRef.current.close();
      } catch {}
      portRef.current = null;
    }

    setStatus("disconnected");
    setError(null);
  }, []);

  const write = useCallback(async (data: string) => {
    if (!portRef.current?.writable) return;

    const writer = portRef.current.writable.getWriter();
    try {
      await writer.write(new TextEncoder().encode(data));
    } finally {
      writer.releaseLock();
    }
  }, []);

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      readingRef.current = false;
      readerRef.current?.cancel().catch(() => {});
      portRef.current?.close().catch(() => {});
    };
  }, []);

  return { status, error, isSupported, connect, disconnect, write };
}
