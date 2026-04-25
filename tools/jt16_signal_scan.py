#!/usr/bin/env python3
# Run:
#   python3 jt16_signal_scan.py --port /dev/ttyUSB0

import argparse
import os
import select
import string
import time
from dataclasses import dataclass

from jt16_serial_probe import choose_jt16_port, open_raw_serial


DEFAULT_BAUDS = [9600, 115200, 460800, 921600, 1500000, 2000000, 3000000, 5000000]
JT16_MAGIC = b"\xee\xff\x01\x08"


@dataclass
class ProbeResult:
    baud: int
    bytes_seen: int
    chunks_seen: int
    printable_ratio: float
    jt16_magic_hits: int
    nmea_hits: int
    sample_hex: str
    sample_ascii: str


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Probe a serial port at several baud rates and report what kind of traffic "
            "is present. This is useful when you are not sure whether the JT16 RS485 "
            "stream or a slower UART/API lane is the one currently connected."
        )
    )
    parser.add_argument("--port", default="auto")
    parser.add_argument(
        "--baud",
        type=int,
        nargs="*",
        help="Optional baud list to test. Defaults to common JT16/UART rates.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=1.5,
        help="Seconds to listen at each baud. Default: 1.5.",
    )
    parser.add_argument(
        "--sample-bytes",
        type=int,
        default=48,
        help="How many bytes to keep for the sample preview.",
    )
    return parser.parse_args()


def preview_ascii(data: bytes) -> str:
    rendered = []
    for byte in data:
        char = chr(byte)
        rendered.append(char if char in string.printable and char not in "\r\n\t\x0b\x0c" else ".")
    return "".join(rendered)


def printable_ratio(data: bytes) -> float:
    if not data:
        return 0.0
    printable_count = sum(chr(byte) in string.printable for byte in data)
    return printable_count / len(data)


def classify(result: ProbeResult) -> str:
    if result.bytes_seen == 0:
        return "no activity"
    if result.jt16_magic_hits > 0:
        return "looks like JT16 RS485 point-cloud traffic"
    if result.nmea_hits > 0 or result.printable_ratio > 0.7:
        return "looks like ASCII/UART traffic"
    return "binary traffic seen, but not JT16 point-cloud magic"


def probe_once(port: str, baud: int, duration_s: float, sample_bytes: int) -> ProbeResult:
    fd = open_raw_serial(port, baud)
    deadline_s = time.time() + max(duration_s, 0.2)
    bytes_seen = 0
    chunks_seen = 0
    sample = bytearray()
    jt16_magic_hits = 0
    nmea_hits = 0

    try:
        while time.time() < deadline_s:
            readable, _, _ = select.select([fd], [], [], 0.2)
            if not readable:
                continue
            try:
                chunk = os.read(fd, 8192)
            except BlockingIOError:
                continue
            if not chunk:
                continue
            chunks_seen += 1
            bytes_seen += len(chunk)
            jt16_magic_hits += chunk.count(JT16_MAGIC)
            nmea_hits += chunk.count(b"$GP") + chunk.count(b"$GN")
            if len(sample) < sample_bytes:
                remaining = sample_bytes - len(sample)
                sample.extend(chunk[:remaining])
    finally:
        os.close(fd)

    sample_bytes_raw = bytes(sample)
    return ProbeResult(
        baud=baud,
        bytes_seen=bytes_seen,
        chunks_seen=chunks_seen,
        printable_ratio=printable_ratio(sample_bytes_raw),
        jt16_magic_hits=jt16_magic_hits,
        nmea_hits=nmea_hits,
        sample_hex=sample_bytes_raw.hex(" ") if sample_bytes_raw else "",
        sample_ascii=preview_ascii(sample_bytes_raw) if sample_bytes_raw else "",
    )


def main():
    args = parse_args()
    port = choose_jt16_port(args.port)
    if not os.path.exists(port):
        raise SystemExit(f"{port} does not exist. Check /dev/jt16_usb or /dev/ttyUSB* first.")

    bauds = args.baud if args.baud else DEFAULT_BAUDS
    print(f"Scanning {port} for JT16/UART traffic across {len(bauds)} baud rates.")
    print("A healthy JT16 RS485 point-cloud stream should show the magic header 'ee ff 01 08'.")
    print()

    results: list[ProbeResult] = []
    for baud in bauds:
        try:
            result = probe_once(port, baud, args.duration, args.sample_bytes)
        except SystemExit as exc:
            print(f"{baud:>8} baud | unsupported here | {exc}")
            continue
        results.append(result)
        print(
            f"{baud:>8} baud | bytes={result.bytes_seen:<6} chunks={result.chunks_seen:<4} "
            f"class={classify(result)}"
        )
        if result.sample_hex:
            print(f"           sample_hex:   {result.sample_hex}")
            print(f"           sample_ascii: {result.sample_ascii}")

    if not results:
        raise SystemExit("No baud rates could be tested on this machine.")

    best = max(results, key=lambda item: (item.jt16_magic_hits, item.bytes_seen))
    print()
    print("Best candidate:")
    print(
        f"- baud={best.baud}"
        f" bytes={best.bytes_seen}"
        f" jt16_magic_hits={best.jt16_magic_hits}"
        f" class={classify(best)}"
    )

    if all(result.bytes_seen == 0 for result in results):
        print("- No serial data arrived at any tested baud rate.")
        print("- That usually means the lidar's data path is still not reaching this USB adapter.")


if __name__ == "__main__":
    main()
