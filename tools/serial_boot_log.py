"""Capture a tracker's boot log over USB (direct serial port).

    python tools/serial_boot_log.py --port COM4 --seconds 15
    python tools/serial_boot_log.py --port COM4 --pre DELCAL --seconds 90   # wipes calibration first

Sends the --pre commands, then REBOOT, and records everything the tracker
prints. The port must be free (SlimeVR closed).
"""

import argparse
import re
import sys
import time
from pathlib import Path

import serial


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--seconds", type=float, default=15)
    ap.add_argument("--pre", action="append", default=[], help="command to send before REBOOT")
    ap.add_argument("--no-reboot", action="store_true")
    args = ap.parse_args()
    sys.stdout.reconfigure(encoding="utf-8")

    s = serial.Serial()
    s.port, s.baudrate, s.timeout = args.port, 115200, 0.2
    s.dtr = s.rts = False
    s.open()
    out = b""
    for cmd in args.pre:
        s.write(cmd.encode() + b"\n")
        end = time.time() + 1.5
        while time.time() < end:
            out += s.read(4096)
    if not args.no_reboot:
        s.write(b"REBOOT\n")
    end = time.time() + args.seconds
    while time.time() < end:
        out += s.read(4096)
    s.close()

    text = out.decode("utf-8", "replace")
    dest = Path(__file__).resolve().parent.parent / "firmware" / f"bootlog_{args.port}_{time.strftime('%Y%m%d-%H%M%S')}.txt"
    dest.write_text(text, encoding="utf-8")
    for line in text.splitlines():
        line = re.sub(r"[^\x20-\x7eÀ-ſ]", "", line)
        if line.startswith("[") and not re.search(r"(?i)wifi|udp|packet|handshake|network|searching", line):
            print(line[:200])
    print(f"\nfull log in {dest}")


if __name__ == "__main__":
    main()
