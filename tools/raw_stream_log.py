"""Log the raw IMU stream (firmware 0.7.3+i2cbat.3raw and later).

    python tools/raw_stream_log.py --port COM4 --minutes 20 --interval 200

Turns the stream on over the serial console (STREAM ON <ms>), writes the
[RAW] lines to raw/<date>_<port>.csv and turns it off at the end. SlimeVR
must be closed, since it holds the COM port.

Columns: t_pc, sensor, millis, gx, gy, gz, ax, ay, az, tempC, n
(gx..az in raw sensor LSB; on the LSM6DSV at 1000 dps, 1 LSB = 0.035 deg/s)
"""

import argparse
import csv
import sys
import time
from pathlib import Path

import serial

DEST = Path(__file__).resolve().parent.parent / "raw"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--minutes", type=float, default=20)
    ap.add_argument("--interval", type=int, default=200, help="ms between lines")
    ap.add_argument("--label", default="")
    args = ap.parse_args()
    sys.stdout.reconfigure(encoding="utf-8")

    DEST.mkdir(exist_ok=True)
    name = f"{time.strftime('%Y%m%d-%H%M%S')}_{args.port}{('_' + args.label) if args.label else ''}.csv"
    path = DEST / name

    s = serial.Serial()
    s.port, s.baudrate, s.timeout = args.port, 115200, 0.5
    s.dtr = s.rts = False
    s.open()
    s.write(f"STREAM ON {args.interval}\n".encode())

    fim = time.time() + args.minutes * 60
    n = 0
    buf = b""
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t_pc", "sensor", "millis", "gx", "gy", "gz", "ax", "ay", "az", "tempC", "n"])
        try:
            while time.time() < fim:
                buf += s.read(4096)
                partes_linhas = buf.split(b"\n")
                buf = partes_linhas.pop()
                for linha in partes_linhas:
                    txt = linha.decode("utf-8", "replace").strip()
                    if not txt.startswith("[RAW]"):
                        continue
                    fields = txt.split()[1:]
                    if len(fields) != 10:
                        continue
                    w.writerow([round(time.time(), 3), *fields])
                    n += 1
                    if n % 50 == 0:
                        f.flush()
                        print(f"  {n} rows  temp {fields[8]}C  gyro {fields[2]} {fields[3]} {fields[4]}", flush=True)
        finally:
            s.write(b"STREAM OFF\n")
            time.sleep(0.3)
            s.close()
    print(f"\n{n} rows saved to {path}")


if __name__ == "__main__":
    main()
